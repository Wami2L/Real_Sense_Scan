"""Escaneo texturizado (plan 14): textura UV con las fotos, RSM v2, ráfaga de profundidad, falda de
papel, toma repetida y reprocesado desde las tomas guardadas. Escena sintética con verdad de
referencia."""

import os
import time

import cv2
import numpy as np
import open3d as o3d
import pytest

from live3d.objscan.library import Library
from live3d.objscan.rafaga import change_frac, fuse_burst, fuse_depths
from live3d.objscan.rsm import read_rsm
from live3d.objscan.session import ObjectScanSession, _skirt_mask
from live3d.objscan.sintetico import ObjectScene, orbit_pose


def _shot(sc, k, n=10):
    return sc.render(orbit_pose(10, elevation_deg=30 if k % 2 == 0 else 45), k * 360 / n)


@pytest.fixture(scope="module")
def textured(tmp_path_factory):
    tmp = tmp_path_factory.mktemp("textura")
    sc = ObjectScene()
    lib = Library(str(tmp / "lib"))
    s = ObjectScanSession("Figura", base_dir=str(tmp), library=lib, log=lambda *a: None)
    for k in range(10):
        s.add_snapshot(_shot(sc, k))
    s.finish(background=False)
    assert s.error is None, s.error
    return s, lib


def _render_from_view(lib, man, v):
    """Color del modelo texturizado visto desde la pose de la vista v (raycast + UV)."""
    m = o3d.io.read_triangle_mesh(lib.path(man["id"], "objeto.obj"))
    V, F, UV = np.asarray(m.vertices), np.asarray(m.triangles), np.asarray(m.triangle_uvs)
    tex = cv2.cvtColor(cv2.imread(lib.path(man["id"], "textura.jpg")), cv2.COLOR_BGR2RGB)
    To = np.eye(4)
    To[:3, 3] = man["pivote_desplazado_m"]
    T = To @ v.T
    K = v.snap["intrinsics"]
    h, w = v.snap["color"].shape[:2]
    uu, vv = np.meshgrid(np.arange(w, dtype=np.float64), np.arange(h, dtype=np.float64))
    d = np.stack([(uu - K["ppx"]) / K["fx"], (vv - K["ppy"]) / K["fy"], np.ones_like(uu)], -1) @ T[:3, :3].T
    rays = np.concatenate([np.broadcast_to(T[:3, 3], d.shape), d], -1).astype(np.float32)
    scene = o3d.t.geometry.RaycastingScene()
    scene.add_triangles(o3d.t.geometry.TriangleMesh(o3d.core.Tensor(V.astype(np.float32)),
                                                    o3d.core.Tensor(F.astype(np.int32))))
    hit = scene.cast_rays(o3d.core.Tensor(rays))
    ok = np.isfinite(hit["t_hit"].numpy())
    pid = np.where(ok, hit["primitive_ids"].numpy(), 0)
    b = hit["primitive_uvs"].numpy()
    tuv = UV.reshape(-1, 3, 2)[pid]
    uv = tuv[..., 0, :] * (1 - b[..., 0] - b[..., 1])[..., None] + tuv[..., 1, :] * b[..., 0][..., None] \
        + tuv[..., 2, :] * b[..., 1][..., None]
    S = tex.shape[0]
    col = cv2.remap(tex, np.clip(uv[..., 0] * S, 0, S - 1).astype(np.float32),
                    np.clip((1 - uv[..., 1]) * S, 0, S - 1).astype(np.float32), cv2.INTER_LINEAR)
    return col, ok


def test_textura_cubre_el_objeto_y_guarda_los_archivos(textured):
    s, lib = textured
    man = s.result
    info = man["textura"]
    assert info["texels_vistos"] > 0.9            # todo menos la base apoyada en la hoja
    assert np.allclose(info["ganancias"], 1, atol=0.15)  # la escena sintética no cambia de exposición
    for f in ("textura.jpg", "objeto.obj", "objeto.mtl", "modelo.rsm", "miniatura.jpg"):
        assert os.path.isfile(lib.path(man["id"], f))


def test_textura_reproduce_las_fotos(textured):
    """Visto desde la pose de una toma, el modelo texturizado tiene los colores de la foto."""
    s, lib = textured
    for v in s.views[::3]:
        col, ok = _render_from_view(lib, s.result, v)
        inner = cv2.erode((ok & v.mask).astype(np.uint8), np.ones((9, 9), np.uint8)) > 0
        diff = np.abs(col[inner].astype(float) - v.snap["color"][inner].astype(float)).mean()
        assert inner.sum() > 2000 and diff < 18, (v.label, diff)


def test_rsm_v2_lleva_uv_y_textura(textured):
    s, lib = textured
    r = read_rsm(lib.path(s.result["id"], "modelo.rsm"))
    assert r["version"] == 2
    assert r["uvs"].shape == (len(r["vertices"]), 2)
    assert r["uvs"].min() >= 0 and r["uvs"].max() <= 1
    img = cv2.imdecode(np.frombuffer(r["texture"], np.uint8), cv2.IMREAD_COLOR)
    assert img.shape[:2] == (2048, 2048)


def test_malla_de_blender_se_retexturiza(textured):
    """objeto_limpio.ply (Blender) más nueva -> el RSM se regenera CON textura desde las fotos."""
    s, lib = textured
    oid = s.result["id"]
    mesh = o3d.io.read_triangle_mesh(lib.path(oid, "objeto.ply")).simplify_quadric_decimation(15000)
    time.sleep(0.05)
    o3d.io.write_triangle_mesh(lib.path(oid, "objeto_limpio.ply"), mesh)
    lib.ensure_rsm(oid)
    man = lib.get(oid)
    r = read_rsm(lib.path(oid, "modelo.rsm"))
    assert man["fuente"] == "objeto_limpio.ply"
    assert "texture" in r and man["textura"]["texels_vistos"] > 0.85
    os.remove(lib.path(oid, "objeto_limpio.ply"))


def test_reprocesar_desde_las_tomas_guardadas(textured):
    s, lib = textured
    s2 = ObjectScanSession.from_folder(s.dir, name="Figura (reprocesada)", library=lib, log=lambda *a: None)
    assert len(s2.views) == len(s.views)
    s2.state = "processing"
    s2._process(150000, texture=False)
    assert s2.error is None, s2.error
    assert np.allclose(s2.result["tamano_m"], s.result["tamano_m"], atol=0.005)


def test_toma_desde_la_misma_vista_se_rechaza(tmp_path):
    sc = ObjectScene()
    s = ObjectScanSession("x", base_dir=str(tmp_path), library=Library(str(tmp_path / "lib")),
                          log=lambda *a: None, save_views=False)
    shot = _shot(sc, 0)
    s.add_snapshot(shot)
    with pytest.raises(ValueError, match="misma vista"):
        s.add_snapshot(_shot(sc, 0))
    assert s.novelty_deg(s.views[0].T) < 1
    s.add_snapshot(_shot(sc, 1))                    # girando la hoja sí vale
    assert s.shots == 2


def test_rafaga_reduce_ruido_y_descarta_movidos():
    rng = np.random.default_rng(0)
    truth = np.full((120, 160), 600.0)
    frames = [{"depth": np.round(truth + rng.normal(0, 2.0, truth.shape)).astype(np.uint16),
               "color": np.zeros((120, 160, 3), np.uint8), "depth_scale": 0.001} for _ in range(6)]
    moved = dict(frames[0])
    d = moved["depth"].copy()
    d[:, :40] += 60                                  # 6 cm en un cuarto de la imagen: algo se movió
    moved["depth"] = d
    out = fuse_depths([moved] + frames[1:])
    assert out["burst"] == 5 and out["burst_cambio"][0] > 0.2
    assert np.std(out["depth"].astype(float) - truth) < 0.6 * np.std(frames[0]["depth"].astype(float) - truth)
    assert change_frac(frames[1]["depth"], frames[2]["depth"]) < 0.001
    dual = fuse_burst([{"cam1": f, "cam2": f} for f in frames])
    assert dual["cam1"]["burst"] == 6 and dual["cam2"]["burst"] == 6


def test_falda_de_papel_se_quita():
    """Caja de 10x15x5 cm + "falda" de hoja pegada a su base: fuera la falda, la caja intacta."""
    rng = np.random.default_rng(1)
    box = rng.uniform([-0.05, 0.0, -0.025], [0.05, 0.15, 0.025], (20000, 3))
    side = rng.integers(0, 4, len(box))               # puntos sobre las 4 caras laterales
    box[side == 0, 0], box[side == 1, 0] = -0.05, 0.05
    box[side == 2, 2], box[side == 3, 2] = -0.025, 0.025
    ang = rng.uniform(0, 2 * np.pi, 3000)
    rad = rng.uniform(0.07, 0.10, 3000)
    skirt = np.column_stack((rad * np.cos(ang), rng.uniform(0.002, 0.01, 3000), rad * np.sin(ang)))
    V = np.vstack((box, skirt))
    cut = _skirt_mask(V)
    assert cut[len(box):].mean() > 0.95 and cut[:len(box)].mean() < 0.001


def test_cerrar_sesion_terminada_conserva_las_tomas(textured):
    """«Escanear otro objeto» cierra la sesión: si ya creó un objeto, sus tomas se quedan."""
    s, _ = textured
    s.discard()
    assert os.path.isdir(s.dir) and any(d.startswith("vista_") for d in os.listdir(s.dir))
