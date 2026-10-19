"""Escáner de objetos (live3d/objscan) contra la escena sintética con verdad de referencia."""

import os

import numpy as np
import pytest

from live3d.objscan.base import LAYOUTS, BaseLayout, estimate_pose
from live3d.objscan.library import Library
from live3d.objscan.rsm import read_rsm, write_rsm
from live3d.objscan.segment import WorkVolume, analyze, segment_with_base
from live3d.objscan.session import ObjectScanSession
from live3d.objscan.sintetico import ObjectScene, object_dimensions, orbit_pose

import cv2


def _err(A, B):
    d = np.linalg.inv(A) @ B
    return (np.linalg.norm(d[:3, 3]) * 1000,
            np.degrees(np.arccos(np.clip((np.trace(d[:3, :3]) - 1) / 2, -1, 1))))


@pytest.fixture(scope="module")
def scene():
    return ObjectScene()


@pytest.mark.parametrize("az,tt", [(0, 0), (60, 0), (20, 140)])
def test_pose_por_base_de_marcadores(scene, az, tt):
    T = orbit_pose(az)
    s = scene.render(T, tt)
    bp = estimate_pose(cv2.cvtColor(s["color"], cv2.COLOR_RGB2BGR), s["intrinsics"], LAYOUTS["A3"],
                       zmap=s["depth"] * 0.001)
    assert bp.ok and len(bp.markers) >= 4
    mm, deg = _err(bp.T_cam_to_base, scene.gt_T_cam_to_base(T, tt))
    assert mm < 8 and deg < 0.6
    assert bp.scale_ratio == pytest.approx(1.0, abs=0.01)


def test_detecta_hoja_impresa_a_otra_escala():
    nominal = LAYOUTS["A3"]
    k = 0.9
    small = BaseLayout(width_mm=nominal.width_mm * k, height_mm=nominal.height_mm * k,
                       marker_mm=nominal.marker_mm * k, margin_mm=nominal.margin_mm * k)
    sc = ObjectScene(layout=small)
    s = sc.render(orbit_pose(15), 0)
    bp = estimate_pose(cv2.cvtColor(s["color"], cv2.COLOR_RGB2BGR), s["intrinsics"], nominal, zmap=s["depth"] * 0.001)
    assert bp.scale_ratio == pytest.approx(k, abs=0.02)


def test_segmentacion_aisla_el_objeto(scene):
    T = orbit_pose(35)
    s = scene.render(T, 20)
    info = analyze(s, LAYOUTS["A3"], WorkVolume())
    assert info["ok"] and not info["warnings"]
    seg = segment_with_base(s, info["T_cam_to_base"], WorkVolume())
    lo, hi = object_dimensions()
    gt = scene.gt_T_cam_to_base(T, 20)
    Pb = seg["points_cam"] @ gt[:3, :3].T + gt[:3, 3]
    inside = np.all((Pb > lo - 0.012) & (Pb < hi + 0.012), axis=1)
    assert inside.mean() > 0.99          # nada de la hoja, la mesa, la pared ni el "estorbo"


def _scan(scene, tmp_path, use_base, steps):
    lib = Library(str(tmp_path / "lib"))
    s = ObjectScanSession("Figura", use_base=use_base, base_dir=str(tmp_path), library=lib, log=lambda *a: None)
    for k in range(steps):
        s.add_snapshot(scene.render(orbit_pose(10, elevation_deg=30 if k % 2 == 0 else 45), k * 360 / steps))
    s.finish(background=False)
    assert s.error is None, s.error
    return s, lib


def test_escaneo_con_base_reconstruye_el_objeto_a_escala(scene, tmp_path):
    s, lib = _scan(scene, tmp_path, True, 10)
    man = s.result
    lo, hi = object_dimensions()
    assert np.allclose(man["tamano_m"], hi - lo, atol=0.005)
    assert man["poses"]["base"] == 10
    for f in ("objeto.ply", "modelo.rsm", "miniatura.jpg", "objeto_puntos.ply"):
        assert os.path.isfile(lib.path(man["id"], f))
    r = read_rsm(lib.path(man["id"], "modelo.rsm"))
    assert r["bounds"][0][1] == pytest.approx(0.0, abs=1e-4)      # apoyado en y = 0 (pivote en la base)


def _surface_error_mm(lib, man):
    """Distancia media (mm) de la malla obtenida a la real, tras alinearlas (giro + ICP)."""
    import open3d as o3d
    from live3d.objscan.sintetico import make_object
    gt = make_object().sample_points_uniformly(30000)
    got = o3d.io.read_triangle_mesh(lib.path(man["id"], "objeto.ply")).sample_points_uniformly(30000)
    got.translate(-np.asarray(man["pivote_desplazado_m"]))      # deshacer el pivote
    reg = o3d.pipelines.registration
    best = None
    for ang in range(0, 360, 15):
        a = np.radians(ang)
        T0 = np.eye(4)
        T0[[0, 0, 2, 2], [0, 2, 0, 2]] = np.cos(a), np.sin(a), -np.sin(a), np.cos(a)
        T0[:3, 3] = gt.get_center() - T0[:3, :3] @ got.get_center()
        r = reg.registration_icp(got, gt, 0.02, T0, reg.TransformationEstimationPointToPoint())
        if best is None or r.inlier_rmse * (2 - r.fitness) < best.inlier_rmse * (2 - best.fitness):
            best = r
    got.transform(best.transformation)
    return float(np.mean(np.asarray(got.compute_point_cloud_distance(gt)))) * 1000


def test_superficie_con_base_es_fiel(scene, tmp_path):
    s, lib = _scan(scene, tmp_path, True, 10)
    assert _surface_error_mm(lib, s.result) < 2.0


def test_escaneo_sin_base_tambien_funciona(scene, tmp_path):
    s, lib = _scan(scene, tmp_path, False, 14)
    assert s.result["tamano_m"][1] == pytest.approx(0.18, abs=0.01)   # la altura no depende del marco
    assert _surface_error_mm(lib, s.result) < 3.0


def test_rsm_ida_y_vuelta_en_convencion_unity(tmp_path):
    V = np.array([[0, 0, 0], [1, 0, 0], [0, 1, 0.5]], np.float64)
    T = np.array([[0, 1, 2]])
    C = np.array([[1, 0, 0], [0, 1, 0], [0, 0, 1]], np.float64)
    p = tmp_path / "x.rsm"
    write_rsm(str(p), V, T, normals=np.tile([0, 0, 1.0], (3, 1)), colors=C)
    r = read_rsm(str(p))
    assert r["vertices"][2, 2] == pytest.approx(-0.5)             # z -> -z (Unity es zurdo)
    assert r["indices"].tolist() == [[2, 1, 0]]                    # orden invertido
    assert r["normals"][0, 2] == pytest.approx(-1)
    assert r["colors"][1].tolist() == [0, 255, 0, 255]


def test_biblioteca_publicar_y_regenerar_rsm(scene, tmp_path):
    import open3d as o3d
    import time
    lib = Library(str(tmp_path / "lib"))
    mesh = o3d.geometry.TriangleMesh.create_box(0.1, 0.2, 0.1)
    mesh.compute_vertex_normals()
    man = lib.create("Caja de prueba", mesh)
    assert lib.list(published_only=True) == []
    man2 = lib.publish(man["id"])
    assert man2["publicado"] and man2["version"] == 2
    assert [m["id"] for m in lib.list(published_only=True)] == [man["id"]]
    # Blender guarda una versión limpia más nueva -> el RSM se regenera desde ella
    time.sleep(0.05)
    clean = o3d.geometry.TriangleMesh.create_sphere(0.05)
    o3d.io.write_triangle_mesh(lib.path(man["id"], "objeto_limpio.ply"), clean)
    os.utime(lib.path(man["id"], "objeto_limpio.ply"), None)
    lib.ensure_rsm(man["id"])
    assert lib.get(man["id"])["fuente"] == "objeto_limpio.ply"
    assert read_rsm(lib.path(man["id"], "modelo.rsm"))["vertices"].shape[0] == len(clean.vertices)
    with pytest.raises(ValueError):
        lib.path("../fuera")


def test_dual_con_calibracion_desajustada_se_corrige_sin_fantasmas(tmp_path):
    """La Cam 2 no ve la base y su calibración está mal (4 cm, 5°): el escaneo estima UNA
    corrección común con todas las tomas y la superficie sale igual de fiel."""
    from live3d.registration import euler_translation_to_matrix
    sc1, sc2 = ObjectScene(), ObjectScene()
    sc2.sheet[:] = 1.0
    lib = Library(str(tmp_path / "lib"))
    s = ObjectScanSession("dual", base_dir=str(tmp_path), library=lib, log=lambda *a: None)
    P = euler_translation_to_matrix(4, -2, 2, 5, -2.5, 1.7)
    T1, T2 = orbit_pose(10, elevation_deg=28), orbit_pose(75, elevation_deg=40, distance=0.65)
    for k in range(8):
        s.add_snapshot({"cam1": sc1.render(T1, k * 45), "cam2": sc2.render(T2, k * 45),
                        "extrinsics": np.linalg.inv(T1) @ T2 @ P})
    s.finish(background=False)
    info = s.result["correccion_calibracion_dual"]
    assert info["aplicada"] and info["muestras"] >= 4
    assert 30 < info["mm"] < 70 and 3 < info["deg"] < 9          # mide el desajuste inyectado
    assert _surface_error_mm(lib, s.result) < 2.0
