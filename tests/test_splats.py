"""Herramientas offline de Gaussian Splatting (plan 15): PLY por bloques, recorte de SH, limpieza, partes de
< 2 GB y poda por importancia con oclusión. Escena sintética: una "habitación" con suelo y una pared que tapa
un objeto escondido."""

import json
import os

import numpy as np
import pytest

from live3d.splats import SplatPly, analisis, estacion, limpieza, lod, write_columns, write_ply


def _write_scene(path, pos, alpha, sigma, sh_degree=3, seed=0):
    """PLY 3DGS (formato INRIA) con f_rest_* aleatorios marcados por canal y coeficiente."""
    rng = np.random.default_rng(seed)
    n = len(pos)
    per = {0: 0, 1: 3, 2: 8, 3: 15}[sh_degree]
    props = ["x", "y", "z", "f_dc_0", "f_dc_1", "f_dc_2"] + [f"f_rest_{i}" for i in range(3 * per)] + \
            ["opacity", "scale_0", "scale_1", "scale_2", "rot_0", "rot_1", "rot_2", "rot_3"]
    data = np.zeros(n, np.dtype([(p, "<f4") for p in props]))
    data["x"], data["y"], data["z"] = pos.T
    data["f_dc_0"], data["f_dc_1"], data["f_dc_2"] = rng.normal(0, 1, (3, n))
    for c in range(3):
        for k in range(per):
            data[f"f_rest_{c * per + k}"] = 100 * c + k     # valor = canal·100 + coeficiente
    data["opacity"] = np.log(alpha / (1 - alpha))
    for i in range(3):
        data[f"scale_{i}"] = np.log(sigma[:, i])
    data["rot_0"] = 1
    header = ["ply", "format binary_little_endian 1.0", f"element vertex {n}"] + \
             [f"property float {p}" for p in props] + ["end_header"]
    with open(path, "wb") as f:
        f.write(("\n".join(header) + "\n").encode())
        f.write(data.tobytes())
    return path


def _room(seed=0):
    """Suelo en y = 2 (RDF: arriba = -y), una pared en z = 5 y un objeto escondido detrás en z = 8."""
    rng = np.random.default_rng(seed)
    floor = np.c_[rng.uniform(-5, 5, 20000), np.full(20000, 2.0), rng.uniform(-5, 10, 20000)]
    wall = np.c_[rng.uniform(-5, 5, 20000), rng.uniform(-3, 2, 20000), np.full(20000, 5.0)]
    hidden = np.c_[rng.uniform(-0.5, 0.5, 2000), rng.uniform(0, 1, 2000), np.full(2000, 8.0)]
    floaters = np.c_[rng.uniform(-60, 60, 50), rng.uniform(-60, 60, 50), rng.uniform(-60, 60, 50)]
    pos = np.vstack([floor, wall, hidden, floaters]).astype(np.float32)
    n = len(pos)
    alpha = np.full(n, 0.9, np.float32)
    alpha[:500] = 0.005                                  # algunos casi transparentes
    sigma = np.full((n, 3), 0.03, np.float32)
    sigma[:, 2] = 0.005
    sigma[600:603] = 5.0                                 # tres gigantes
    tags = np.r_[np.zeros(20000), np.ones(20000), np.full(2000, 2), np.full(50, 3)]
    return pos, alpha, sigma, tags


@pytest.fixture(scope="module")
def scene(tmp_path_factory):
    d = tmp_path_factory.mktemp("splats")
    pos, alpha, sigma, tags = _room()
    path = _write_scene(str(d / "scene.ply"), pos, alpha, sigma)
    return d, path, tags


def test_lectura_y_derivados(scene):
    _, path, _ = scene
    ply = SplatPly.open(path)
    assert ply.count == 42050 and ply.sh_degree == 3 and ply.bytes_per_splat == (14 + 45) * 4
    core = analisis.load_core(ply)
    assert core["finite"].all()
    assert np.allclose(core["alpha"][:500], 0.005, atol=1e-4)
    assert np.allclose(core["sigma"][0], [0.03, 0.03, 0.005], rtol=1e-4)   # ordenadas de mayor a menor


def test_recorte_sh_renumera(scene):
    d, path, _ = scene
    ply = SplatPly.open(path)
    out = str(d / "sh1.ply")
    n = write_ply(out, ply, sh_degree=1)
    sh1 = SplatPly.open(out)
    assert n == ply.count and sh1.sh_degree == 1 and sh1.sh_rest == 9
    b = next(sh1.chunks())[1][0]
    # Por canal, los 3 primeros coeficientes del original (canal·100 + k), en el orden del formato.
    assert [b[f"f_rest_{i}"] for i in range(9)] == [0, 1, 2, 100, 101, 102, 200, 201, 202]


def test_informe_y_suelo(scene):
    _, path, _ = scene
    ply = SplatPly.open(path)
    rep, _ = analisis.inspect(ply)
    json.dumps(rep)                                       # serializable
    assert rep["suelo"]["eje"] == "-y"
    assert rep["suelo"]["inclinacion_grados"] < 1.0
    assert abs(rep["suelo"]["punto"][1] - 2.0) < 0.05


def test_limpieza_y_partes(scene, monkeypatch):
    d, path, tags = scene
    ply = SplatPly.open(path)
    out = str(d / "limpio.ply")
    man = limpieza.clean(ply, out, limpieza.Opciones())
    q = man["quitados"]
    assert q["transparente"] == 500 and q["gigante"] == 3
    assert q["fuera_de_zona"] + q["aislado"] >= 40           # los flotantes lejanos
    assert len(man["partes"]) == 1 and abs(man["traslacion_restada"][1] - 2.0) < 0.05
    clean = SplatPly.open(out)
    y = clean.columns(["y"])[:, 0]
    assert np.median(y[:19000]) == pytest.approx(0.0, abs=0.05)   # el suelo (primeros splats) queda en y = 0
    # Por encima del límite se parte en trozos a lo largo de z con la misma cantidad total.
    monkeypatch.setattr(limpieza, "MAX_BYTES", 4 * 1024 * 1024)
    man2 = limpieza.clean(ply, str(d / "partes.ply"), limpieza.Opciones())
    assert len(man2["partes"]) >= 2
    assert sum(SplatPly.open(p).count for p in man2["partes"]) == man["splats_salida"]
    assert all(os.path.exists(p) for p in man2["partes"])


def test_importancia_respeta_oclusion(scene):
    _, path, tags = scene
    ply = SplatPly.open(path)
    core = analisis.load_core(ply)
    pts = estacion.view_points(np.array([0.0, 2.0, 0.0]), radius=0.5)   # delante de la pared
    imp = estacion.importance(core["pos"], core["alpha"], core["sigma"], core["finite"], pts, res=(512, 256),
                              normal=core["normal"])
    wall, hidden = tags == 1, tags == 2
    assert (imp[wall] > 0).mean() > 0.9                  # la pared se ve
    assert (imp[hidden] > 0).mean() < 0.05               # lo de detrás de la pared no
    keep = estacion.top_k(imp, 5000)
    assert keep.sum() == 5000 and not (keep & hidden).any()


def test_cuaterniones_ida_y_vuelta():
    rng = np.random.default_rng(1)
    q = rng.normal(size=(500, 4))
    q /= np.linalg.norm(q, axis=1, keepdims=True)
    q[q[:, 0] < 0] *= -1
    q2 = lod._mat_to_quat(lod._quat_to_mat(q))
    q2[q2[:, 0] < 0] *= -1
    assert np.allclose(q, q2, atol=1e-9)


def _cols_from_arrays(pos, alpha, sigma, quat, dc):
    cols = {"x": pos[:, 0], "y": pos[:, 1], "z": pos[:, 2],
            "f_dc_0": dc[:, 0], "f_dc_1": dc[:, 1], "f_dc_2": dc[:, 2],
            "opacity": np.log(alpha / (1 - alpha)),
            "scale_0": np.log(sigma[:, 0]), "scale_1": np.log(sigma[:, 1]), "scale_2": np.log(sigma[:, 2]),
            "rot_0": quat[:, 0], "rot_1": quat[:, 1], "rot_2": quat[:, 2], "rot_3": quat[:, 3]}
    return {k: np.asarray(v, np.float32) for k, v in cols.items()}


def test_fusion_de_una_pared_lejana():
    """Pared densa a 20 m: se fusiona en pocas gaussianas planas, opacas, del mismo color y en su sitio."""
    rng = np.random.default_rng(2)
    n = 40000
    pos = np.c_[rng.uniform(-2, 2, n), rng.uniform(-1, 1, n), np.full(n, 20.0)]
    sigma = np.c_[np.full(n, 0.02), np.full(n, 0.02), np.full(n, 0.002)]   # planas en z (local = mundo)
    quat = np.tile([1.0, 0, 0, 0], (n, 1))
    dc = np.tile([0.5, -0.2, 0.1], (n, 1))
    cols = _cols_from_arrays(pos, np.full(n, 0.8), sigma, quat, dc)
    key, size = lod.cell_keys(pos, np.zeros(3), theta=np.radians(1.0))
    out, st = lod.merge(cols, key, size)
    m = len(out["x"])
    assert m < n / 20 and st["intactos"] < 50
    p = np.c_[out["x"], out["y"], out["z"]]
    assert np.allclose(p[:, 2], 20.0, atol=0.02)                            # siguen en el plano de la pared
    assert np.allclose(np.c_[out["f_dc_0"], out["f_dc_1"], out["f_dc_2"]], [0.5, -0.2, 0.1], atol=1e-5)
    a = 1 / (1 + np.exp(-out["opacity"]))
    assert np.median(a) > 0.9                                               # pared densa: opaca
    s = np.sort(np.exp(np.c_[out["scale_0"], out["scale_1"], out["scale_2"]]), axis=1)
    assert np.median(s[:, 0]) < 0.01 < np.median(s[:, 1])                    # planas: espesor << extensión
    # El eje más corto de cada fusionada apunta hacia la cámara (normal de la pared = z).
    R = lod._quat_to_mat(np.c_[out["rot_0"], out["rot_1"], out["rot_2"], out["rot_3"]].astype(np.float64))
    i_min = np.argmin(np.c_[out["scale_0"], out["scale_1"], out["scale_2"]], axis=1)
    normal = R[np.arange(m), :, i_min]
    assert np.median(np.abs(normal[:, 2])) > 0.99


def test_cerca_queda_intacto():
    """Splats sueltos a 1 m: cada uno en su celda, se copian sin tocar."""
    rng = np.random.default_rng(3)
    n = 300
    pos = np.c_[rng.uniform(-3, 3, n), rng.uniform(-3, 3, n), np.full(n, 1.2)]   # muy separados entre sí
    cols = _cols_from_arrays(pos, np.full(n, 0.5), np.full((n, 3), 0.01), np.tile([1.0, 0, 0, 0], (n, 1)),
                             rng.normal(size=(n, 3)))
    key, size = lod.cell_keys(pos, np.zeros(3), theta=np.radians(0.5))
    out, st = lod.merge(cols, key, size)
    assert st["intactos"] == n and len(out["x"]) == n
    assert sorted(out["x"].tolist()) == sorted(cols["x"].tolist())


def test_pared_de_canto_no_se_tapa_a_si_misma():
    """Una pared larga vista muy de canto (como las altas de la nave) con splats grandes y planos y detalle
    pequeño encima: los discos de los grandes no deben ocultar las partes más lejanas de la misma pared."""
    rng = np.random.default_rng(4)
    big = 3000
    small = 20000
    z = np.r_[rng.uniform(1, 40, big), rng.uniform(1, 40, small)]
    y = np.r_[rng.uniform(-6, 0, big), rng.uniform(-6, 0, small)]
    pos = np.c_[np.full(big + small, 3.0), y, z].astype(np.float32)
    sigma = np.r_[np.tile([0.35, 0.35, 0.01], (big, 1)), np.tile([0.03, 0.03, 0.005], (small, 1))].astype(np.float32)
    alpha = np.full(big + small, 0.9, np.float32)
    pts = estacion.view_points(np.array([0.0, 0.0, 0.0]), radius=0.5)
    normal = np.tile(np.float32([1, 0, 0]), (len(pos), 1))            # la pared mira hacia x
    imp = estacion.importance(pos, alpha, sigma, np.ones(len(pos), bool), pts, normal=normal)
    assert (imp[big:] > 0).mean() > 0.97


def test_piramide_rangos_y_niveles(scene):
    """Pirámide LOD por trozos: rangos contiguos sin solapes, niveles cada vez más pequeños, cajas en ejes de
    Unity (y invertida) y nivel 0 = splats originales."""
    from live3d.splats import piramide
    d, path, _ = scene
    ply = SplatPly.open(path)
    out = str(d / "piramide.ply")
    man = piramide.build(ply, out, trozo=4.0, voxels=(0.1, 0.4), log=lambda *a: None)
    pyr = SplatPly.open(out)
    assert pyr.count == man["splats_total"]
    ranges = sorted((s, c) for t in man["trozos"] for s, c in zip(t["inicio"], t["cuenta"]))
    cursor = 0
    for s, c in ranges:                                   # contiguos, sin huecos ni solapes
        assert s == cursor
        cursor += c
    assert cursor == pyr.count
    per_level = man["splats_por_nivel"]
    assert per_level[0] >= ply.count - 100               # solo se descartan trozos casi vacíos
    assert per_level[0] > per_level[1] > per_level[2]
    # Cajas en ejes de Unity: y = -y del archivo, así que el suelo (y = 2 en RDF) queda en y_unity = -2.
    ys = [(t["min"][1], t["max"][1]) for t in man["trozos"]]
    assert min(lo for lo, _ in ys) < -1.9 and all(lo <= hi for lo, hi in ys)
    # Nivel 0 de cada trozo = sus splats originales, sin tocar.
    orig = ply.columns(["x", "y", "z"])
    lvl0 = np.concatenate([pyr.columns(["x", "y", "z"])[t["inicio"][0]:t["inicio"][0] + t["cuenta"][0]]
                           for t in man["trozos"]])
    key = lambda a: np.sort(a.view(np.dtype((np.void, a.dtype.itemsize * 3))).ravel())   # noqa: E731
    assert np.isin(key(np.ascontiguousarray(lvl0)), key(np.ascontiguousarray(orig))).all()


def test_suelo_y_escala_desde_camaras_colmap(tmp_path):
    """Escena de COLMAP sin gravedad ni escala: el suelo (y = 2, arriba = -y) se encuentra aunque el "arriba" medio
    de las cámaras esté inclinado 20° (móvil apuntando hacia abajo), y su altura sobre él da la escala."""
    from live3d.splats import colmap
    rng = np.random.default_rng(5)
    floor = np.c_[rng.uniform(-5, 5, 30000), np.full(30000, 2.0), rng.uniform(-5, 5, 30000)]
    wall = np.c_[rng.uniform(-5, 5, 20000), rng.uniform(-6, 2, 20000), np.full(20000, 5.0)]
    pos = np.vstack([floor, wall]).astype(np.float32)
    alpha = np.full(len(pos), 0.9, np.float32)
    centers = np.c_[np.linspace(-3, 3, 50), np.full(50, 2.0 - 11.5), np.zeros(50)]   # 11,5 unidades sobre el suelo
    tilt = np.radians(20)
    ups = np.tile([0.0, -np.cos(tilt), np.sin(tilt)], (50, 1))
    s = colmap.floor_from_cameras(pos, alpha, np.ones(len(pos), bool), centers, ups)
    assert s["eje"] == "-y" and s["inclinacion_grados"] < 1.0
    assert abs(s["altura_camaras"] - 11.5) < 0.1
    assert 15 < s["angulo_con_camaras"] < 25
    # images.txt mínimo: una cámara en el origen mirando a +z.
    txt = tmp_path / "images.txt"
    txt.write_text("# c\n1 1 0 0 0 0 0 0 1 a.jpg\n\n")
    c, u, f = colmap.read_cameras(str(txt))
    assert np.allclose(c, 0) and np.allclose(u, [[0, -1, 0]]) and np.allclose(f, [[0, 0, 1]])


def test_filtro_de_agujas():
    """Solo se quitan las agujas largas y casi transparentes; las finas opacas (barrotes) se quedan."""
    n = 4
    core = {
        "pos": np.zeros((n, 3), np.float32), "finite": np.ones(n, bool),
        "alpha": np.array([0.1, 0.9, 0.1, 0.1], np.float32),
        "sigma": np.array([[0.2, 0.005, 0.004],      # aguja fantasma -> fuera
                           [0.2, 0.005, 0.004],      # barrote opaco -> se queda
                           [0.01, 0.0005, 0.0004],   # alargada pero diminuta -> se queda
                           [0.2, 0.1, 0.004]], np.float32),   # disco plano, no aguja -> se queda
    }
    rep = {"zona_principal": {"min": [-1, -1, -1], "max": [1, 1, 1], "diagonal": 3.46}}
    extra = {"inside": np.ones(n, bool), "neigh": np.full(n, 100)}
    m = limpieza.masks(core, rep, extra, limpieza.Opciones(agujas_ratio=10), escala=1.0)
    assert m["aguja"].tolist() == [True, False, False, False]
