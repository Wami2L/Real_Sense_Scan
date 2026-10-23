"""Herramientas offline de Gaussian Splatting (plan 15): PLY por bloques, recorte de SH, limpieza, partes de
< 2 GB y poda por importancia con oclusión. Escena sintética: una "habitación" con suelo y una pared que tapa
un objeto escondido."""

import json
import os

import numpy as np
import pytest

from live3d.splats import SplatPly, analisis, estacion, limpieza, write_ply


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
    imp = estacion.importance(core["pos"], core["alpha"], core["sigma"], core["finite"], pts, res=(512, 256))
    wall, hidden = tags == 1, tags == 2
    assert (imp[wall] > 0).mean() > 0.9                  # la pared se ve
    assert (imp[hidden] > 0).mean() < 0.05               # lo de detrás de la pared no
    keep = estacion.top_k(imp, 5000)
    assert keep.sum() == 5000 and not (keep & hidden).any()
