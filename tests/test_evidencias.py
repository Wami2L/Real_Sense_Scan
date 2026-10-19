"""Evidencia inequívoca para la alineación: marcadores ArUco, puntos fijados a mano, mapa de error
y combinación de varias tomas (live3d/alignment.py)."""

import cv2
import numpy as np
import pytest

from live3d.alignment import (D455, AlignOptions, RGBDView, align_pair, alignment_error_map, combine_poses,
                              detect_markers, pose_delta, preflight, public_result, transform)
from live3d.registration import euler_translation_to_matrix

from test_alineacion import GT, pair, synthetic_view  # noqa: F401  (fixture reutilizada)

INTR = {"width": 848, "height": 480, "fx": 430.0, "fy": 430.0, "ppx": 424.0, "ppy": 240.0}
PLANE_Z = 1.6
# Cam 2 a 60 cm a la izquierda y girada 40° hacia el centro: convergente como el montaje real
T_MARK = euler_translation_to_matrix(-60, 3, 20, 40, 2, -3)
MARKERS = {0: (-0.22, -0.05), 1: (0.20, 0.08)}   # centro (x, y) en el plano, marco de Cam 1
MARKER_M, PAPER_M = 0.15, 0.21


def _marker_textures():
    d = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
    return {i: cv2.aruco.generateImageMarker(d, i, 240) for i in MARKERS}


def _render(T_cam_to_1, tex):
    """Plano z = PLANE_Z (marco de Cam 1) con textura suave y dos marcadores impresos en papel."""
    vv, uu = np.mgrid[0:INTR["height"], 0:INTR["width"]]
    d = np.stack([(uu - INTR["ppx"]) / INTR["fx"], (vv - INTR["ppy"]) / INTR["fy"], np.ones(uu.shape)], -1)
    R, t = T_cam_to_1[:3, :3], T_cam_to_1[:3, 3]
    d1 = d @ R.T
    s = (PLANE_Z - t[2]) / d1[..., 2]
    X = t + d1 * s[..., None]
    rng = np.random.default_rng(3)
    noise = cv2.resize(rng.random((24, 24)).astype(np.float32), (512, 512), interpolation=cv2.INTER_CUBIC)
    gi = ((X[..., 0] + 2) * 120).astype(int) % 512
    gj = ((X[..., 1] + 2) * 120).astype(int) % 512
    gray = 90 + 70 * noise[gj, gi]
    for mid, (cx, cy) in MARKERS.items():
        paper = (np.abs(X[..., 0] - cx) < PAPER_M / 2) & (np.abs(X[..., 1] - cy) < PAPER_M / 2)
        gray[paper] = 235
        inside = (np.abs(X[..., 0] - cx) < MARKER_M / 2) & (np.abs(X[..., 1] - cy) < MARKER_M / 2)
        mu = ((X[..., 0] - cx) / MARKER_M + 0.5) * (tex[mid].shape[1] - 1)
        mv = ((X[..., 1] - cy) / MARKER_M + 0.5) * (tex[mid].shape[0] - 1)
        mu = np.clip(mu, 0, tex[mid].shape[1] - 1).astype(int)
        mv = np.clip(mv, 0, tex[mid].shape[0] - 1).astype(int)
        gray[inside] = np.where(tex[mid][mv[inside], mu[inside]] > 127, 235, 20)
    color = cv2.cvtColor(gray.clip(0, 255).astype(np.uint8), cv2.COLOR_GRAY2BGR)
    depth = np.where((s > 0), s * 1000.0, 0).astype(np.uint16)  # z en el marco de la cámara = s
    return color, depth


def marker_views():
    """Par sintético con dos marcadores ArUco en un plano; Cam 2 en T_MARK (verdad de referencia)."""
    tex = _marker_textures()
    views = []
    for i, T in enumerate((np.eye(4), T_MARK)):
        color, depth = _render(T, tex)
        views.append(RGBDView(color, depth, dict(INTR), 0.001, None, None, 10.0, f"Cam {i + 1}"))
    return views


@pytest.fixture(scope="module")
def marker_pair():
    return marker_views()


def test_marcadores_se_detectan_en_ambas_camaras(marker_pair):
    for v in marker_pair:
        assert set(detect_markers(v.color)) == set(MARKERS)


def test_marcadores_dan_la_pose_en_un_plano_sin_textura_util(marker_pair):
    """Un plano es ambiguo para la geometría (el ICP desliza); los marcadores fijan la pose."""
    res = align_pair(*marker_pair)
    assert res["success"], public_result(res)
    d = pose_delta(T_MARK, res["matrix_4x4"])
    assert d["translation_cm"] < 1.5 and d["rotation_deg"] < 1.0, d
    assert res["metrics"]["evidence"] == "marcadores"
    assert res["metrics"]["markers"]["used"] == sorted(MARKERS)
    corr = [c for c in res["correspondences"] if c["kind"] == "marcador"]
    assert len(corr) == 8 and all(c["ok"] for c in corr)


def test_marcadores_permiten_un_montaje_nuevo_sin_confirmacion(marker_pair):
    vieja = euler_translation_to_matrix(55, 0, 30, -60, 0, 0)  # calibración guardada del lado contrario
    res = align_pair(*marker_pair, prior_T=vieja)
    assert res["success"] and not res["needs_confirmation"], public_result(res)
    assert pose_delta(T_MARK, res["matrix_4x4"])["translation_cm"] < 1.5


def test_previsualizacion_avisa_de_marcadores_comunes(marker_pair):
    res = preflight(*marker_pair, None)
    assert res["markers"]["common"] == sorted(MARKERS)
    assert res["expected_quality"] == "aceptable"
    assert any("marcador" in i["text"].lower() for i in res["issues"])


def _manual_pairs(v1, v2, T, n=6):
    """Píxeles de color de los mismos puntos físicos en ambas cámaras (proyección con la pose real)."""
    d = v1.depth_m()
    di = v1.depth_intr
    pts = []
    for (u, v) in [(60, 60), (360, 70), (200, 150), (90, 200), (330, 210), (250, 40), (150, 110), (300, 140)]:
        z = d[v, u]
        pts.append(((u - di["ppx"]) / di["fx"] * z, (v - di["ppy"]) / di["fy"] * z, z))
    pts = np.asarray(pts)

    def to_px(view, P_depth):
        Pc = transform(view.E, P_depth)
        ci = view.cintr
        return np.column_stack((Pc[:, 0] / Pc[:, 2] * ci["fx"] + ci["ppx"], Pc[:, 1] / Pc[:, 2] * ci["fy"] + ci["ppy"]))

    px1 = to_px(v1, pts)
    px2 = to_px(v2, transform(np.linalg.inv(T), pts))
    ok = (px2[:, 0] > 5) & (px2[:, 0] < v2.cintr["width"] - 5) & (px2[:, 1] > 5) & (px2[:, 1] < v2.cintr["height"] - 5)
    return [(tuple(a), tuple(b)) for a, b in zip(px1[ok], px2[ok])][:n]


def _gris(v, label):
    return RGBDView(np.full_like(v.color, 128), v.depth, v.depth_intr, 0.001, v.color_intr,
                    v.depth_to_color, v.timestamp, label)


def test_puntos_a_mano_dan_la_pose_sin_textura_ni_semilla(pair):  # noqa: F811
    v1, v2 = _gris(pair[0], "Cam 1"), _gris(pair[1], "Cam 2")
    pares = _manual_pairs(*pair, GT)
    assert len(pares) >= 5
    vieja = euler_translation_to_matrix(-60, 5, 25, 60, 5, -10)
    res = align_pair(v1, v2, prior_T=vieja, manual_pairs=pares, opt=AlignOptions(mount_search=False))
    assert res["success"] and not res["needs_confirmation"], public_result(res)
    assert pose_delta(GT, res["matrix_4x4"])["translation_cm"] < 1.0
    assert res["metrics"]["evidence"] == "manual"
    man = [c for c in res["correspondences"] if c["kind"] == "manual"]
    assert len(man) == len(pares)
    # alguna pareja del generador cae en un punto que la otra cámara no ve: se descarta y se avisa
    assert sum(c["ok"] for c in man) >= len(pares) - 1
    assert all(c["residual_mm"] < 15 for c in man if c["ok"])


def test_puntos_a_mano_incoherentes_avisan(pair):  # noqa: F811
    v1, v2 = _gris(pair[0], "Cam 1"), _gris(pair[1], "Cam 2")
    pares = _manual_pairs(*pair, GT, n=3)
    pares[0] = (pares[0][0], pares[2][1])  # la pareja 1 apunta a otro punto en la Cam 2
    res = align_pair(v1, v2, manual_pairs=pares, opt=AlignOptions(mount_search=False))
    assert any("no son coherentes" in w for w in res["warnings"]), res["warnings"]
    assert res["metrics"].get("evidence") != "manual"


def test_mapa_de_error_distingue_pose_buena_y_mala(pair):  # noqa: F811
    img, bien = alignment_error_map(*pair, GT)
    mal_T = GT.copy()
    mal_T[0, 3] += 0.05
    _, mal = alignment_error_map(*pair, mal_T)
    assert img.shape[0] > pair[0].color.shape[0]  # imagen + leyenda
    assert bien["median_mm"] < 8 and bien["pct_lt_10"] > 80
    assert mal["median_mm"] > 2 * bien["median_mm"]


def test_combinar_tomas_descarta_la_atipica():
    rng = np.random.default_rng(0)
    entries = []
    for k in range(5):
        dT = euler_translation_to_matrix(*(rng.normal(0, 0.4, 3)), *(rng.normal(0, 0.4, 3)))
        entries.append({"T": GT @ dT, "weight": 1000 + k})
    entries.append({"T": GT @ euler_translation_to_matrix(12, 0, 0, 8, 0, 0), "weight": 900})  # toma rara
    Tm, used, devs = combine_poses(entries)
    assert len(used) == 5
    d = pose_delta(GT, Tm)
    assert d["translation_cm"] < 0.5 and d["rotation_deg"] < 0.5
    assert max(x["rotation_deg"] for x in devs) < 2.0


def test_sigma_d455_sigue_definida():
    assert float(D455.sigma_z(1.0)) > 0
