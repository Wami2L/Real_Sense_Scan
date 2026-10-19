"""Motor de alineación (live3d/alignment.py) contra la verdad de referencia sintética.

La escena sintética dual coloca Cam 2 desplazada SyntheticDualSource.CAM2_OFFSET en x, así
que T(2->1) real = traslación (+offset, 0, 0). Las vistas se construyen igual que llegan por
el stream: profundidad y color con intrínsecas distintas y extrínseca depth->color.
"""

import cv2
import numpy as np
import pytest

from live3d.alignment import D455, AlignOptions, RGBDView, align_pair, pose_delta, public_result
from live3d.capture import SyntheticDualSource as S
from live3d.registration import euler_translation_to_matrix

GT = np.eye(4)
GT[0, 3] = S.CAM2_OFFSET


def synthetic_view(cam_offset_x, t=1.0, timestamp=None, label="cam"):
    di = S._intr(424, 240, S.DEPTH_HFOV)
    ci = S._intr(424, 240, S.COLOR_HFOV)
    z, _ = S._trace(di, (cam_offset_x, 0, 0), t, False)
    _, col = S._trace(ci, (cam_offset_x - S.BASELINE[0], 0, 0), t, True)
    meta = {"depth": dict(di, scale=0.001), "color": ci,
            "depth_to_color": {"rotation": [1, 0, 0, 0, 1, 0, 0, 0, 1], "translation": list(S.BASELINE)}}
    return RGBDView.from_meta(cv2.cvtColor(col, cv2.COLOR_RGB2BGR), (z * 1000).astype(np.uint16), meta,
                              timestamp=t if timestamp is None else timestamp, label=label)


@pytest.fixture(scope="module")
def pair():
    return synthetic_view(0.0, label="Cam 1"), synthetic_view(S.CAM2_OFFSET, label="Cam 2")


def test_modelo_de_ruido_d455_crece_con_z_al_cuadrado():
    s1, s2 = float(D455.sigma_z(1.0)), float(D455.sigma_z(2.0))
    assert 0.001 < s1 < 0.006           # pocos mm a 1 m
    assert s2 == pytest.approx(4 * s1)  # cuadrático


def test_recupera_la_pose_sin_semilla(pair):
    res = align_pair(*pair)
    assert res["success"] and res["quality"] == "buena", public_result(res)
    d = pose_delta(GT, res["matrix_4x4"])
    assert d["translation_cm"] < 1.0 and d["rotation_deg"] < 0.5
    assert res["metrics"]["inliers"] >= 20   # la fotogrametría aporta en esta escena


def test_recupera_la_pose_con_semilla_erronea(pair):
    wrong = euler_translation_to_matrix(55, 10, -20, 25, 8, -6)
    res = align_pair(*pair, prior_T=wrong)
    assert res["success"], public_result(res)
    assert pose_delta(GT, res["matrix_4x4"])["translation_cm"] < 1.0


def test_es_determinista(pair):
    a, b = align_pair(*pair), align_pair(*pair)
    np.testing.assert_allclose(a["matrix_4x4"], b["matrix_4x4"], atol=1e-9)


def test_sin_profundidad_falla_sin_pose(pair):
    v1, v2 = pair
    vacia = RGBDView(v2.color, np.zeros_like(v2.depth), v2.depth_intr, v2.depth_scale,
                     v2.color_intr, v2.depth_to_color, v2.timestamp, "Cam 2")
    res = align_pair(v1, vacia, prior_T=GT)
    assert not res["success"] and "matrix_4x4" not in res
    assert "Cam 2" in res["message"]


def test_cuadros_desincronizados_se_rechazan():
    v1 = synthetic_view(0.0, timestamp=10.0)
    v2 = synthetic_view(S.CAM2_OFFSET, timestamp=10.6)
    res = align_pair(v1, v2)
    assert not res["success"] and "desincronizados" in res["message"]


def test_escena_sin_relacion_no_produce_falso_positivo(pair):
    v1, _ = pair
    rng = np.random.default_rng(1)
    ruido = RGBDView(rng.integers(0, 255, v1.color.shape, dtype=np.uint8),
                     rng.integers(600, 2900, v1.depth.shape).astype(np.uint16),
                     v1.depth_intr, 0.001, v1.color_intr, v1.depth_to_color, v1.timestamp, "Cam 2")
    res = align_pair(v1, ruido)
    assert not res["success"] and "matrix_4x4" not in res


def test_salto_grande_respecto_a_calibracion_sin_evidencia_pide_confirmacion(pair):
    """Sin fotogrametría (imágenes planas), alejarse mucho de la calibración vigente no se aplica."""
    v1, v2 = pair
    gris1 = RGBDView(np.full_like(v1.color, 128), v1.depth, v1.depth_intr, 0.001, v1.color_intr,
                     v1.depth_to_color, v1.timestamp, "Cam 1")
    gris2 = RGBDView(np.full_like(v2.color, 128), v2.depth, v2.depth_intr, 0.001, v2.color_intr,
                     v2.depth_to_color, v2.timestamp, "Cam 2")
    prior = euler_translation_to_matrix(25 + 60, 0, 0, 30, 0, 0)  # 60 cm y 30° lejos de la real
    res = align_pair(gris1, gris2, prior_T=prior)
    if res["success"]:  # solo aceptable si se queda cerca de la semilla (no debería)
        assert pose_delta(prior, res["matrix_4x4"])["translation_cm"] <= 5.0
    else:
        assert "matrix_4x4" not in res


def _gris(v, label):
    return RGBDView(np.full_like(v.color, 128), v.depth, v.depth_intr, 0.001, v.color_intr,
                    v.depth_to_color, v.timestamp, label)


def test_consistencia_no_valida_poses_fuera_del_rango_del_sensor(pair):
    """Un punto predicho a cientos de metros no puede contar como consistente (sigma acotada a z_max)."""
    from live3d.alignment import _cloud, evaluate, plausible_pose
    v1, v2 = pair
    lejos = np.eye(4)
    lejos[:3, 3] = (-24.0, -314.0, 1.3)
    assert not plausible_pose(lejos, D455)
    c1, c2 = (_cloud(v, D455, 0.01) for v in pair)
    nubes = [(np.asarray(c.points), np.asarray(c.normals)) for c in (c1, c2)]
    zd = [v.depth_m() for v in pair]
    ev = evaluate(lejos, *nubes, v1, v2, *zd, D455, AlignOptions())
    assert ev["overlap_points"] < 50


def test_camaras_movidas_sin_fotogrametria_se_encuentran_y_piden_confirmacion(pair):
    """Montaje cambiado de lado (calibración guardada en espejo) y sin texturas: la búsqueda de
    montaje encuentra la pose, pero solo se aplica si el usuario confirma que movió las cámaras."""
    v1, v2 = _gris(pair[0], "Cam 1"), _gris(pair[1], "Cam 2")
    espejo = euler_translation_to_matrix(-60, 5, 25, 60, 5, -10)
    res = align_pair(v1, v2, prior_T=espejo)
    assert not res["success"] and "matrix_4x4" not in res
    assert res["needs_confirmation"] and res["quality_if_confirmed"] in ("buena", "aceptable")
    assert pose_delta(GT, res["candidate_matrix_4x4"])["translation_cm"] < 2.0
    confirmado = align_pair(v1, v2, prior_T=None)  # el usuario confirma: la calibración vieja deja de contar
    assert confirmado["success"], public_result(confirmado)
    assert pose_delta(GT, confirmado["matrix_4x4"])["translation_cm"] < 2.0


def test_semilla_extra_de_los_sliders(pair):
    v1, v2 = _gris(pair[0], "Cam 1"), _gris(pair[1], "Cam 2")
    cerca = euler_translation_to_matrix(S.CAM2_OFFSET * 100 + 4, 2, -3, 3, 0, 0)
    res = align_pair(v1, v2, extra_seeds=[("pose de los sliders", cerca)], opt=AlignOptions(mount_search=False))
    assert res["success"], public_result(res)
    assert pose_delta(GT, res["matrix_4x4"])["translation_cm"] < 1.0


def test_poca_zona_comun_no_da_falso_positivo_en_la_busqueda_a_ciegas(pair):
    """Con muy poca superficie común (cámara tapada de cerca), probar muchas poses no debe
    acabar aceptando una que encaje por azar (pasó en hardware: Ty -3 m y roll -171°)."""
    v1, v2 = _gris(pair[0], "Cam 1"), _gris(pair[1], "Cam 2")
    recorte = np.zeros_like(v2.depth)
    h, w = recorte.shape
    recorte[h // 2 - 20:h // 2 + 20, w // 2 - 30:w // 2 + 30] = v2.depth[h // 2 - 20:h // 2 + 20, w // 2 - 30:w // 2 + 30]
    tapada = RGBDView(v2.color, recorte, v2.depth_intr, 0.001, v2.color_intr, v2.depth_to_color, v2.timestamp, "Cam 2")
    res = align_pair(v1, tapada, opt=AlignOptions(min_valid_depth_frac=0.0))
    assert not res["success"] and "matrix_4x4" not in res
    assert not res.get("needs_confirmation")
