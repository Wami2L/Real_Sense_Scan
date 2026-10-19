"""Rectificación de la lente (live3d/objscan/optica.py)."""

import cv2
import numpy as np
import pytest

from live3d.objscan.base import LAYOUTS, estimate_pose
from live3d.objscan.optica import _brown, rectify_snapshot, source_map
from live3d.objscan.sintetico import ObjectScene, orbit_pose

# Coeficientes del orden de los de una lente RGB gran angular
COEFFS = [-0.055, 0.065, 0.0004, -0.0006, -0.02]


def _distort(snap, model):
    """Simula la imagen que entregaría una lente con distorsión (inversa de la rectificación)."""
    intr = dict(snap["intrinsics"], model=model, coeffs=COEFFS)
    w, h = intr["width"], intr["height"]
    uu, vv = np.meshgrid(np.arange(w, dtype=np.float64), np.arange(h, dtype=np.float64))
    x, y = (uu - intr["ppx"]) / intr["fx"], (vv - intr["ppy"]) / intr["fy"]
    k = np.array(COEFFS)
    if "inverse" in model:
        xi, yi = _brown(x, y, k)                    # ideal = f(distorsionado)
    else:
        pts = np.column_stack((uu.ravel(), vv.ravel())).reshape(-1, 1, 2).astype(np.float64)
        K = np.array([[intr["fx"], 0, intr["ppx"]], [0, intr["fy"], intr["ppy"]], [0, 0, 1]])
        und = cv2.undistortPoints(pts, K, k[[0, 1, 2, 3, 4]]).reshape(h, w, 2)
        xi, yi = und[..., 0], und[..., 1]
    mx = (xi * intr["fx"] + intr["ppx"]).astype(np.float32)
    my = (yi * intr["fy"] + intr["ppy"]).astype(np.float32)
    out = dict(snap, intrinsics=intr)
    out["color"] = cv2.remap(snap["color"], mx, my, cv2.INTER_LINEAR)
    out["depth"] = cv2.remap(snap["depth"], mx, my, cv2.INTER_NEAREST)
    return out


@pytest.mark.parametrize("model", ["distortion.brown_conrady", "distortion.inverse_brown_conrady"])
def test_rectificar_devuelve_la_imagen_ideal_y_la_pose(model):
    sc = ObjectScene()
    T = orbit_pose(30)
    ideal = sc.render(T, 0, noise=False)
    dist = _distort(ideal, model)
    rect = rectify_snapshot(dist)
    assert rect["intrinsics"]["coeffs"] == [0.0] * 5
    # la imagen rectificada coincide con la ideal (salvo bordes e interpolación)
    m = (rect["depth"] > 0) & (ideal["depth"] > 0)
    assert np.median(np.abs(rect["depth"][m].astype(int) - ideal["depth"][m].astype(int))) <= 1
    gt = sc.gt_T_cam_to_base(T, 0)

    def err(s):
        bp = estimate_pose(cv2.cvtColor(s["color"], cv2.COLOR_RGB2BGR), s["intrinsics"], LAYOUTS["A3"])
        return np.linalg.norm(bp.T_cam_to_base[:3, 3] - gt[:3, 3]) * 1000 if bp.ok else 1e9

    assert err(rect) < 8.0                         # con la lente corregida, como sin distorsión
    assert err(rect) < err(dist)                   # y mejor que ignorándola


def test_sin_coeficientes_no_toca_nada():
    snap = {"color": np.zeros((4, 4, 3), np.uint8), "depth": np.zeros((4, 4), np.uint16),
            "intrinsics": {"width": 4, "height": 4, "fx": 2, "fy": 2, "ppx": 1.5, "ppy": 1.5}}
    assert rectify_snapshot(snap) is snap


def test_mapa_en_cache():
    intr = {"width": 64, "height": 48, "fx": 40, "fy": 40, "ppx": 31.5, "ppy": 23.5,
            "model": "distortion.brown_conrady", "coeffs": COEFFS}
    assert source_map(intr) is source_map(dict(intr))
