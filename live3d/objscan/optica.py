"""
Rectificación de la lente: deja cada snapshot como una cámara pinhole ideal (sin distorsión).

La RGB de la D455 es gran angular (~90°) y librealsense publica sus coeficientes de
distorsión. La profundidad alineada al color vive en esa imagen distorsionada. Rectificar
color y profundidad a la vez nada más llegar el snapshot hace coherentes la pose por
marcadores (PnP), la segmentación y el TSDF, que asumen pinhole.

Modelos (librealsense, coeficientes k1, k2, p1, p2, k3):
  brown_conrady / modified_brown_conrady   distorsionado = f(ideal)    (igual que OpenCV)
  inverse_brown_conrady                    ideal = f(distorsionado)
"""

import cv2
import numpy as np

_CACHE = {}


def has_distortion(intr):
    c = intr.get("coeffs")
    return bool(c) and any(abs(float(x)) > 1e-9 for x in c[:5])


def _brown(x, y, k):
    k1, k2, p1, p2, k3 = k
    r2 = x * x + y * y
    f = 1 + k1 * r2 + k2 * r2 * r2 + k3 * r2 * r2 * r2
    return x * f + 2 * p1 * x * y + p2 * (r2 + 2 * x * x), y * f + 2 * p2 * x * y + p1 * (r2 + 2 * y * y)


def source_map(intr):
    """Para cada píxel de la imagen ideal, el píxel de la imagen distorsionada (mapas float32)."""
    key = (intr["width"], intr["height"], intr["fx"], intr["fy"], intr["ppx"], intr["ppy"],
           str(intr.get("model", "")), tuple(float(c) for c in intr.get("coeffs", [])[:5]))
    if key in _CACHE:
        return _CACHE[key]
    w, h = int(intr["width"]), int(intr["height"])
    k = np.array([float(c) for c in intr["coeffs"][:5]])
    uu, vv = np.meshgrid(np.arange(w, dtype=np.float64), np.arange(h, dtype=np.float64))
    x = (uu - intr["ppx"]) / intr["fx"]
    y = (vv - intr["ppy"]) / intr["fy"]
    if "inverse" in str(intr.get("model", "")):
        # ideal = g(distorsionado): invertir por punto fijo
        xd, yd = x.copy(), y.copy()
        for _ in range(12):
            gx, gy = _brown(xd, yd, k)
            xd += x - gx
            yd += y - gy
    else:
        xd, yd = _brown(x, y, k)
    mx = (xd * intr["fx"] + intr["ppx"]).astype(np.float32)
    my = (yd * intr["fy"] + intr["ppy"]).astype(np.float32)
    _CACHE[key] = (mx, my)
    return _CACHE[key]


def rectify_snapshot(snap):
    """Copia del snapshot sin distorsión (mismas intrínsecas K, coeficientes a cero)."""
    intr = snap["intrinsics"]
    if not has_distortion(intr):
        return snap
    mx, my = source_map(intr)
    out = dict(snap)
    out["color"] = cv2.remap(snap["color"], mx, my, cv2.INTER_LINEAR, borderMode=cv2.BORDER_CONSTANT)
    out["depth"] = cv2.remap(snap["depth"], mx, my, cv2.INTER_NEAREST, borderMode=cv2.BORDER_CONSTANT)
    if snap.get("heat") is not None and snap["heat"].shape[:2] == snap["depth"].shape:
        out["heat"] = cv2.remap(snap["heat"], mx, my, cv2.INTER_NEAREST, borderMode=cv2.BORDER_CONSTANT)
    ni = dict(intr)
    ni["coeffs"] = [0.0] * 5
    ni["model"] = "rectificada"
    out["intrinsics"] = ni
    return out
