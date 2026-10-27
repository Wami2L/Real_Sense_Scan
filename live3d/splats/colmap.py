"""
Cámaras de un modelo COLMAP (images.txt) como referencia de una escena 3DGS entrenada sobre él (plan 15 §14).

COLMAP no fija gravedad ni escala. Con las cámaras de un vídeo grabado a pie sí se pueden estimar:
- arriba: el "arriba" medio de las cámaras (en la imagen, −y) es una primera aproximación, sesgada por la inclinación
  con la que se sostuvo el móvil; el suelo se busca después con RANSAC entre los splats bajos, cerca de esa dirección;
- escala: las cámaras van a la altura de la mano (≈1,5 m) sobre ese suelo.
"""

import numpy as np


def _quat_to_mat(qw, qx, qy, qz):
    return np.array([
        [1 - 2 * (qy * qy + qz * qz), 2 * (qx * qy - qw * qz), 2 * (qx * qz + qw * qy)],
        [2 * (qx * qy + qw * qz), 1 - 2 * (qx * qx + qz * qz), 2 * (qy * qz - qw * qx)],
        [2 * (qx * qz - qw * qy), 2 * (qy * qz + qw * qx), 1 - 2 * (qx * qx + qy * qy)],
    ])


def read_cameras(images_txt):
    """Centros (N,3), "arriba" (N,3) y "adelante" (N,3) de las cámaras en el mundo COLMAP (convención OpenCV)."""
    centers, ups, fwds = [], [], []
    with open(images_txt, encoding="utf-8", errors="replace") as f:
        lines = [ln for ln in f if not ln.startswith("#")]
    i = 0
    while i < len(lines):
        p = lines[i].split()
        # Línea de imagen: IMAGE_ID QW QX QY QZ TX TY TZ CAMERA_ID NAME; la siguiente son sus puntos 2D.
        if len(p) >= 10:
            qw, qx, qy, qz, tx, ty, tz = map(float, p[1:8])
            R = _quat_to_mat(qw, qx, qy, qz)                 # mundo -> cámara
            centers.append(-R.T @ np.array([tx, ty, tz]))
            ups.append(R.T @ np.array([0.0, -1.0, 0.0]))
            fwds.append(R.T @ np.array([0.0, 0.0, 1.0]))
            i += 2
        else:
            i += 1
    if not centers:
        raise ValueError(f"{images_txt}: no hay cámaras")
    return np.array(centers), np.array(ups), np.array(fwds)


def floor_from_cameras(pos, alpha, finite, centers, ups, max_tilt=35.0, tol=None, seed=0, iters=3000):
    """
    Suelo de la escena guiado por las cámaras. Devuelve un dict como analisis.floor_plane (normal_arriba, punto,
    inclinacion_grados respecto al eje más cercano, eje) más altura_camaras (mediana de la altura de las cámaras sobre
    el suelo, en unidades del archivo) y angulo_con_camaras.
    """
    up = ups.mean(0)
    up /= np.linalg.norm(up)
    h = pos @ up
    sel = finite & (alpha > 0.5) & (h < np.percentile(h[finite], 12))
    P = pos[sel].astype(np.float64)
    if len(P) < 50:
        raise ValueError("muy pocos splats opacos bajos para buscar el suelo")
    rng = np.random.default_rng(seed)
    if len(P) > 300_000:
        P = P[rng.choice(len(P), 300_000, replace=False)]
    if tol is None:                                             # 0,1 % de la extensión de la escena
        tol = 0.001 * float(np.linalg.norm(np.percentile(pos[finite], 99, 0) - np.percentile(pos[finite], 1, 0)))
    cos_max = np.cos(np.radians(max_tilt))
    best, best_n = None, -1
    for _ in range(iters):
        a, b, c = P[rng.choice(len(P), 3, replace=False)]
        n = np.cross(b - a, c - a)
        nn = np.linalg.norm(n)
        if nn < 1e-12:
            continue
        n /= nn
        if abs(n @ up) < cos_max:
            continue
        cnt = int((np.abs((P - a) @ n) < tol).sum())
        if cnt > best_n:
            best, best_n = (n, a), cnt
    if best is None:
        raise ValueError("no se encontró un plano de suelo compatible con las cámaras")
    n, a = best
    inl = P[np.abs((P - a) @ n) < tol]
    cen = inl.mean(0)
    n = np.linalg.svd(inl - cen, full_matrices=False)[2][-1]
    if n @ up < 0:
        n = -n
    heights = (centers - cen) @ n
    ax = int(np.argmax(np.abs(n)))
    axis = np.zeros(3)
    axis[ax] = np.sign(n[ax])
    return {
        "normal_arriba": n.tolist(), "punto": cen.tolist(),
        "inclinacion_grados": float(np.degrees(np.arccos(np.clip(abs(n[ax]), -1, 1)))),
        "eje": ("+" if n[ax] > 0 else "-") + "xyz"[ax],
        "inliers": int(len(inl)), "muestra": int(len(P)),
        "altura_camaras": float(np.median(heights)),
        "altura_camaras_p5_p95": [float(np.percentile(heights, 5)), float(np.percentile(heights, 95))],
        "angulo_con_camaras": float(np.degrees(np.arccos(np.clip(n @ up, -1, 1)))),
        "fuente": "camaras_colmap",
    }
