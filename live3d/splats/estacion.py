"""
Poda por importancia desde una estación (plan 15 §11.3-C): para Quest 2 se dibujan solo los K splats que más
aportan a la imagen vista desde la zona por la que se mueve el usuario.

Sin GPU: desde varios puntos de la estación se proyectan los centros de los splats opacos a un mapa esférico de
profundidad (oclusión: lo que queda detrás de paredes, bancos o columnas no se ve). Cada splat visible puntúa
alfa × ángulo sólido (σ1·σ2 / d²), que es su aporte de "tinta" a la imagen; la importancia es el máximo sobre los
puntos de vista. Es una aproximación de la significancia global de LightGaussian sin rasterizar.
"""

import numpy as np


def view_points(center, radius=0.75, heights=(1.2, 1.6, 1.9), up=np.array([0.0, -1.0, 0.0]), n_ring=6):
    """Puntos de vista de la estación: centro y un anillo de radio `radius`, a varias alturas sobre el suelo.
    `center` es un punto del suelo (coordenadas del archivo); `up` la normal hacia arriba."""
    up = up / np.linalg.norm(up)
    a = np.cross(up, [1.0, 0, 0])
    if np.linalg.norm(a) < 1e-6:
        a = np.cross(up, [0, 0, 1.0])
    a /= np.linalg.norm(a)
    b = np.cross(up, a)
    ring = [np.zeros(3)] + [radius * (np.cos(t) * a + np.sin(t) * b)
                            for t in np.linspace(0, 2 * np.pi, n_ring, endpoint=False)]
    return np.array([center + h * up + r for h in heights for r in ring])


def _bins(dirs, w, h):
    lon = np.arctan2(dirs[:, 0], dirs[:, 2])                       # -pi..pi
    lat = np.arcsin(np.clip(dirs[:, 1], -1, 1))                    # -pi/2..pi/2
    u = ((lon + np.pi) / (2 * np.pi) * w).astype(np.int64) % w
    v = np.clip(((lat + np.pi / 2) / np.pi * h).astype(np.int64), 0, h - 1)
    return v * w + u


def _splat_depth(depth, b, d, radius_bins, extent, w, h, max_r=6):
    """
    Escribe en el mapa de profundidad (mínimo) cada oclusor con su tamaño angular: un disco de `radius_bins`
    celdas (hasta max_r). Con solo los centros, una pared con pocos splats por celda deja huecos y lo de detrás
    parece visible. Fuera de la celda central se escribe d + extent (lo más lejos que puede estar la superficie del
    splat): un splat grande visto de canto no debe ocultar las partes más lejanas de su propia pared.
    """
    np.minimum.at(depth, b, d)
    v0, u0 = np.divmod(b, w)
    r = np.minimum(radius_bins, max_r)
    d_far = d + extent
    for dy in range(-max_r, max_r + 1):
        for dx in range(-max_r, max_r + 1):
            if dx == 0 and dy == 0:
                continue
            s = r + 0.7 >= np.hypot(dx, dy)          # un disco casi del tamaño de la celda ya toca las vecinas
            if not s.any():
                continue
            vv = np.clip(v0[s] + dy, 0, h - 1)
            uu = (u0[s] + dx) % w
            np.minimum.at(depth, vv * w + uu, d_far[s])


def importance(pos, alpha, sigma, finite, points, res=(1024, 512), occ_alpha=0.6, margin=(1.04, 0.08),
               max_dist=None, normal=None):
    """
    Importancia por splat (float32, 0 = no visible desde ningún punto).
    res: resolución del mapa esférico de oclusión. margin=(factor, metros): un splat es visible si su distancia
    no supera la del oclusor de su dirección (mediana 3×3 del mapa) × factor + metros.
    normal (N,3), opcional (analisis.load_core la da): escorzo de los splats planos vistos de canto, que proyectan
    mucho menos de 2σ en una dirección; sin él, una pared vista de canto se tapa a sí misma.
    """
    w, h = res
    n = len(pos)
    imp = np.zeros(n, np.float32)
    area = (sigma[:, 0] * sigma[:, 1] * alpha).astype(np.float32)
    flatness = (sigma[:, 2] / np.maximum(sigma[:, 1], 1e-12)).astype(np.float32)   # 1 = bola, 0 = disco plano
    occ = finite & (alpha >= occ_alpha)
    bin_rad = np.float32(2 * np.pi / w)
    for p in points:
        v = pos - p.astype(np.float32)
        d = np.sqrt((v * v).sum(1))
        d = np.maximum(d, 1e-3)
        dirs = v / d[:, None]
        b = _bins(dirs, w, h)
        if normal is not None:
            fore = np.maximum(np.abs((normal * dirs).sum(1)), flatness)
        else:
            fore = np.ones(len(d), np.float32)
        depth = np.full(w * h, np.inf, np.float32)
        # Radio angular del oclusor (2σ del eje intermedio, escorzado) en celdas del mapa.
        rad = (2 * sigma[occ, 1] * fore[occ] / d[occ]) / bin_rad
        _splat_depth(depth, b[occ], d[occ], rad, 2 * sigma[occ, 0], w, h)
        dm = depth.reshape(h, w)
        # Mediana 3×3: rellena los agujeros sueltos del mapa (una celda sin pared deja ver lo de detrás) sin
        # desplazar los bordes de las siluetas como haría un mínimo.
        pad = np.pad(dm, 1, mode="edge")
        pad[:, 0], pad[:, -1] = pad[:, -2], pad[:, 1]              # la longitud da la vuelta
        stack = np.stack([pad[1 + dy:h + 1 + dy, 1 + dx:w + 1 + dx] for dy in (-1, 0, 1) for dx in (-1, 0, 1)])
        far = np.median(stack, axis=0).reshape(-1)
        # Tolerancia por pendiente: vista de canto, una celda abarca metros de profundidad de la MISMA superficie
        # (a 30 m de una pared a 3 m de lado, ~2 m por celda de 0,35°). El mayor salto con las celdas vecinas mide
        # esa pendiente; dentro de un pilar es pequeño y el pilar sigue ocultando lo que tiene detrás.
        center = stack[4]
        ok = np.isfinite(stack) & np.isfinite(center)
        with np.errstate(invalid="ignore"):
            slope = np.where(ok, np.abs(stack - center), 0).max(axis=0).reshape(-1)
        vis = finite & (d <= far[b] * margin[0] + margin[1] + 1.5 * slope[b])
        if max_dist is not None:
            vis &= d <= max_dist
        score = np.where(vis, area * fore / (d * d), 0).astype(np.float32)
        np.maximum(imp, score, out=imp)
    return imp


def top_k(imp, k):
    """Máscara con los k splats de mayor importancia (solo entre los visibles)."""
    k = int(min(k, (imp > 0).sum()))
    keep = np.zeros(len(imp), bool)
    if k > 0:
        keep[np.argpartition(-imp, k - 1)[:k]] = True
    return keep
