"""
LOD por fusión de splats desde una estación (plan 15 §12.4, G2Q).

En lugar de borrar lo lejano (la poda por importancia deja agujeros), se FUSIONA: alrededor del centro de la
estación se usa una rejilla esférica cuyas celdas abarcan un ángulo θ visto desde el centro y tienen una
profundidad Δd = d·θ (d medida desde el borde de la zona de movimiento). Cerca, las celdas son milimétricas y los
splats quedan intactos; lejos, cada celda se reduce a una sola gaussiana por ajuste de momentos:

    peso (tinta)   w_i = α_i · π σ1_i σ2_i            (aporte de color a la imagen, σ1 ≥ σ2 los dos ejes mayores)
    media          μ = Σ w_i μ_i / Σ w_i
    covarianza     Σ = Σ w_i (Σ_i + (μ_i − μ)(μ_i − μ)ᵀ) / Σ w_i       (+ inflado opcional (f·L)² I)
    color / SH     media ponderada por w_i
    opacidad       α = 1 − exp(−Σ w_i / (π σ1 σ2))      (cobertura: tinta repartida en el área de la fusionada)

Antes de fusionar se descarta lo que no se ve desde ningún punto de la estación (estacion.importance).
El resultado es 3D (conserva el paralaje al moverse dentro de la estación), a diferencia de un cubemap.
"""

import numpy as np

from .ply import SplatPly, sigmoid


def _quat_to_mat(q):
    """Cuaterniones (N,4) (w, x, y, z) sin normalizar -> matrices de rotación (N,3,3)."""
    q = q / np.maximum(np.linalg.norm(q, axis=1, keepdims=True), 1e-12)
    w, x, y, z = q.T
    return np.stack([
        np.stack([1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)], 1),
        np.stack([2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)], 1),
        np.stack([2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)], 1),
    ], 1)


def _mat_to_quat(m):
    """Matrices de rotación (N,3,3) -> cuaterniones (N,4) (w, x, y, z) normalizados."""
    t = np.trace(m, axis1=1, axis2=2)
    q = np.empty((len(m), 4))
    a = t > 0
    s = np.sqrt(np.maximum(t[a] + 1.0, 1e-12)) * 2
    q[a] = np.stack([0.25 * s, (m[a, 2, 1] - m[a, 1, 2]) / s, (m[a, 0, 2] - m[a, 2, 0]) / s,
                     (m[a, 1, 0] - m[a, 0, 1]) / s], 1)
    for i, (j, k) in enumerate(((1, 2), (2, 0), (0, 1))):
        # Rama por la diagonal mayor cuando la traza no es positiva (estabilidad numérica).
        b = ~a & (m[:, i, i] >= m[:, j, j]) & (m[:, i, i] >= m[:, k, k])
        a = a | b
        s = np.sqrt(np.maximum(1.0 + m[b, i, i] - m[b, j, j] - m[b, k, k], 1e-12)) * 2
        qq = np.empty((b.sum(), 4))
        qq[:, 0] = (m[b, k, j] - m[b, j, k]) / s
        qq[:, 1 + i] = 0.25 * s
        qq[:, 1 + j] = (m[b, j, i] + m[b, i, j]) / s
        qq[:, 1 + k] = (m[b, k, i] + m[b, i, k]) / s
        q[b] = qq
    return q / np.linalg.norm(q, axis=1, keepdims=True)


def cell_keys(pos, eye, theta, r_station=0.75, d_min=0.3, radial=1.0):
    """
    Celda esférica de cada splat: (anillo de distancia, latitud, longitud) con celdas de ≈θ radianes de lado vistas
    desde `eye` (las filas de longitud se acortan con cos(lat) para que las celdas sean de área parecida) y
    profundidad Δd = radial·d_eff·θ, con d_eff = distancia al borde de la zona de movimiento (radio r_station).
    radial > 1 alarga las celdas a lo largo del rayo: una superficie vista con un ángulo γ proyecta su profundidad
    escorzada por sin γ, así que suelos y paredes de canto no necesitan tanta resolución radial.
    Devuelve (clave int64, tamaño lateral L de la celda en metros).
    """
    v = (pos - eye).astype(np.float64)
    d = np.sqrt((v * v).sum(1))
    d = np.maximum(d, 1e-6)
    d_eff = np.maximum(d - r_station, d_min)
    lat = np.arcsin(np.clip(v[:, 1] / d, -1, 1))
    lon = np.arctan2(v[:, 0], v[:, 2])
    n_lat = int(np.ceil(np.pi / theta))
    max_lon = int(np.ceil(2 * np.pi / theta)) + 1
    lat_i = np.clip(np.floor((lat + np.pi / 2) / theta), 0, n_lat - 1).astype(np.int64)
    lat_c = (lat_i + 0.5) * theta - np.pi / 2
    n_lon = np.maximum(1, np.ceil(2 * np.pi * np.cos(lat_c) / theta)).astype(np.int64)
    lon_i = np.floor((lon + np.pi) / (2 * np.pi) * n_lon).astype(np.int64) % n_lon
    r_i = np.floor(np.log(d_eff / d_min) / np.log1p(radial * theta)).astype(np.int64)
    key = (r_i * n_lat + lat_i) * max_lon + lon_i
    return key, theta * d_eff


def merge(cols, key, cell_size, inflate=0.0):
    """
    Fusiona por celda. `cols`: dict de arrays float32 por propiedad del PLY (x, y, z, rot_*, scale_*, opacity,
    f_dc_*, f_rest_*). Devuelve un dict con las mismas propiedades: las celdas de un solo splat se copian tal cual.
    """
    order = np.argsort(key, kind="stable")
    k_sorted = key[order]
    starts = np.r_[0, np.nonzero(np.diff(k_sorted))[0] + 1]
    counts = np.diff(np.r_[starts, len(k_sorted)])
    cell_of = np.empty(len(key), np.int64)
    cell_of[order] = np.repeat(np.arange(len(starts)), counts)
    n_cells = len(starts)
    single = counts[cell_of] == 1
    multi_cells = np.nonzero(counts > 1)[0]

    out = {p: [cols[p][single]] for p in cols}
    if len(multi_cells):
        m = ~single
        remap = np.full(n_cells, -1, np.int64)
        remap[multi_cells] = np.arange(len(multi_cells))
        c = remap[cell_of[m]]
        nc = len(multi_cells)

        pos = np.stack([cols["x"][m], cols["y"][m], cols["z"][m]], 1).astype(np.float64)
        sig = np.exp(np.stack([cols[f"scale_{i}"][m] for i in range(3)], 1).astype(np.float64))
        alpha = sigmoid(cols["opacity"][m].astype(np.float64))
        s_sorted = -np.sort(-sig, axis=1)
        w = alpha * np.pi * s_sorted[:, 0] * s_sorted[:, 1]
        w = np.maximum(w, 1e-30)
        W = np.bincount(c, w, nc)

        def wmean(x):
            return np.bincount(c, w * x, nc) / W

        mu = np.stack([wmean(pos[:, j]) for j in range(3)], 1)
        R = _quat_to_mat(np.stack([cols[f"rot_{i}"][m] for i in range(4)], 1).astype(np.float64))
        cov = np.einsum("nij,nj,nkj->nik", R, sig * sig, R)                  # R diag(σ²) Rᵀ
        dp = pos - mu[c]
        cov += dp[:, :, None] * dp[:, None, :]
        C = np.empty((nc, 3, 3))
        for i in range(3):
            for j in range(i, 3):
                C[:, i, j] = C[:, j, i] = np.bincount(c, w * cov[:, i, j], nc) / W
        if inflate > 0:
            L = np.bincount(c, cell_size[m] * w, nc) / W
            C += ((inflate * L) ** 2)[:, None, None] * np.eye(3)
        lam, V = np.linalg.eigh(C)
        lam = np.maximum(lam, 1e-14)
        V[np.linalg.det(V) < 0, :, 0] *= -1                                   # rotación propia
        sig_m = np.sqrt(lam)
        top2 = sig_m[:, 2] * sig_m[:, 1]                                      # eigh: ascendente
        a_m = np.clip(1.0 - np.exp(-W / (np.pi * top2)), 1 / 255, 0.995)
        q = _mat_to_quat(V)

        merged = {"x": mu[:, 0], "y": mu[:, 1], "z": mu[:, 2],
                  "opacity": np.log(a_m / (1 - a_m))}
        for i in range(3):
            merged[f"scale_{i}"] = np.log(sig_m[:, i])
        for i in range(4):
            merged[f"rot_{i}"] = q[:, i]
        for p in cols:
            if p.startswith("f_dc_") or p.startswith("f_rest_"):
                merged[p] = wmean(cols[p][m].astype(np.float64))
        for p in cols:
            out[p].append(merged[p].astype(np.float32) if p in merged else np.zeros(nc, np.float32))
    return {p: np.concatenate(v) for p, v in out.items()}, {
        "celdas": int(n_cells), "intactos": int(single.sum()), "fusionadas": int(len(multi_cells)),
        "splats_fusionados": int((~single).sum()),
    }


def read_columns(ply: SplatPly, mask=None, sh_degree=None, size=1_000_000):
    """Lee las propiedades del PLY (sin normales, SH recortados a sh_degree) de los splats de `mask`."""
    rest = ply.rest_names(sh_degree)
    props = [p for p in ply.props if not p.startswith("f_rest_") and p not in ("nx", "ny", "nz")] + rest
    n = int(mask.sum()) if mask is not None else ply.count
    cols = {p: np.empty(n, np.float32) for p in props}
    k = 0
    for s, b in ply.chunks(size):
        if mask is not None:
            b = b[mask[s:s + len(b)]]
        for p in props:
            cols[p][k:k + len(b)] = b[p]
        k += len(b)
    # f_rest renumerados 0..n-1 como espera el formato al recortar grados.
    for i, p in enumerate(rest):
        if p != f"f_rest_{i}":
            cols[f"f_rest_{i}"] = cols.pop(p)
    return cols
