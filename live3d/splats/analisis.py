"""
Diagnóstico de una escena 3DGS: qué la hace pesada (plan 15 §3) y qué se puede quitar sin que se note.

Todo en las unidades del archivo (un splat de internet no trae escala métrica; se fija después con una medida
conocida). Las magnitudes se dan también relativas a la diagonal de la "zona principal" (percentiles 0,5-99,5).
"""

import numpy as np

from .ply import SH_C0, SplatPly, sigmoid

# Los armónicos reales del formato son ortonormales en la esfera: la media de (Σ c·Y)² sobre todas las
# direcciones es Σ c² / 4π. Su raíz es el RMS del color que añaden los SH al mirar desde cualquier lado.
_SH_RMS = 1.0 / np.sqrt(4.0 * np.pi)


def load_core(ply: SplatPly, size=1_000_000):
    """
    Propiedades derivadas por splat, leídas por bloques (sin cargar los f_rest enteros):
    pos (N,3), alpha (N,), sigma (N,3) ordenadas de mayor a menor, color (N,3) base 0-1,
    sh_amp (N,) RMS (peor canal, 0-1) del color que depende de la vista (0 si no hay SH), finite (N,) bool.
    """
    ply.require("x", "y", "z", "opacity", "scale_0", "scale_1", "scale_2", "f_dc_0", "f_dc_1", "f_dc_2")
    n = ply.count
    out = {
        "pos": np.empty((n, 3), np.float32), "alpha": np.empty(n, np.float32),
        "sigma": np.empty((n, 3), np.float32), "color": np.empty((n, 3), np.float32),
        "sh_amp": np.zeros(n, np.float32), "finite": np.empty(n, bool),
        "normal": np.zeros((n, 3), np.float32),
    }
    has_rot = all(f"rot_{i}" in ply.props for i in range(4))
    per = ply.sh_rest // 3
    with np.errstate(over="ignore", invalid="ignore"):
        for s, b in ply.chunks(size):
            e = s + len(b)
            pos = np.stack([b["x"], b["y"], b["z"]], 1)
            logit = b["opacity"]
            ls = np.stack([b["scale_0"], b["scale_1"], b["scale_2"]], 1)
            dc = np.stack([b["f_dc_0"], b["f_dc_1"], b["f_dc_2"]], 1)
            fin = np.isfinite(pos).all(1) & np.isfinite(logit) & np.isfinite(ls).all(1) & np.isfinite(dc).all(1)
            amp = np.zeros(len(b), np.float32)
            for c in range(3):
                if not per:
                    break
                coeffs = np.stack([b[f"f_rest_{c * per + k}"] for k in range(per)], 1)
                fin &= np.isfinite(coeffs).all(1)
                amp = np.maximum(amp, _SH_RMS * np.sqrt((coeffs * coeffs).sum(1)))
            out["pos"][s:e] = pos
            out["alpha"][s:e] = sigmoid(logit)
            out["sigma"][s:e] = -np.sort(-np.exp(ls), axis=1)
            out["color"][s:e] = np.clip(0.5 + SH_C0 * dc, 0, 1)
            out["sh_amp"][s:e] = amp
            out["finite"][s:e] = fin
            if has_rot:
                # Normal = eje local de σ mínima, rotado: columna de R(q) (w, x, y, z).
                q = np.stack([b[f"rot_{i}"] for i in range(4)], 1).astype(np.float64)
                q /= np.maximum(np.linalg.norm(q, axis=1, keepdims=True), 1e-12)
                w, x, y, z = q.T
                cols = (np.stack([1 - 2 * (y * y + z * z), 2 * (x * y + w * z), 2 * (x * z - w * y)], 1),
                        np.stack([2 * (x * y - w * z), 1 - 2 * (x * x + z * z), 2 * (y * z + w * x)], 1),
                        np.stack([2 * (x * z + w * y), 2 * (y * z - w * x), 1 - 2 * (x * x + y * y)], 1))
                k = np.argmin(ls, axis=1)
                nrm = np.choose(k[:, None], cols)
                out["normal"][s:e] = np.nan_to_num(nrm)
    return out


def main_region(pos, finite, lo=0.5, hi=99.5):
    p = pos[finite]
    a, b = np.percentile(p, [lo, hi], axis=0)
    return a, b


def neighbourhood_counts(pos, voxel, origin):
    """Splats en el cubo 3×3×3 de vóxeles alrededor de cada splat (densidad local, para flotantes)."""
    ijk = np.floor((pos - origin) / voxel).astype(np.int64)
    ijk -= ijk.min(0)
    dims = ijk.max(0) + 3
    key = ((ijk[:, 0] + 1) * dims[1] + (ijk[:, 1] + 1)) * dims[2] + (ijk[:, 2] + 1)
    uniq, inv, cnt = np.unique(key, return_inverse=True, return_counts=True)
    total = np.zeros(len(uniq), np.int64)
    for dx in (-1, 0, 1):
        for dy in (-1, 0, 1):
            for dz in (-1, 0, 1):
                nk = uniq + (dx * dims[1] + dy) * dims[2] + dz
                j = np.searchsorted(uniq, nk)
                j = np.clip(j, 0, len(uniq) - 1)
                hit = uniq[j] == nk
                total[hit] += cnt[j[hit]]
    return total[inv], len(uniq)


def up_axis_guess(pos, finite, alpha, bins=400):
    """
    Busca el suelo: una capa fina y densa de splats opacos en un extremo de un eje. Devuelve una lista de
    candidatos (eje, signo hacia arriba, pico relativo) ordenada de más a menos probable.
    """
    sel = finite & (alpha > 0.5)
    p = pos[sel]
    lo, hi = np.percentile(p, [0.5, 99.5], axis=0)
    cands = []
    for ax in range(3):
        h, edges = np.histogram(p[:, ax], bins=bins, range=(lo[ax], hi[ax]))
        h = h / max(1, h.mean())
        edge = bins // 6
        bottom, top = h[:edge].max(), h[-edge:].max()
        # El suelo está en el extremo con el pico: "arriba" apunta hacia el otro lado.
        if bottom >= top:
            cands.append({"eje": "xyz"[ax], "arriba": "+", "pico": float(bottom),
                          "suelo": float(edges[int(np.argmax(h[:edge]))])})
        else:
            cands.append({"eje": "xyz"[ax], "arriba": "-", "pico": float(top),
                          "suelo": float(edges[bins - edge + int(np.argmax(h[-edge:]))])})
    return sorted(cands, key=lambda c: -c["pico"])


def floor_plane(pos, finite, alpha, cand, band=0.4, iters=400, tol=0.03, seed=0):
    """
    Plano del suelo por RANSAC con los splats opacos cerca del nivel del candidato `cand` (de up_axis_guess).
    Devuelve normal unitaria hacia arriba, un punto del plano, inclinación respecto al eje (grados) e inliers.
    """
    ax = "xyz".index(cand["eje"])
    sel = finite & (alpha > 0.5) & (np.abs(pos[:, ax] - cand["suelo"]) < band)
    p = pos[sel].astype(np.float64)
    rng = np.random.default_rng(seed)
    if len(p) > 400_000:
        p = p[rng.choice(len(p), 400_000, replace=False)]
    best, best_n = None, -1
    for _ in range(iters):
        a, b, c = p[rng.choice(len(p), 3, replace=False)]
        n = np.cross(b - a, c - a)
        if np.linalg.norm(n) < 1e-9:
            continue
        n /= np.linalg.norm(n)
        cnt = int((np.abs((p - a) @ n) < tol).sum())
        if cnt > best_n:
            best, best_n = (n, a), cnt
    n, a = best
    inl = p[np.abs((p - a) @ n) < tol]
    centroid = inl.mean(0)                       # ajuste final por mínimos cuadrados (PCA de los inliers)
    n = np.linalg.svd(inl - centroid, full_matrices=False)[2][-1]
    axis = np.zeros(3)
    axis[ax] = 1.0 if cand["arriba"] == "+" else -1.0
    if n @ axis < 0:
        n = -n
    tilt = float(np.degrees(np.arccos(np.clip(n @ axis, -1, 1))))
    return {"normal_arriba": n.tolist(), "punto": centroid.tolist(), "inclinacion_grados": tilt,
            "inliers": int(len(inl)), "muestra": int(len(p)), "eje": cand["arriba"] + cand["eje"]}


def inspect(ply: SplatPly, core=None, voxel_frac=1 / 200):
    """Informe (dict serializable a JSON) con lo que hace pesada la escena."""
    c = core if core is not None else load_core(ply)
    n = ply.count
    fin = c["finite"]
    lo, hi = main_region(c["pos"], fin)
    diag = float(np.linalg.norm(hi - lo))
    pos, alpha, sigma, amp = c["pos"], c["alpha"], c["sigma"], c["sh_amp"]

    inside = fin & np.all((pos >= lo - 0.1 * (hi - lo)) & (pos <= hi + 0.1 * (hi - lo)), 1)
    voxel = diag * voxel_frac
    neigh = np.zeros(n, np.int64)
    neigh[inside], occupied = neighbourhood_counts(pos[inside], voxel, lo)

    # Área de la elipse (dos ejes mayores) × alfa: cuánto "relleno" aporta cada splat, a igual distancia.
    footprint = np.where(fin, sigma[:, 0] * sigma[:, 1] * alpha, 0).astype(np.float64)
    order = np.argsort(-footprint)
    cum = np.cumsum(footprint[order]) / max(footprint.sum(), 1e-30)

    def share_top(frac):
        return float(cum[max(0, int(frac * n) - 1)])

    def pct(x, qs=(1, 5, 25, 50, 75, 95, 99, 99.9)):
        return {str(q): float(v) for q, v in zip(qs, np.percentile(x, qs))}

    smax = sigma[fin, 0]
    aniso = sigma[fin, 0] / np.maximum(sigma[fin, 2], 1e-12)
    rep = {
        "archivo": ply.path,
        "splats": n,
        "propiedades": len(ply.props),
        "grado_sh": ply.sh_degree,
        "bytes_por_splat": ply.bytes_per_splat,
        "tamano_mb": round(n * ply.bytes_per_splat / 2**20, 1),
        "no_finitos": int((~fin).sum()),
        "zona_principal": {"min": lo.tolist(), "max": hi.tolist(), "tamano": (hi - lo).tolist(), "diagonal": diag},
        "fuera_de_zona_10pct": int((fin & ~inside).sum()),
        "opacidad": {
            "percentiles": pct(alpha[fin]),
            "bajo": {str(t): int((alpha[fin] < t).sum()) for t in (0.005, 0.01, 0.02, 0.05, 0.1)},
        },
        "tamano_sigma_max": {
            "percentiles": pct(smax),
            "relativo_diagonal": {str(t): int((smax > t * diag).sum()) for t in (0.002, 0.005, 0.01, 0.02, 0.05)},
            "agujas_ratio_mayor_100": int((aniso > 100).sum()),
        },
        "relleno": {
            "cuota_1pct_mas_grandes": share_top(0.01),
            "cuota_5pct": share_top(0.05),
            "cuota_10pct": share_top(0.10),
        },
        "sh": {
            "amplitud_percentiles": pct(amp[fin]) if ply.sh_degree else {},
            "bajo_umbral": {str(t): int((amp[fin] < t).sum()) for t in (0.01, 0.02, 0.05)} if ply.sh_degree else {},
            "memoria_sh_mb": round(n * ply.sh_rest * 4 / 2**20, 1),
        },
        "densidad": {
            "voxel": voxel,
            "voxeles_ocupados": int(occupied),
            "vecinos_percentiles": pct(neigh[inside]),
            "aislados": {str(t): int((inside & (neigh < t)).sum()) for t in (2, 4, 8, 16)},
        },
        "eje_arriba": up_axis_guess(pos, fin, alpha),
    }
    rep["suelo"] = floor_plane(pos, fin, alpha, rep["eje_arriba"][0])
    return rep, {"neigh": neigh, "inside": inside, "footprint": footprint}
