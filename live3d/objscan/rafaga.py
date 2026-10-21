"""
Ráfaga de cuadros por toma: la mediana de N profundidades reduce el ruido temporal de la D455
(≈ σ/√N; parte del ruido es fijo por píxel y no baja) y rellena huecos sueltos. El objeto y la
cámara deben estar quietos durante la ráfaga (~0.3 s): los cuadros que cambian demasiado respecto
al último (algo se movió) se descartan.
"""

import numpy as np

NEAR_M = 1.5          # solo cuenta la escena cercana (el fondo lejano es ruidoso y no importa)
MOVED_FRAC = 0.02     # > 2 % de los píxeles cercanos con un salto grande = algo se movió


def change_frac(a, b, depth_scale=0.001):
    """Fracción de píxeles cercanos (válidos en ambos) cuya profundidad cambió más que el ruido
    esperable (1.5 cm o 2 % de z) entre dos cuadros uint16."""
    za, zb = a.astype(np.float32) * depth_scale, b.astype(np.float32) * depth_scale
    valid = (za > 0) & (zb > 0) & (zb < NEAR_M)
    n = int(valid.sum())
    if n < 1000:
        return 0.0
    tol = np.maximum(0.015, 0.02 * zb[valid])
    return float(np.mean(np.abs(za[valid] - zb[valid]) > tol))


def fuse_depths(snaps):
    """Lista de snapshots de UNA cámara -> snapshot con la profundidad mediana.
    Claves añadidas: "burst" (cuadros usados) y "burst_cambio" (fracción de cambio de cada cuadro)."""
    ref = snaps[-1]
    ds = ref.get("depth_scale", 0.001)
    fracs = [change_frac(s["depth"], ref["depth"], ds) for s in snaps[:-1]]
    keep = [s for s, f in zip(snaps[:-1], fracs) if f < MOVED_FRAC] + [ref]
    out = dict(ref)
    out["color"] = keep[len(keep) // 2]["color"]           # el del centro de la ráfaga
    out["burst_cambio"] = [round(f, 4) for f in fracs]
    if len(keep) == 1:
        out["burst"] = 1
        return out
    D = np.stack([s["depth"] for s in keep]).astype(np.float32)
    D[D == 0] = np.nan
    cnt = np.sum(np.isfinite(D), axis=0)
    with np.errstate(all="ignore"):
        med = np.nanmedian(D, axis=0)
    med[(cnt < max(2, (len(keep) + 1) // 2)) | ~np.isfinite(med)] = 0  # dato en al menos la mitad
    out["depth"] = np.round(med).astype(np.uint16)
    out["burst"] = len(keep)
    return out


def fuse_burst(snapshots):
    """Snapshots simples o duales ({"cam1", "cam2", ...}) -> uno solo con la profundidad fusionada."""
    first = snapshots[-1]
    if not any(first.get(k) is not None for k in ("cam1", "cam2")):
        return fuse_depths(snapshots)
    out = dict(first)
    for k in ("cam1", "cam2"):
        parts = [s[k] for s in snapshots if s.get(k) is not None]
        if parts:
            out[k] = fuse_depths(parts)
    return out


def capture_burst(src, n=6, timeout=6.0):
    """Toma n snapshots seguidos de la fuente y los fusiona."""
    snaps = [src.get_snapshot(timeout) for _ in range(max(1, n))]
    return fuse_burst(snaps)
