"""
Vistas previas rápidas de una escena 3DGS a partir de los centros de los splats (sin rasterizar gaussianas):
sirven para entender la planta, elegir el eje "arriba", ver flotantes y comparar limpiezas, no para juzgar la
calidad final (eso se hace en Unity con gsplat).
"""

import numpy as np
from PIL import Image

_AXES = {"x": 0, "y": 1, "z": 2}


def ortho(pos, color, alpha, view="y", flip=False, res=1024, mode="cercano", clip=None, bounds=None,
          min_alpha=0.3, highlight=None):
    """
    Proyección ortográfica a lo largo del eje `view` (x, y o z).
      mode "cercano": el splat opaco más cercano al observador por píxel (vista de superficie).
      mode "densidad": log del número de splats por píxel (rayos X; deja ver flotantes y estructura).
    clip=(a, b) recorta la escena en el eje de vista (cortes de planta). flip mira desde el lado negativo.
    highlight: máscara bool de splats a pintar en rojo encima (p. ej. los que se van a quitar).
    """
    ax = _AXES[view]
    u, v = [i for i in range(3) if i != ax]
    sel = np.isfinite(pos).all(1)
    if clip is not None:
        sel &= (pos[:, ax] >= clip[0]) & (pos[:, ax] <= clip[1])
    if bounds is None:
        lo, hi = np.percentile(pos[sel], [0.5, 99.5], axis=0)
    else:
        lo, hi = bounds
    span = max(hi[u] - lo[u], hi[v] - lo[v])
    w = res if (hi[u] - lo[u]) >= (hi[v] - lo[v]) else max(1, int(res * (hi[u] - lo[u]) / span))
    h = res if (hi[v] - lo[v]) >= (hi[u] - lo[u]) else max(1, int(res * (hi[v] - lo[v]) / span))
    px = ((pos[:, u] - lo[u]) / span * (res - 1)).astype(np.int64)
    py = ((pos[:, v] - lo[v]) / span * (res - 1)).astype(np.int64)
    sel &= (px >= 0) & (px < w) & (py >= 0) & (py < h)
    pix = py * w + px

    if mode == "densidad":
        cnt = np.bincount(pix[sel], minlength=w * h).astype(np.float64)
        img = np.log1p(cnt) / max(np.log1p(cnt).max(), 1e-9)
        rgb = (np.stack([img, img, img], 1) * 255).astype(np.uint8)
    else:
        s = sel & (alpha >= min_alpha)
        depth = pos[:, ax] if flip else -pos[:, ax]      # el más cercano al observador va primero
        idx = np.nonzero(s)[0]
        order = idx[np.lexsort((depth[idx], pix[idx]))]
        first = order[np.unique(pix[order], return_index=True)[1]]
        rgb = np.full((w * h, 3), 18, np.uint8)
        rgb[pix[first]] = (np.clip(color[first], 0, 1) * 255).astype(np.uint8)
    if highlight is not None:
        hl = sel & highlight
        rgb[pix[hl]] = (255, 40, 40)
    img = rgb.reshape(h, w, 3)[::-1]                     # v hacia arriba en la imagen
    return Image.fromarray(img)


def save_contact_sheet(images, labels, path, pad=8):
    """Une varias vistas en una sola imagen con su etiqueta."""
    from PIL import ImageDraw
    hmax = max(im.height for im in images)
    total_w = sum(im.width for im in images) + pad * (len(images) + 1)
    sheet = Image.new("RGB", (total_w, hmax + 2 * pad + 22), (10, 10, 10))
    draw = ImageDraw.Draw(sheet)
    x = pad
    for im, lab in zip(images, labels):
        sheet.paste(im, (x, pad + 22))
        draw.text((x, pad), lab, fill=(230, 230, 230))
        x += im.width + pad
    sheet.save(path)
    return path
