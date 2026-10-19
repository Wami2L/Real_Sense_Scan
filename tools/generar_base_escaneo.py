"""
Hoja imprimible de la BASE DE ESCANEO de objetos (live3d/objscan/base.py).

    python tools/generar_base_escaneo.py                 # A3 -> base_escaneo_A3.pdf
    python tools/generar_base_escaneo.py --hoja A4       # A4 (objetos de hasta ~10 cm)

Se pone el objeto en el círculo central. Cada toma ve algunos marcadores alrededor y con ellos
se conoce la posición EXACTA de la cámara respecto a la hoja: así las vistas encajan sin
depender de la textura del objeto. Imprimir al 100 % (sin "ajustar a página"); si la escala
no es exacta el escáner lo detecta con la profundidad y lo corrige, pero mejor que sea exacta.
Pegarla plana sobre cartón (o una tabla) para poder girarla con el objeto encima.
"""

import argparse
import io
import os
import sys

import numpy as np
from PIL import Image, ImageDraw, ImageFont

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from live3d.objscan.base import LAYOUTS, render_sheet  # noqa: E402

DPI = 300


def _font(px):
    for name in ("arial.ttf", "DejaVuSans.ttf", "LiberationSans-Regular.ttf"):
        try:
            return ImageFont.truetype(name, px)
        except OSError:
            continue
    return ImageFont.load_default()


def base_sheet_image(layout):
    ppm = DPI / 25.4
    img = Image.fromarray(render_sheet(layout, px_per_mm=ppm, guides=True)).convert("RGB")
    d = ImageDraw.Draw(img)
    W, H = img.size
    f, fs = _font(int(4.2 * ppm)), _font(int(2.8 * ppm))
    r = layout.free_radius_m() * 1000 * ppm
    cx, cy = W / 2, H / 2
    title = f"BASE DE ESCANEO {layout.name} · RealSenseScans"
    tw = d.textlength(title, font=f)
    d.text((cx - tw / 2, cy + r * 0.55), title, fill=(120, 120, 120), font=f)
    t2 = "objeto en el centro · imprimir al 100 % · pegar plano"
    d.text((cx - d.textlength(t2, font=fs) / 2, cy + r * 0.55 + 6 * ppm), t2, fill=(150, 150, 150), font=fs)
    # barra de escala de 5 cm en el hueco entre los marcadores de la izquierda
    bar_y = int((layout.margin_mm + layout.marker_mm + 8) * ppm)
    x0 = int(layout.margin_mm * ppm)
    d.rectangle([x0, bar_y, x0 + int(50 * ppm), bar_y + int(1.5 * ppm)], fill=(0, 0, 0))
    d.text((x0, bar_y + int(2.5 * ppm)), "5 cm (compruébalo con una regla)", fill=(110, 110, 110), font=fs)
    return img


def base_sheet_pdf(name="A3"):
    buf = io.BytesIO()
    base_sheet_image(LAYOUTS[name]).save(buf, "PDF", resolution=DPI)
    return buf.getvalue()


def main():
    ap = argparse.ArgumentParser(description="Hoja imprimible de la base de escaneo de objetos")
    ap.add_argument("--hoja", choices=sorted(LAYOUTS), default="A3")
    ap.add_argument("--salida", default=None)
    a = ap.parse_args()
    out = a.salida or f"base_escaneo_{a.hoja}.pdf"
    with open(out, "wb") as fh:
        fh.write(base_sheet_pdf(a.hoja))
    lay = LAYOUTS[a.hoja]
    print(f"{os.path.abspath(out)}: hoja {a.hoja} ({lay.width_mm:g}×{lay.height_mm:g} mm), "
          f"marcadores {list(lay.marker_origins_mm())} de {lay.marker_mm:g} mm")


if __name__ == "__main__":
    main()
