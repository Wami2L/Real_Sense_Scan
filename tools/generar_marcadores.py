"""
Hoja imprimible de marcadores ArUco para alinear las dos cámaras (live3d/alignment.py).

    python tools/generar_marcadores.py                    # marcadores 0-3 de 16 cm -> marcadores_aruco.pdf
    python tools/generar_marcadores.py --ids 0-5 --tamano-cm 10 --salida mis_marcadores.pdf

El tamaño no hace falta saberlo para alinear (el motor usa la profundidad), pero cuanto más
grandes, mejor se detectan a distancia y con ángulo. Imprimir al 100 % (sin «ajustar a página»)
y pegar sobre cartón rígido: el motor ajusta un plano al papel.
"""

import argparse
import io
import os

import cv2
import numpy as np
from PIL import Image, ImageDraw, ImageFont

DPI = 300
A4_MM = (210, 297)
ARUCO_DICT = cv2.aruco.DICT_4X4_50   # el mismo que detecta live3d/alignment.py (distinto del ChArUco)


def _px(mm):
    return int(round(mm / 25.4 * DPI))


def _font(size_px):
    for name in ("arial.ttf", "DejaVuSans.ttf", "LiberationSans-Regular.ttf"):
        try:
            return ImageFont.truetype(name, size_px)
        except OSError:
            continue
    return ImageFont.load_default()


def marker_sheet_pages(ids=(0, 1, 2, 3), size_cm=16.0):
    """Páginas A4 (imágenes PIL a 300 ppp) con tantos marcadores por página como quepan."""
    d = cv2.aruco.getPredefinedDictionary(ARUCO_DICT)
    side = _px(size_cm * 10)
    quiet = _px(8)                      # margen blanco alrededor (zona de silencio + plano del papel)
    label_h = _px(10)
    cell_w, cell_h = side + 2 * quiet, side + 2 * quiet + label_h
    page_w, page_h = _px(A4_MM[0]), _px(A4_MM[1])
    margin, header_h = _px(8), _px(22)
    cols = max(1, (page_w - 2 * margin) // cell_w)
    rows = max(1, (page_h - 2 * margin - header_h) // cell_h)
    per_page = cols * rows
    f_title, f_text, f_small = _font(_px(5)), _font(_px(3.6)), _font(_px(3))
    pages = []
    ids = list(ids)
    for start in range(0, len(ids), per_page):
        page = Image.new("L", (page_w, page_h), 255)
        draw = ImageDraw.Draw(page)
        draw.text((margin, margin), "Marcadores ArUco · RealSenseScans (DICT_4X4_50)", fill=0, font=f_title)
        draw.text((margin, margin + _px(7)),
                  "Imprimir al 100 %. Pegar plano sobre cartón. Colocar donde lo vean AMBAS cámaras, "
                  "orientado hacia el punto medio entre ellas, a 0,6–2 m.", fill=60, font=f_small)
        bar_y = margin + _px(13)                        # barra de 5 cm para comprobar la escala
        draw.rectangle([margin, bar_y, margin + _px(50), bar_y + _px(2)], fill=0)
        draw.text((margin + _px(52), bar_y - _px(1)), "= 5 cm (compruébalo con una regla)", fill=60, font=f_small)
        ox = (page_w - cols * cell_w) // 2
        oy = margin + header_h
        for k, mid in enumerate(ids[start:start + per_page]):
            r, c = divmod(k, cols)
            x0, y0 = ox + c * cell_w, oy + r * cell_h
            img = cv2.aruco.generateImageMarker(d, int(mid), 6 * 40)
            img = cv2.resize(img, (side, side), interpolation=cv2.INTER_NEAREST)
            page.paste(Image.fromarray(img), (x0 + quiet, y0 + quiet))
            draw.rectangle([x0, y0, x0 + cell_w - 1, y0 + cell_h - 1], outline=200, width=2)  # línea de corte
            draw.text((x0 + quiet, y0 + quiet + side + _px(4)),
                      f"ID {mid}   ·   lado {size_cm:g} cm", fill=0, font=f_text)
        pages.append(page.convert("RGB"))
    return pages


def marker_sheet_pdf(ids=(0, 1, 2, 3), size_cm=16.0):
    """PDF en memoria (bytes) listo para imprimir."""
    pages = marker_sheet_pages(ids, size_cm)
    buf = io.BytesIO()
    pages[0].save(buf, "PDF", resolution=DPI, save_all=True, append_images=pages[1:])
    return buf.getvalue()


def _parse_ids(text):
    out = []
    for part in text.split(","):
        a, _, b = part.partition("-")
        out += list(range(int(a), int(b) + 1)) if b else [int(a)]
    if any(not 0 <= i < 50 for i in out):
        raise argparse.ArgumentTypeError("los IDs de DICT_4X4_50 van de 0 a 49")
    return out


def main():
    ap = argparse.ArgumentParser(description="Hoja imprimible de marcadores ArUco (DICT_4X4_50)")
    ap.add_argument("--ids", type=_parse_ids, default=[0, 1, 2, 3], help="p. ej. 0-3 o 0,2,5")
    ap.add_argument("--tamano-cm", type=float, default=16.0, help="lado del marcador negro (cm)")
    ap.add_argument("--salida", default="marcadores_aruco.pdf")
    a = ap.parse_args()
    with open(a.salida, "wb") as f:
        f.write(marker_sheet_pdf(a.ids, a.tamano_cm))
    print(f"{os.path.abspath(a.salida)}: marcadores {a.ids} de {a.tamano_cm:g} cm")


if __name__ == "__main__":
    main()
