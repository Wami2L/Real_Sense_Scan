"""
Vuelve a procesar un objeto desde sus tomas guardadas, sin cámara (p. ej. tras mejorar el código).

    python tools/reprocesar_objeto.py Saves/2026-10-21/objeto_162841 --nombre "Caja RealSense"
    python tools/reprocesar_objeto.py Saves/2026-10-21/objeto_162841 --reemplazar 20261021_164135_realsense_box

Crea un objeto nuevo en la biblioteca (Saves/objetos/). Con --reemplazar, conserva el nombre y el
estado de publicación del objeto indicado y lo borra al terminar bien.
"""

import argparse
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from live3d.objscan.library import Library  # noqa: E402
from live3d.objscan.session import ObjectScanSession  # noqa: E402


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("sesion", help="carpeta de la sesión (Saves/AAAA-MM-DD/objeto_HHMMSS)")
    ap.add_argument("--nombre", help="nombre del objeto nuevo")
    ap.add_argument("--reemplazar", metavar="ID", help="id de la biblioteca al que sustituye")
    ap.add_argument("--sin-textura", action="store_true", help="solo color por vértice")
    a = ap.parse_args()

    lib = Library()
    old = lib.get(a.reemplazar) if a.reemplazar else None
    name = a.nombre or (old or {}).get("nombre")
    s = ObjectScanSession.from_folder(a.sesion, name=name, library=lib)
    print(f"{len(s.views)} vistas de {s.shots} tomas · vóxel {s.voxel * 1000:.2f} mm")
    s.state = "processing"
    s._process(150000, texture=not a.sin_textura)
    if s.error:
        print(s.error)
        return 1
    man = s.result
    if old is not None:
        man = lib.update(man["id"], publicado=old.get("publicado", False), version=int(old.get("version", 1)) + 1)
        lib.delete(old["id"])
    print(f"Objeto {man['id']}: {man['tamano_m']} m, {man['triangulos']} triángulos, "
          f"textura {(man.get('textura') or {}).get('texels_vistos')}, {man['tiempo_proceso_s']} s")
    return 0


if __name__ == "__main__":
    sys.exit(main())
