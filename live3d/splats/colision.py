"""
Colisiones de un entorno 3DGS para Unity (plan 15 G5): los splats no tienen geometría, así que se voxeliza la escena
con los splats opacos y pequeños (los que dibujan superficies), se limpian los flotantes y se tapan los huecos, y la
ocupación se agrupa en cajas (fusión voraz x → z → y). Unity crea un BoxCollider por caja (ScanWorld › Escena VR).

Las cajas salen ya en ejes de Unity ((x, y, z) del archivo RDF -> (x, -y, z)) y en el marco del PLY limpio
(trasladado: el suelo queda en y = 0), el mismo en que se importan las pirámides. Con `rotacion` (la rotacion_unity
de la limpieza, que Unity aplica a la raíz del entorno) salen ya niveladas, en el marco del mundo: en una escena de
COLMAP el suelo del archivo puede estar inclinado y las cajas alineadas con los ejes deben estarlo con la gravedad.
"""

from dataclasses import dataclass

import numpy as np

from .ply import SplatPly, sigmoid


@dataclass
class Opciones:
    voxel: float = 0.2            # m: más fino = más cajas y más fiel; 0,2 deja pasar por puertas de > 0,6 m
    min_alfa: float = 0.3         # splats casi transparentes no son superficie
    max_sigma: float = 0.2        # m: los splats gigantes (relleno, cielo) no son superficie
    min_splats: int = 5           # splats por vóxel para contarlo ocupado
    min_componente: int = 8       # vóxeles: grupos más pequeños son flotantes
    banda_suelo: tuple = (-0.3, 0.25)   # m: alturas que cuentan como suelo para taparle los huecos


def rotar(q, v):
    """Rota los vectores v (N×3) con el cuaternión q = (x, y, z, w) de Unity."""
    u, w = np.asarray(q[:3], np.float64), float(q[3])
    return v + 2 * np.cross(u, np.cross(u, v) + w * v)


def ocupacion(ply: SplatPly, o: Opciones, log=print, rotacion=None):
    """Rejilla booleana (ejes de Unity) y su origen: vóxel ijk ocupa [origen + ijk·voxel, origen + (ijk+1)·voxel]."""
    c = ply.columns(["x", "y", "z", "opacity", "scale_0", "scale_1", "scale_2"])
    pos = c[:, :3].astype(np.float64)
    pos[:, 1] *= -1                                   # RDF -> Unity
    if rotacion is not None:
        pos = rotar(rotacion, pos)                    # nivelada: marco del mundo
    keep = (sigmoid(c[:, 3]) > o.min_alfa) & (np.exp(c[:, 4:7].max(1)) < o.max_sigma) & np.isfinite(pos).all(1)
    pos = pos[keep]
    log(f"  {keep.sum():,} de {len(keep):,} splats son superficie (alfa > {o.min_alfa}, sigma < {o.max_sigma} m)")
    # Origen alineado con la rejilla y con y = 0 en un borde de vóxel: el suelo queda a ras.
    origin = np.floor(pos.min(0) / o.voxel) * o.voxel
    ijk = np.floor((pos - origin) / o.voxel).astype(np.int64)
    dims = ijk.max(0) + 1
    cnt = np.zeros(dims, np.int32)
    np.add.at(cnt, tuple(ijk.T), 1)
    return cnt >= o.min_splats, origin


def limpiar(occ, origin, o: Opciones, log=print):
    """Quita flotantes, cierra rendijas de un vóxel y tapa los huecos del suelo (bajo bancos, zonas poco vistas)."""
    from scipy import ndimage
    lab, n = ndimage.label(occ, structure=np.ones((3, 3, 3), bool))
    sizes = np.bincount(lab.ravel())
    sizes[0] = 0
    occ = sizes[lab] >= o.min_componente
    log(f"  {n:,} grupos; quedan {occ.sum():,} vóxeles sin flotantes")
    occ = ndimage.binary_closing(occ, structure=ndimage.generate_binary_structure(3, 1)) | occ

    # Suelo: huella de lo ocupado en la banda del suelo, cerrada y sin agujeros, como losa justo bajo y = 0.
    y0 = int(round(-origin[1] / o.voxel))             # índice del primer vóxel por encima de y = 0
    lo = max(0, y0 + int(np.floor(o.banda_suelo[0] / o.voxel)))
    hi = min(occ.shape[1], y0 + int(np.ceil(o.banda_suelo[1] / o.voxel)))
    if 0 < y0 <= occ.shape[1] and hi > lo:
        huella = occ[:, lo:hi, :].any(1)
        huella = ndimage.binary_closing(huella, structure=np.ones((5, 5), bool), iterations=2) | huella
        huella = ndimage.binary_fill_holes(huella)
        occ[:, y0 - 1, :] |= huella
        # La capa justo encima del suelo es casi toda ruido de los splats del suelo (escalones fantasma de 20 cm):
        # se queda solo donde sigue algo por encima (pies de bancos, arranque de paredes y columnas).
        if y0 + 2 <= occ.shape[1]:
            ruido = occ[:, y0, :] & huella & ~occ[:, y0 + 1, :]
            occ[:, y0, :] &= ~ruido
            log(f"  suelo: {huella.sum() * o.voxel ** 2:.0f} m² a y = 0 ({ruido.sum():,} vóxeles de ruido encima)")
    return occ


def cajas(occ):
    """Agrupa la ocupación en cajas (i0, j0, k0, ni, nj, nk) creciendo en x, luego z, luego y."""
    libre = occ.copy()
    out = []
    X, Y, Z = occ.shape
    for i, j, k in np.argwhere(occ):
        if not libre[i, j, k]:
            continue
        i1 = i + 1
        while i1 < X and libre[i1, j, k]:
            i1 += 1
        k1 = k + 1
        while k1 < Z and libre[i:i1, j, k1].all():
            k1 += 1
        j1 = j + 1
        while j1 < Y and libre[i:i1, j1, k:k1].all():
            j1 += 1
        libre[i:i1, j:j1, k:k1] = False
        out.append((i, j, k, i1 - i, j1 - j, k1 - k))
    return np.array(out, np.int64).reshape(-1, 6)


def zonas_libres(occ, origin, voxel, n=16, separacion=3.0, min_holgura=0.8, alto=2.0):
    """
    Puntos del suelo con más espacio libre alrededor (para colocar al jugador, mesas, objetos): [x, z, holgura_m],
    de más a menos holgura, separados al menos `separacion` m. Holgura = distancia al obstáculo más cercano entre el
    suelo y `alto` m.
    """
    from scipy import ndimage
    y0 = int(round(-origin[1] / voxel))
    if not 0 < y0 <= occ.shape[1]:
        return []
    suelo = occ[:, y0 - 1, :]
    libre = suelo & ~occ[:, y0:y0 + int(np.ceil(alto / voxel)), :].any(1)
    holgura = ndimage.distance_transform_edt(libre) * voxel
    out = []
    h = holgura.copy()
    ii, kk = np.indices(h.shape)
    while len(out) < n:
        i, k = np.unravel_index(np.argmax(h), h.shape)
        if h[i, k] < min_holgura:
            break
        x, z = origin[0] + (i + 0.5) * voxel, origin[2] + (k + 0.5) * voxel
        out.append([round(float(x), 3), round(float(z), 3), round(float(holgura[i, k]), 2)])
        h[(ii - i) ** 2 + (kk - k) ** 2 <= (separacion / voxel) ** 2] = 0
    return out


def build(ply: SplatPly, o: Opciones = None, log=print, rotacion=None):
    """
    Manifiesto de colisiones: cajas (cx, cy, cz, sx, sy, sz) en metros y ejes de Unity y zonas libres (x, z, holgura).
    Con `rotacion`, en el marco del mundo (nivelado: "nivelado": true); sin ella, en el de las pirámides.
    """
    o = o or Opciones()
    nivelado = rotacion is not None and not np.allclose(rotacion, [0, 0, 0, 1])
    occ, origin = ocupacion(ply, o, log, rotacion if nivelado else None)
    occ = limpiar(occ, origin, o, log)
    b = cajas(occ)
    centro = origin + (b[:, :3] + b[:, 3:] / 2) * o.voxel
    tam = b[:, 3:] * o.voxel
    zonas = zonas_libres(occ, origin, o.voxel)
    log(f"  {occ.sum():,} vóxeles -> {len(b):,} cajas; {len(zonas)} zonas libres (la mayor, "
        f"{zonas[0][2] if zonas else 0:.1f} m de holgura)")
    return {
        "version": 1,
        "convencion": ("Ejes de Unity (x, -y, z del PLY RDF) con la rotacion_unity aplicada (marco del mundo)"
                       if nivelado else "Ejes de Unity (x, -y, z del PLY RDF), mismo marco que las pirámides")
                      + "; suelo en y = 0",
        "nivelado": bool(nivelado),
        "rotacion_aplicada": [float(v) for v in rotacion] if nivelado else None,
        "voxel_m": o.voxel,
        "opciones": {k: getattr(o, k) for k in o.__dataclass_fields__},
        "min": (origin).round(4).tolist(),
        "max": (origin + np.array(occ.shape) * o.voxel).round(4).tolist(),
        # Planos (JsonUtility de Unity no lee listas anidadas): 6 valores por caja, 3 por zona.
        "cajas": np.round(np.c_[centro, tam], 4).ravel().tolist(),
        "zonas_libres": [v for z in zonas for v in z],
    }, occ, origin


def vista_planta(occ, origin, voxel, path, alturas=(0.0, 2.0)):
    """PNG cenital de lo que choca a la altura de una persona (blanco) sobre el suelo (gris)."""
    from PIL import Image
    y0 = int(round(-origin[1] / voxel))
    a = max(0, y0 + int(alturas[0] / voxel))
    b = min(occ.shape[1], y0 + int(np.ceil(alturas[1] / voxel)))
    img = np.zeros(occ.shape[::2], np.uint8)          # (x, z)
    if 0 < y0 <= occ.shape[1]:
        img[occ[:, y0 - 1, :]] = 90
    img[occ[:, a:b, :].any(1)] = 255
    # Imagen con x a la derecha y z hacia arriba (planta vista desde +y).
    Image.fromarray(img.T[::-1]).resize((img.shape[0] * 3, img.shape[1] * 3), Image.NEAREST).save(path)
