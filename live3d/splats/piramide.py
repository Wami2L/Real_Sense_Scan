"""
Pirámide LOD por trozos de una escena 3DGS completa (plan 15 §13): el formato que usa el selector dinámico de Unity
(ScanWorld.SplatLod), como el LOD por octree de PlayCanvas/SuperSplat.

La escena se trocea en cubos de `trozo` metros; para cada cubo se guarda el nivel 0 (los splats originales) y niveles
fusionados en vóxeles cada vez mayores (por defecto 4, 8, 16, 32, 64 y 128 cm) con el mismo ajuste de momentos que
lod.merge. Todo va a UN solo PLY ordenado por (trozo, nivel) y un manifiesto con la caja de cada trozo (en ejes de
Unity) y el rango [inicio, cuenta] de cada nivel: en Unity basta un GsplatRenderer y, cada vez que cambia la
selección, la lista de rangos elegidos (GsplatRenderer.SetSelection).
"""

import numpy as np

from . import lod
from .ply import SplatPly, write_columns


def _voxel_keys(pos, size):
    ijk = np.floor(pos / size).astype(np.int64)
    ijk -= ijk.min(0)
    dims = ijk.max(0) + 1
    return (ijk[:, 0] * dims[1] + ijk[:, 1]) * dims[2] + ijk[:, 2]


def build(ply: SplatPly, out_path, trozo=3.0, voxels=(0.04, 0.08, 0.16, 0.32, 0.64, 1.28), sh_degree=1,
          min_splats_trozo=16, nivel_minimo=0, log=print):
    """
    Escribe la pirámide en out_path (.ply) y devuelve el manifiesto (dict). nivel_minimo > 0 omite los niveles más
    finos (p. ej. 1 para una versión de Quest sin los splats originales: el nivel 0 queda igual al 1).
    """
    cols = lod.read_columns(ply, sh_degree=sh_degree)
    pos = np.stack([cols["x"], cols["y"], cols["z"]], 1).astype(np.float64)
    origin = np.floor(pos.min(0) / trozo) * trozo
    cij = np.floor((pos - origin) / trozo).astype(np.int64)
    dims = cij.max(0) + 1
    ckey = (cij[:, 0] * dims[1] + cij[:, 1]) * dims[2] + cij[:, 2]
    order = np.argsort(ckey, kind="stable")
    ck = ckey[order]
    starts = np.r_[0, np.nonzero(np.diff(ck))[0] + 1]
    ends = np.r_[starts[1:], len(ck)]
    sigma_max = np.exp(np.max(np.stack([cols[f"scale_{i}"] for i in range(3)], 1), axis=1))

    props = list(cols)
    out = {p: [] for p in props}
    trozos = []
    cursor = 0
    tamanos = [0.0] + list(voxels)                       # tamaño de vóxel nominal por nivel (0 = original)
    n_niveles = len(tamanos)
    descartados = 0
    for s, e in zip(starts, ends):
        idx = order[s:e]
        if len(idx) < min_splats_trozo:
            descartados += len(idx)
            continue
        chunk = {p: cols[p][idx] for p in props}
        p_chunk = pos[idx]
        # Caja del trozo con el tamaño de sus splats (las fusionadas no salen de aquí: son medias ponderadas).
        r = 2 * sigma_max[idx][:, None]
        lo = (p_chunk - r).min(0)
        hi = (p_chunk + r).max(0)
        inicio, cuenta = [], []
        prev = chunk
        for k, v in enumerate(tamanos):
            if k == 0:
                level = chunk
            else:
                key = _voxel_keys(np.stack([prev["x"], prev["y"], prev["z"]], 1).astype(np.float64), v)
                if len(np.unique(key)) == len(key):
                    level = prev                        # nada que fusionar a este tamaño
                else:
                    level, _ = lod.merge(prev, key, np.full(len(key), v))
            prev = level
            if k < nivel_minimo:
                inicio.append(-1)
                cuenta.append(0)
                continue
            n = len(level["x"])
            for p in props:
                out[p].append(level[p].astype(np.float32))
            inicio.append(cursor)
            cuenta.append(n)
            cursor += n
        # Niveles omitidos por nivel_minimo apuntan al primero guardado.
        first = next(k for k in range(n_niveles) if cuenta[k] > 0)
        for k in range(first):
            inicio[k], cuenta[k] = inicio[first], cuenta[first]
        # RDF del archivo -> ejes de Unity (x, -y, z): la y cambia de signo y se intercambian min y max.
        trozos.append({"min": [float(lo[0]), float(-hi[1]), float(lo[2])],
                       "max": [float(hi[0]), float(-lo[1]), float(hi[2])],
                       "inicio": [int(x) for x in inicio], "cuenta": [int(x) for x in cuenta]})

    merged = {p: np.concatenate(out[p]) for p in props}
    n_total = write_columns(out_path, merged, comments=[
        f"piramide LOD: trozos de {trozo} m, voxeles {list(voxels)}, nivel minimo {nivel_minimo}"])
    por_nivel = [int(sum(t["cuenta"][k] for t in trozos)) for k in range(n_niveles)]
    log(f"{len(trozos)} trozos · {n_total:,} splats en total · por nivel {por_nivel} · "
        f"{descartados} splats en trozos casi vacíos descartados")
    return {
        "version": 1,
        "fuente": ply.path,
        "ply": out_path,
        "convencion": "RDF en el archivo (importar en gsplat como RDF); cajas de los trozos ya en ejes de Unity",
        "sh": min(sh_degree, ply.sh_degree),
        "trozo_m": trozo,
        "niveles_voxel_m": tamanos,
        "nivel_minimo": nivel_minimo,
        "splats_total": int(n_total),
        "splats_por_nivel": por_nivel,
        "trozos": trozos,
    }
