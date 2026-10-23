"""
Limpieza y reducción de una escena 3DGS (plan 15 §7, pasos 2-4).

Cada criterio da una máscara de splats a quitar; se informan por separado (un splat puede caer en varios).
La escena solo se TRASLADA (suelo a y = 0 y centro de la nave en x = z = 0, en los ejes del archivo): una rotación
obligaría a rotar también los SH. La nivelación fina, si hiciera falta, la aplica Unity en el transform (gsplat
evalúa los SH en el espacio del objeto).
"""

from dataclasses import asdict, dataclass

import numpy as np

from . import analisis
from .ply import SplatPly, write_ply


@dataclass
class Opciones:
    margen_zona: float = 0.10       # fracción del tamaño de la zona principal (p0,5-p99,5) que se tolera fuera
    min_vecinos: int = 4            # splats en el cubo 3×3×3 de vóxeles (vóxel = diagonal/200)
    min_opacidad: float = 0.01
    max_sigma_diag: float = 0.02    # σ máxima como fracción de la diagonal de la zona principal
    sh: int = 3                     # grado SH de salida


def masks(core, rep, extra, o: Opciones):
    """Máscaras booleanas (True = quitar) por criterio."""
    lo = np.array(rep["zona_principal"]["min"])
    hi = np.array(rep["zona_principal"]["max"])
    diag = rep["zona_principal"]["diagonal"]
    pos, fin = core["pos"], core["finite"]
    m = {"no_finito": ~fin}
    span = hi - lo
    m["fuera_de_zona"] = fin & ~np.all((pos >= lo - o.margen_zona * span) & (pos <= hi + o.margen_zona * span), 1)
    m["aislado"] = fin & extra["inside"] & (extra["neigh"] < o.min_vecinos)
    m["transparente"] = fin & (core["alpha"] < o.min_opacidad)
    m["gigante"] = fin & (core["sigma"][:, 0] > o.max_sigma_diag * diag)
    return m


def origin_from_floor(rep, core):
    """Origen: centro de la planta (p2-p98 de los splats del suelo) sobre el plano del suelo."""
    s = rep["suelo"]
    if s["eje"] != "-y":
        raise ValueError(f"suelo con arriba {s['eje']}: solo está implementado -y (RDF)")
    fy = s["punto"][1]
    pos, alpha = core["pos"], core["alpha"]
    floor = pos[(alpha > 0.5) & (np.abs(pos[:, 1] - fy) < 0.15)]
    x = np.percentile(floor[:, 0], [2, 98]).mean()
    z = np.percentile(floor[:, 2], [2, 98]).mean()
    return np.array([x, fy, z])


MAX_BYTES = int(1.8 * 2**30)   # gsplat (C#) no lee archivos de más de 2 GB: se parte la salida


def clean(ply: SplatPly, out_path, o: Opciones, core=None, rep=None, extra=None):
    """
    Escribe la escena limpia y devuelve el manifiesto (dict). Si pasa de MAX_BYTES se parte en trozos
    contiguos a lo largo de z (<nombre>_p0.ply, _p1.ply…): el manifiesto lista las partes en "partes".
    """
    if core is None:
        core = analisis.load_core(ply)
    if rep is None or extra is None:
        rep, extra = analisis.inspect(ply, core)
    mk = masks(core, rep, extra, o)
    drop = np.zeros(ply.count, bool)
    for v in mk.values():
        drop |= v
    keep = ~drop
    origin = origin_from_floor(rep, core)

    def translate(cols):
        cols["x"] = cols["x"] - np.float32(origin[0])
        cols["y"] = cols["y"] - np.float32(origin[1])
        cols["z"] = cols["z"] - np.float32(origin[2])
        return cols

    sh_out = min(o.sh, ply.sh_degree)
    bytes_per = 4 * (14 + 3 * {0: 0, 1: 3, 2: 8, 3: 15}[sh_out])
    n_parts = max(1, int(np.ceil(keep.sum() * bytes_per / MAX_BYTES)))
    comment = [f"RealSenseScans tools/splats.py limpiar: origen {origin.round(4).tolist()}"]
    outputs, n = [], 0
    if n_parts == 1:
        n = write_ply(out_path, ply, mask=keep, sh_degree=o.sh, transform=translate, comments=comment)
        outputs.append(out_path)
    else:
        z = core["pos"][:, 2]
        cuts = np.percentile(z[keep], np.linspace(0, 100, n_parts + 1)[1:-1])
        part_id = np.searchsorted(cuts, z)
        stem = out_path[:-4] if out_path.endswith(".ply") else out_path
        for i in range(n_parts):
            p = f"{stem}_p{i}.ply"
            n += write_ply(p, ply, mask=keep & (part_id == i), sh_degree=o.sh, transform=translate,
                           comments=comment + [f"parte {i + 1} de {n_parts} (corte en z)"])
            outputs.append(p)
    return {
        "fuente": ply.path,
        "salida": out_path,
        "partes": outputs,
        "splats_entrada": ply.count,
        "splats_salida": n,
        "quitados": {k: int(v.sum()) for k, v in mk.items()},
        "quitados_total": int(drop.sum()),
        "opciones": asdict(o),
        "sh_salida": sh_out,
        "traslacion_restada": origin.tolist(),
        "convencion": "RDF (x derecha, y abajo, z adelante); suelo en y = 0, arriba = -y. Importar en gsplat como RDF.",
        "inclinacion_suelo_grados": rep["suelo"]["inclinacion_grados"],
    }
