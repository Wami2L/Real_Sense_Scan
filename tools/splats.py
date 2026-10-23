"""
Herramientas offline para escenas 3D Gaussian Splatting (plan 15 §7 y §11).

    python tools/splats.py info Saves/entornos/escena1/original/scene.ply

`info` escribe junto al entorno (Saves/entornos/<nombre>/analisis/): informe.json con lo que hace pesada la
escena (opacidades, tamaños, cuota de relleno, SH, densidad, eje arriba) y vistas previas PNG desde los tres
ejes (superficie y densidad). Los PLY de varios GB se leen por bloques.
"""

import argparse
import json
import os
import sys
import time

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from live3d.splats import SplatPly  # noqa: E402
from live3d.splats import analisis, vista  # noqa: E402


def entorno_dir(ply_path):
    """Saves/entornos/<nombre>/original/x.ply -> Saves/entornos/<nombre>/ (si no, la carpeta del archivo)."""
    d = os.path.dirname(os.path.abspath(ply_path))
    return os.path.dirname(d) if os.path.basename(d) == "original" else d


def cmd_info(a):
    t0 = time.time()
    ply = SplatPly.open(a.ply)
    print(f"{ply.count:,} splats · SH{ply.sh_degree} · {ply.bytes_per_splat} B/splat · "
          f"{ply.count * ply.bytes_per_splat / 2**30:.2f} GB")
    core = analisis.load_core(ply)
    print(f"leído en {time.time() - t0:.0f} s")
    rep, extra = analisis.inspect(ply, core)
    out = a.salida or os.path.join(entorno_dir(a.ply), "analisis")
    os.makedirs(out, exist_ok=True)
    with open(os.path.join(out, "informe.json"), "w", encoding="utf-8") as f:
        json.dump(rep, f, indent=2, ensure_ascii=False)

    if not a.sin_vistas:
        pos, color, alpha = core["pos"], core["color"], core["alpha"]
        bounds = (np.array(rep["zona_principal"]["min"]), np.array(rep["zona_principal"]["max"]))
        aislados = extra["inside"] & (extra["neigh"] < 4)
        for view in "xyz":
            ims = [vista.ortho(pos, color, alpha, view, False, a.res, "cercano", bounds=bounds),
                   vista.ortho(pos, color, alpha, view, True, a.res, "cercano", bounds=bounds),
                   vista.ortho(pos, color, alpha, view, False, a.res, "densidad", bounds=bounds,
                               highlight=aislados)]
            vista.save_contact_sheet(ims, [f"desde +{view}", f"desde -{view}", f"densidad {view} (rojo: aislados)"],
                                     os.path.join(out, f"vista_{view}.png"))
    print_summary(rep)
    print(f"\nInforme y vistas en {out} ({time.time() - t0:.0f} s)")
    return 0


def cmd_camaras(a):
    """Cámaras de prueba a altura de ojos (ejes de Unity tras importar en RDF: (x, -y, z))."""
    ply = SplatPly.open(a.ply)
    core = analisis.load_core(ply)
    rep_path = os.path.join(entorno_dir(a.ply), "analisis", "informe.json")
    with open(rep_path, encoding="utf-8") as f:
        rep = json.load(f)
    suelo = rep["suelo"]
    if suelo["eje"] != "-y":
        raise SystemExit(f"Solo implementado para suelo con arriba -y (RDF); este: {suelo['eje']}")
    pos, alpha = core["pos"], core["alpha"]
    fy = suelo["punto"][1]
    floor = pos[(alpha > 0.5) & (np.abs(pos[:, 1] - fy) < 0.15)]
    x5, x50, x98 = np.percentile(floor[:, 0], [5, 50, 98])
    z2, z50, z98 = np.percentile(floor[:, 2], [2, 50, 98])
    # Capilla lateral: zona opaca a la izquierda de la nave (x < p5 del suelo - 2).
    side = pos[(alpha > 0.5) & (pos[:, 0] < x5 - 2.0) & (np.abs(pos[:, 1] - fy) < 3)]
    eye = -fy + a.ojos                     # Unity: y = -y_datos
    xc = float(x50)

    def v(nombre, p, m):
        return {"nombre": nombre, "pos": [float(c) for c in p], "mirar": [float(c) for c in m]}

    vistas = [
        v("entrada", (xc, eye, z2 + 3), (xc, eye, z98)),
        v("nave", (xc, eye, z50), (xc, eye, z98)),
        v("nave_atras", (xc, eye, z50), (xc, eye, z2)),
        v("boveda", (xc, eye, z50), (xc, eye + 10, z50 + 3)),
        v("altar", (xc, eye, z98 - 6), (xc, eye - 0.3, z98)),
        v("pared_cerca", (x98 - 1.2, eye, z50), (x98, eye, z50)),
    ]
    if len(side) > 1000:
        zc = float(np.median(side[:, 2]))
        vistas.append(v("capilla", (xc, eye, zc), (float(np.percentile(side[:, 0], 5)), eye, zc)))
    out = {"fov_v": a.fov, "vistas": vistas,
           "nota": f"Ejes de Unity (RDF importado: y = -y_datos). Suelo y = {-fy:.3f}, ojos a {a.ojos} m."}
    path = os.path.join(entorno_dir(a.ply), "camaras.json")
    with open(path, "w", encoding="utf-8") as f:
        json.dump(out, f, indent=2, ensure_ascii=False)
    print(f"{len(vistas)} cámaras en {path} (nave x={xc:.2f}, z {z2:.1f}…{z98:.1f}, suelo y={-fy:.2f})")
    return 0


def cmd_limpiar(a):
    from live3d.splats import limpieza
    t0 = time.time()
    ply = SplatPly.open(a.ply)
    core = analisis.load_core(ply)
    rep, extra = analisis.inspect(ply, core)
    o = limpieza.Opciones(margen_zona=a.margen, min_vecinos=a.vecinos, min_opacidad=a.opacidad,
                          max_sigma_diag=a.gigante, sh=a.sh)
    base = entorno_dir(a.ply)
    variants = [int(s) for s in a.variantes_sh.split(",")] if a.variantes_sh else [a.sh]
    for sh in variants:
        o.sh = sh
        name = f"{a.nombre}_sh{sh}"
        out = os.path.join(base, name + ".ply")
        man = limpieza.clean(ply, out, o, core, rep, extra)
        man["licencia"] = leer_licencia(a.ply)
        # Un manifiesto por archivo escrito (el banco de Unity lee el de cada parte para recolocarla).
        for p in man["partes"]:
            with open(p[:-4] + ".json", "w", encoding="utf-8") as f:
                json.dump(man, f, indent=2, ensure_ascii=False)
        size = sum(os.path.getsize(p) for p in man["partes"]) / 2**20
        print(f"{name}: {man['splats_salida']:,} splats ({100 * man['splats_salida'] / ply.count:.1f} %), "
              f"{size:.0f} MB en {len(man['partes'])} archivo(s) · quitados {man['quitados']}")
    print(f"({time.time() - t0:.0f} s)")
    return 0


def cmd_estacion(a):
    """Poda por importancia desde una estación (Quest 2): <entorno>/estacion_<nombre>_<K>k.ply."""
    from live3d.splats import estacion
    from live3d.splats.ply import write_ply
    t0 = time.time()
    ply = SplatPly.open(a.ply)
    man_in_path = a.ply[:-4] + ".json"
    if not os.path.exists(man_in_path):
        raise SystemExit("La entrada debe ser una salida de 'limpiar' (suelo en y = 0, con su .json)")
    with open(man_in_path, encoding="utf-8") as f:
        man_in = json.load(f)
    t = np.array(man_in["traslacion_restada"])
    base = entorno_dir(a.ply)
    with open(os.path.join(base, "camaras.json"), encoding="utf-8") as f:
        cams = json.load(f)
    if a.vista:
        ux, _, uz = next(v["pos"] for v in cams["vistas"] if v["nombre"] == a.vista)
    else:
        ux, uz = (float(c) for c in a.centro.split(","))
    # Unity (x, y, z) -> archivo original RDF (x, -y, z) -> archivo de entrada (restada la traslación).
    center = np.array([ux - t[0], 0.0, uz - t[2]])           # punto del suelo (suelo en y = 0)
    core = analisis.load_core(ply)
    pts = estacion.view_points(center, radius=a.radio)
    imp = estacion.importance(core["pos"], core["alpha"], core["sigma"], core["finite"], pts,
                              max_dist=a.distancia)
    visible = int((imp > 0).sum())
    print(f"Estación '{a.nombre}' en Unity ({ux:.2f}, {uz:.2f}): {len(pts)} puntos de vista, "
          f"{visible:,} splats visibles de {ply.count:,} ({time.time() - t0:.0f} s)")
    for k in [int(float(s) * 1000) for s in a.k.split(",")]:
        keep = estacion.top_k(imp, k)
        name = f"estacion_{a.nombre}_{k // 1000}k"
        out = os.path.join(base, name + ".ply")
        n = write_ply(out, ply, mask=keep, sh_degree=a.sh,
                      comments=[f"estacion {a.nombre}: top {k} por importancia desde {len(pts)} puntos"])
        cubierto = float(imp[keep].sum() / max(imp.sum(), 1e-30))
        man = dict(man_in, salida=out, partes=[out], splats_salida=n, sh_salida=min(a.sh, ply.sh_degree),
                   estacion={"nombre": a.nombre, "centro_unity_xz": [ux, uz], "radio": a.radio, "k": k,
                             "visibles": visible, "importancia_cubierta": cubierto, "fuente": a.ply})
        with open(os.path.join(base, name + ".json"), "w", encoding="utf-8") as f:
            json.dump(man, f, indent=2, ensure_ascii=False)
        print(f"  {name}: {n:,} splats, {os.path.getsize(out) / 2**20:.0f} MB, "
              f"{100 * cubierto:.1f} % de la importancia total")
    # Cámaras de la estación: 6 direcciones horizontales + bóveda, desde el centro y desde el borde.
    floor_unity = -man_in["traslacion_restada"][1]
    eye = floor_unity + 1.6
    vistas = []
    for i, ang in enumerate(range(0, 360, 60)):
        d = np.radians(ang)
        vistas.append({"nombre": f"e{ang:03d}", "pos": [ux, eye, uz],
                       "mirar": [ux + np.sin(d), eye, uz + np.cos(d)]})
    vistas.append({"nombre": "e_boveda", "pos": [ux, eye, uz], "mirar": [ux, eye + 3, uz + 1]})
    for off, nm in (((a.radio, 0), "borde_x"), ((0, -a.radio), "borde_z")):
        vistas.append({"nombre": nm, "pos": [ux + off[0], eye, uz + off[1]], "mirar": [ux + off[0], eye, uz + 10]})
    path = os.path.join(base, f"camaras_estacion_{a.nombre}.json")
    with open(path, "w", encoding="utf-8") as f:
        json.dump({"fov_v": cams.get("fov_v", 90), "vistas": vistas}, f, indent=2)
    print(f"Cámaras de la estación en {path}")
    return 0


def cmd_comparar(a):
    """PSNR/SSIM de cada variante contra la referencia, por vista, y hojas lado a lado."""
    from PIL import Image
    from scipy.ndimage import gaussian_filter

    def load(p):
        return np.asarray(Image.open(p).convert("RGB"), np.float32) / 255.0

    def ssim(x, y):
        x, y = x.mean(2), y.mean(2)
        mx, my = gaussian_filter(x, 1.5), gaussian_filter(y, 1.5)
        vx = gaussian_filter(x * x, 1.5) - mx * mx
        vy = gaussian_filter(y * y, 1.5) - my * my
        cxy = gaussian_filter(x * y, 1.5) - mx * my
        c1, c2 = 0.01 ** 2, 0.03 ** 2
        return float((((2 * mx * my + c1) * (2 * cxy + c2)) / ((mx * mx + my * my + c1) * (vx + vy + c2))).mean())

    ref_dir = os.path.join(a.bench, a.referencia)
    labels = a.variantes.split(",")
    rows, table = [], {}
    views = sorted(f[:-4] for f in os.listdir(ref_dir) if f.endswith(".png"))
    for lab in labels:
        tj = os.path.join(a.bench, lab, "tiempos.json")
        times = {}
        if os.path.exists(tj):
            with open(tj, encoding="utf-8") as f:
                tr = json.load(f)
            times = {v["vista"]: v["ms_mediana"] for v in tr["vistas"]}
        table[lab] = {}
        for v in views:
            p = os.path.join(a.bench, lab, v + ".png")
            if not os.path.exists(p):
                continue
            r, x = load(os.path.join(ref_dir, v + ".png")), load(p)
            mse = float(((r - x) ** 2).mean())
            psnr = 10 * np.log10(1.0 / max(mse, 1e-12))
            table[lab][v] = {"psnr": psnr, "ssim": ssim(r, x), "ms": times.get(v)}
            if a.hojas:
                diff = np.clip(np.abs(r - x) * 4, 0, 1)
                sheet = np.concatenate([r, x, diff], 1)
                h, w = sheet.shape[:2]
                img = Image.fromarray((sheet * 255).astype(np.uint8)).resize((w // 2, h // 2))
                os.makedirs(os.path.join(a.bench, "comparar"), exist_ok=True)
                img.save(os.path.join(a.bench, "comparar", f"{v}__{a.referencia}_vs_{lab}.jpg"), quality=88)
    ref_times = {}
    tj = os.path.join(ref_dir, "tiempos.json")
    if os.path.exists(tj):
        with open(tj, encoding="utf-8") as f:
            ref_times = {v["vista"]: v["ms_mediana"] for v in json.load(f)["vistas"]}
    print(f"Referencia: {a.referencia}")
    print("| vista | ref ms | " + " | ".join(f"{l} ms / PSNR / SSIM" for l in labels) + " |")
    print("|---|---|" + "---|" * len(labels))
    for v in views:
        cells = []
        for l in labels:
            m = table[l].get(v)
            cells.append("—" if m is None else f"{m['ms'] or 0:.1f} / {m['psnr']:.1f} / {m['ssim']:.3f}")
        rt = ref_times.get(v)
        print(f"| {v} | {rt:.1f} | " if rt else f"| {v} | — | ", end="")
        print(" | ".join(cells) + " |")
    out = os.path.join(a.bench, f"comparar_{a.referencia}.json")
    with open(out, "w", encoding="utf-8") as f:
        json.dump({"referencia": a.referencia, "tiempos_ref": ref_times, "variantes": table}, f, indent=2)
    return 0


def leer_licencia(ply_path):
    p = os.path.join(os.path.dirname(os.path.abspath(ply_path)), "license.txt")
    return open(p, encoding="utf-8").read().strip() if os.path.exists(p) else None


def print_summary(r):
    n = r["splats"]
    pc = lambda k: f"{100 * k / n:.1f} %"  # noqa: E731
    z = r["zona_principal"]
    print(f"\nZona principal (p0,5-p99,5): {np.round(z['tamano'], 2).tolist()} · diagonal {z['diagonal']:.2f}")
    print(f"No finitos: {r['no_finitos']} · fuera de la zona (+10 %): {r['fuera_de_zona_10pct']:,} "
          f"({pc(r['fuera_de_zona_10pct'])})")
    o = r["opacidad"]["bajo"]
    print("Opacidad baja: " + " · ".join(f"<{t}: {pc(v)}" for t, v in o.items()))
    s = r["tamano_sigma_max"]["relativo_diagonal"]
    print("Splats grandes (σ máx > % diagonal): " + " · ".join(f">{float(t) * 100:g} %: {v:,}" for t, v in s.items()))
    f = r["relleno"]
    print(f"Relleno (área×alfa) del 1 % más grande: {100 * f['cuota_1pct_mas_grandes']:.0f} % · "
          f"5 %: {100 * f['cuota_5pct']:.0f} % · 10 %: {100 * f['cuota_10pct']:.0f} %")
    if r["sh"]["bajo_umbral"]:
        print(f"SH: {r['sh']['memoria_sh_mb']:.0f} MB · amplitud mediana {r['sh']['amplitud_percentiles']['50']:.3f} · "
              + " · ".join(f"<{t}: {pc(v)}" for t, v in r["sh"]["bajo_umbral"].items()))
    d = r["densidad"]
    print(f"Densidad (vóxel {d['voxel']:.3f}, {d['voxeles_ocupados']:,} ocupados): aislados " +
          " · ".join(f"<{t} vecinos: {pc(v)}" for t, v in d["aislados"].items()))
    print("Eje arriba (candidatos): " + " · ".join(f"{c['arriba']}{c['eje']} (pico {c['pico']:.1f})"
                                                   for c in r["eje_arriba"]))
    s = r.get("suelo")
    if s:
        print(f"Suelo: arriba {s['eje']}, normal {np.round(s['normal_arriba'], 4).tolist()}, "
              f"inclinación {s['inclinacion_grados']:.2f}°, punto {np.round(s['punto'], 3).tolist()} "
              f"({s['inliers']:,} de {s['muestra']:,} splats en el plano)")


def main():
    if hasattr(sys.stdout, "reconfigure"):
        sys.stdout.reconfigure(encoding="utf-8", errors="replace")   # la consola de Windows es cp1252
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = ap.add_subparsers(dest="cmd", required=True)
    p = sub.add_parser("info", help="diagnóstico + vistas previas")
    p.add_argument("ply")
    p.add_argument("--salida", help="carpeta del informe (por defecto <entorno>/analisis)")
    p.add_argument("--res", type=int, default=900, help="lado de las vistas previas en píxeles")
    p.add_argument("--sin-vistas", action="store_true")
    p.set_defaults(func=cmd_info)
    p = sub.add_parser("camaras", help="cámaras de prueba (camaras.json) para el banco de Unity")
    p.add_argument("ply")
    p.add_argument("--ojos", type=float, default=1.6, help="altura de los ojos sobre el suelo (m)")
    p.add_argument("--fov", type=float, default=90.0, help="campo de visión vertical (grados)")
    p.set_defaults(func=cmd_camaras)
    p = sub.add_parser("limpiar", help="quitar flotantes/transparentes/gigantes, recortar SH y trasladar")
    p.add_argument("ply")
    p.add_argument("--nombre", default="limpio", help="prefijo del archivo de salida (<entorno>/<nombre>_shN.ply)")
    p.add_argument("--sh", type=int, default=3, help="grado SH de salida")
    p.add_argument("--variantes-sh", help="varias salidas a la vez, p. ej. 3,1,0")
    p.add_argument("--opacidad", type=float, default=0.01, help="quitar alfa menor que esto")
    p.add_argument("--vecinos", type=int, default=4, help="quitar splats con menos vecinos (flotantes)")
    p.add_argument("--gigante", type=float, default=0.02, help="quitar σ máx mayor que esta fracción de la diagonal")
    p.add_argument("--margen", type=float, default=0.10, help="margen fuera de la zona principal")
    p.set_defaults(func=cmd_limpiar)
    p = sub.add_parser("estacion", help="poda por importancia desde una estación (Quest 2)")
    p.add_argument("ply", help="salida de 'limpiar' (con su .json)")
    p.add_argument("--nombre", required=True)
    g = p.add_mutually_exclusive_group(required=True)
    g.add_argument("--vista", help="centro = posición de esta vista de camaras.json")
    g.add_argument("--centro", help="centro 'x,z' en coordenadas de Unity")
    p.add_argument("--k", default="100,150,300", help="miles de splats a conservar (varios separados por comas)")
    p.add_argument("--radio", type=float, default=0.75, help="radio de la zona de movimiento (m)")
    p.add_argument("--distancia", type=float, help="ignorar splats más lejos que esto (m)")
    p.add_argument("--sh", type=int, default=3)
    p.set_defaults(func=cmd_estacion)
    p = sub.add_parser("comparar", help="PSNR/SSIM y tiempos de variantes del banco de Unity")
    p.add_argument("bench", help="carpeta del banco (<entorno>/bench)")
    p.add_argument("--referencia", required=True)
    p.add_argument("--variantes", required=True, help="etiquetas separadas por comas")
    p.add_argument("--hojas", action="store_true", help="guardar hojas referencia | variante | diferencia×4")
    p.set_defaults(func=cmd_comparar)
    a = ap.parse_args()
    return a.func(a)


if __name__ == "__main__":
    sys.exit(main())
