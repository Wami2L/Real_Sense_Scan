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
    name = "camaras.json"
    if a.recorrido:
        # Recorrido por toda la escena: puntos del suelo separados >= 3 m, orientación al azar, ojos a a.ojos.
        rng = np.random.default_rng(a.semilla)
        cand = floor[rng.permutation(len(floor))[:200000]]
        picked = []
        for p in cand:
            if all(np.hypot(p[0] - q[0], p[2] - q[2]) >= 3.0 for q in picked):
                picked.append(p)
            if len(picked) >= a.recorrido:
                break
        vistas = []
        for i, p in enumerate(picked):
            yaw = rng.uniform(0, 2 * np.pi)
            pitch = np.radians(rng.uniform(-10, 25))
            d = (np.cos(pitch) * np.sin(yaw), np.sin(pitch), np.cos(pitch) * np.cos(yaw))
            pos = (float(p[0]), eye, float(p[2]))
            vistas.append(v(f"r{i:02d}", pos, (pos[0] + d[0], pos[1] + d[1], pos[2] + d[2])))
        name = "camaras_recorrido.json"
    out = {"fov_v": a.fov, "vistas": vistas,
           "nota": f"Ejes de Unity (RDF importado: y = -y_datos). Suelo y = {-fy:.3f}, ojos a {a.ojos} m."}
    path = os.path.join(entorno_dir(a.ply), name)
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


def station_setup(a):
    """
    Entrada de 'estacion' y 'lod': salida de 'limpiar' (suelo en y = 0, con su .json) y el centro de la estación
    (vista de camaras.json o 'x,z' de Unity). Devuelve (ply, manifiesto, cámaras, (ux, uz), centro en el suelo
    en coordenadas del archivo).
    """
    ply = SplatPly.open(a.ply)
    man_in_path = a.ply[:-4] + ".json"
    if not os.path.exists(man_in_path):
        raise SystemExit("La entrada debe ser una salida de 'limpiar' (suelo en y = 0, con su .json)")
    with open(man_in_path, encoding="utf-8") as f:
        man_in = json.load(f)
    t = np.array(man_in["traslacion_restada"])
    with open(os.path.join(entorno_dir(a.ply), "camaras.json"), encoding="utf-8") as f:
        cams = json.load(f)
    if a.vista:
        ux, _, uz = next(v["pos"] for v in cams["vistas"] if v["nombre"] == a.vista)
    else:
        ux, uz = (float(c) for c in a.centro.split(","))
    # Unity (x, y, z) -> archivo original RDF (x, -y, z) -> archivo de entrada (restada la traslación).
    center = np.array([ux - t[0], 0.0, uz - t[2]])
    return ply, man_in, cams, (ux, uz), center


def cmd_estacion(a):
    """Poda por importancia desde una estación (Quest 2): <entorno>/estacion_<nombre>_<K>k.ply."""
    from live3d.splats import estacion
    from live3d.splats.ply import write_ply
    t0 = time.time()
    ply, man_in, cams, (ux, uz), center = station_setup(a)
    base = entorno_dir(a.ply)
    core = analisis.load_core(ply)
    pts = estacion.view_points(center, radius=a.radio)
    imp = estacion.importance(core["pos"], core["alpha"], core["sigma"], core["finite"], pts,
                              max_dist=a.distancia, normal=core["normal"])
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


def cmd_lod(a):
    """LOD por fusión desde una estación (plan 15 §12.4): <entorno>/lod_<nombre>_<θ>.ply por cada ángulo."""
    import gc
    from live3d.splats import estacion, lod
    from live3d.splats.ply import write_columns
    t0 = time.time()
    ply, man_in, cams, (ux, uz), center = station_setup(a)
    base = entorno_dir(a.ply)
    up = np.array([0.0, -1.0, 0.0])                         # RDF: arriba = -y; suelo en y = 0
    eye = center + a.ojos * up
    visible = None
    if not a.sin_oclusion:
        core = analisis.load_core(ply)
        pts = estacion.view_points(center, radius=a.radio)
        imp = estacion.importance(core["pos"], core["alpha"], core["sigma"], core["finite"], pts,
                                  normal=core["normal"])
        visible = (imp > 0) & core["finite"]
        del core, imp
        gc.collect()
        print(f"Oclusión: {visible.sum():,} splats visibles de {ply.count:,} ({time.time() - t0:.0f} s)")
    cols = lod.read_columns(ply, visible, sh_degree=a.sh)
    pos = np.stack([cols["x"], cols["y"], cols["z"]], 1)
    for ang in [float(s) for s in a.angulos.split(",")]:
        theta = np.radians(ang)
        key, size = lod.cell_keys(pos, eye, theta, r_station=a.radio, radial=a.radial)
        out, st = lod.merge(cols, key, size, inflate=a.inflar)
        name = (f"lod_{a.nombre}_{ang:g}" + (f"_r{a.radial:g}" if a.radial != 1 else "")
                + (f"_inf{a.inflar:g}" if a.inflar > 0 else "")).replace(".", "p")
        path = os.path.join(base, name + ".ply")
        n = write_columns(path, out, comments=[
            f"lod {a.nombre}: fusion desde la estacion, theta {ang} grados, radio {a.radio} m, inflado {a.inflar}"])
        man = dict(man_in, salida=path, partes=[path], splats_salida=n, sh_salida=min(a.sh, ply.sh_degree),
                   lod={"nombre": a.nombre, "centro_unity_xz": [ux, uz], "radio": a.radio, "theta_grados": ang,
                        "inflado": a.inflar, "radial": a.radial, "oclusion": not a.sin_oclusion,
                        "fuente": a.ply, **st})
        with open(os.path.join(base, name + ".json"), "w", encoding="utf-8") as f:
            json.dump(man, f, indent=2, ensure_ascii=False)
        print(f"  {name}: {n:,} splats ({st['intactos']:,} intactos + {st['fusionadas']:,} fusionadas de "
              f"{st['splats_fusionados']:,}), {os.path.getsize(path) / 2**20:.0f} MB ({time.time() - t0:.0f} s)")
    return 0


def cmd_piramide(a):
    """Pirámide LOD por trozos de la escena completa (plan 15 §13): <entorno>/<nombre>.ply + .json."""
    from live3d.splats import piramide
    t0 = time.time()
    ply = SplatPly.open(a.ply)
    man_in_path = a.ply[:-4] + ".json"
    man_in = {}
    if os.path.exists(man_in_path):
        with open(man_in_path, encoding="utf-8") as f:
            man_in = json.load(f)
    out = os.path.join(entorno_dir(a.ply), a.nombre + ".ply")
    voxels = tuple(float(v) for v in a.voxeles.split(","))
    man = piramide.build(ply, out, trozo=a.trozo, voxels=voxels, sh_degree=a.sh, nivel_minimo=a.nivel_minimo)
    man["traslacion_restada"] = man_in.get("traslacion_restada")
    man["licencia"] = man_in.get("licencia")
    with open(out[:-4] + ".json", "w", encoding="utf-8") as f:
        json.dump(man, f, ensure_ascii=False)
    print(f"{out}: {os.path.getsize(out) / 2**20:.0f} MB ({time.time() - t0:.0f} s)")
    return 0


def cmd_preparar(a):
    """
    Cadena completa para una escena nueva (plan 15 §13): análisis + vistas previas, limpieza (SH1, trasladada, sin
    partir), pirámide LOD para PC y para Quest 2 y, con --unity, exportación al proyecto VR ScanWorld
    (Assets/ScanWorld/Entornos/<nombre>/, PLY con .meta en RDF). En Unity: ScanWorld › Crear escenas de entornos.
    """
    from live3d.splats import limpieza, piramide
    t0 = time.time()
    ply = SplatPly.open(a.ply)
    base = entorno_dir(a.ply)
    print(f"[1/4] Análisis: {ply.count:,} splats · SH{ply.sh_degree} · "
          f"{ply.count * ply.bytes_per_splat / 2**30:.2f} GB")
    core = analisis.load_core(ply)
    rep, extra = analisis.inspect(ply, core)
    out_info = os.path.join(base, "analisis")
    os.makedirs(out_info, exist_ok=True)
    with open(os.path.join(out_info, "informe.json"), "w", encoding="utf-8") as f:
        json.dump(rep, f, indent=2, ensure_ascii=False)
    print_summary(rep)
    escala, inicio = a.escala or 1.0, None
    if a.colmap:
        # COLMAP no tiene gravedad ni escala: el suelo se busca guiado por las cámaras del vídeo y la escala sale de
        # la altura a la que iba el móvil (plan 15 §14).
        from live3d.splats import colmap
        images = a.colmap if a.colmap.endswith(".txt") else os.path.join(a.colmap, "images.txt")
        centers, ups, fwds = colmap.read_cameras(images)
        suelo = colmap.floor_from_cameras(core["pos"], core["alpha"], core["finite"], centers, ups)
        rep["suelo"] = suelo
        if not a.escala:
            escala = a.altura_camara / suelo["altura_camaras"]
        print(f"  Cámaras COLMAP: {len(centers)} · suelo con normal {np.round(suelo['normal_arriba'], 3).tolist()} "
              f"({suelo['angulo_con_camaras']:.1f}° del arriba medio de las cámaras, {suelo['inliers']:,} splats) · "
              f"cámaras a {suelo['altura_camaras']:.2f} unidades del suelo → escala {escala:.4f} m/unidad "
              f"(supuesto: móvil a {a.altura_camara} m)")
        mid = int(np.argmin(np.linalg.norm(centers - np.median(centers, 0), axis=1)))
        inicio = (centers[mid], fwds[mid])
        with open(os.path.join(out_info, "informe.json"), "w", encoding="utf-8") as f:
            json.dump(rep, f, indent=2, ensure_ascii=False)
    elif rep["suelo"]["inclinacion_grados"] > 10:
        print(f"  ! El suelo está inclinado {rep['suelo']['inclinacion_grados']:.1f}° respecto al eje "
              f"{rep['suelo']['eje']}: revisa las vistas de {out_info} (la nivelación se aplica en Unity)")

    print("[2/4] Limpieza")
    o = limpieza.Opciones(sh=a.sh, max_sigma_diag=a.gigante, agujas_ratio=a.agujas, min_vecinos=a.vecinos)
    clean_path = os.path.join(base, f"limpio_sh{a.sh}.ply")
    man = limpieza.clean(ply, clean_path, o, core, rep, extra, max_bytes=float("inf"), escala=escala)
    man["licencia"] = leer_licencia(a.ply)
    with open(clean_path[:-4] + ".json", "w", encoding="utf-8") as f:
        json.dump(man, f, indent=2, ensure_ascii=False)
    print(f"  {man['splats_salida']:,} splats ({100 * man['splats_salida'] / ply.count:.1f} %) · quitados {man['quitados']}")
    del core, extra
    camara = camara_inicio(man, inicio, a.altura_camara)
    print(f"  Escala aplicada {escala:.4f} · cámara inicial en Unity {np.round(camara['pos'], 2).tolist()}")

    print("[3/4] Pirámides LOD")
    clean = SplatPly.open(clean_path)
    # Trozos y niveles a la medida de la escena: en una escena pequeña, trozos de 3 m dejarían muy pocos y el salto
    # entre el nivel 0 y el de 4 cm sería demasiado grande para repartir el presupuesto.
    diag_m = rep["zona_principal"]["diagonal"] * escala
    trozo = a.trozo or float(np.clip(diag_m / 20, 1.0, 3.0))
    voxeles = (0.04, 0.08, 0.16, 0.32, 0.64, 1.28)
    if clean.count < 3_000_000:
        voxeles = (0.02,) + voxeles
    print(f"  escena de {diag_m:.1f} m de diagonal: trozos de {trozo:.2f} m, vóxeles {list(voxeles)}")
    outputs = []
    for name, nivel_min in (("piramide", 0), ("piramide_quest", a.nivel_quest)):
        path = os.path.join(base, name + ".ply")
        pm = piramide.build(clean, path, trozo=trozo, voxels=voxeles, sh_degree=a.sh, nivel_minimo=nivel_min)
        # gsplat no lee PLY de más de 2 GB: subir el nivel mínimo hasta caber.
        while os.path.getsize(path) > 1.9 * 2**30 and pm["nivel_minimo"] < len(pm["niveles_voxel_m"]) - 1:
            print(f"  ! {name}: {os.path.getsize(path) / 2**30:.2f} GB > 1,9 GB; nivel mínimo {pm['nivel_minimo'] + 1}")
            pm = piramide.build(clean, path, trozo=trozo, voxels=voxeles, sh_degree=a.sh,
                                nivel_minimo=pm["nivel_minimo"] + 1)
        pm.update(traslacion_restada=man["traslacion_restada"], rotacion_unity=man["rotacion_unity"],
                  escala_aplicada=escala, camara_inicio=camara, licencia=man["licencia"], nombre=a.nombre,
                  coordenadas="RDF")
        with open(path[:-4] + ".json", "w", encoding="utf-8") as f:
            json.dump(pm, f, ensure_ascii=False)
        outputs.append(path)
        print(f"  {name}: {pm['splats_total']:,} splats, {os.path.getsize(path) / 2**20:.0f} MB")

    # Colisiones para la escena VR (plan 15 G5): mismo marco que las pirámides (metros, suelo en y = 0).
    from live3d.splats import colision
    cman, occ, corigin = colision.build(clean, colision.Opciones())
    col_path = os.path.join(base, "colision.json")
    with open(col_path, "w", encoding="utf-8") as f:
        json.dump(cman, f, ensure_ascii=False)
    colision.vista_planta(occ, corigin, colision.Opciones().voxel, os.path.join(base, "colision_planta.png"))
    print(f"  colisiones: {len(cman['cajas']) // 6:,} cajas, {len(cman.get('zonas_libres', [])) // 3} zonas libres")

    if a.unity:
        print("[4/4] Exportación a Unity")
        exportar_unity(a.unity, a.nombre, outputs, leer_licencia(a.ply))
        exportar_texto(a.unity, a.nombre, col_path)
    else:
        print("[4/4] Sin --unity: no se exporta")
    print(f"Listo en {time.time() - t0:.0f} s")
    return 0


def _rotate(q, v):
    """Rota v con el cuaternión q = (x, y, z, w)."""
    x, y, z, w = q
    u = np.array([x, y, z])
    v = np.asarray(v, np.float64)
    return v + 2 * np.cross(u, np.cross(u, v) + w * v)


def camara_inicio(man, inicio, altura):
    """
    Pose inicial de la cámara en ejes de Unity (con la rotación de nivelación ya aplicada): la cámara COLMAP central
    del recorrido si la hay, si no el centro del suelo a `altura` m mirando a +z.
    """
    q = man["rotacion_unity"]
    if inicio is None:
        return {"pos": [0.0, float(altura), 0.0], "adelante": [0.0, 0.0, 1.0]}
    c, f = inicio
    t = np.asarray(man["traslacion_restada"])
    s = man.get("escala_aplicada", 1.0)
    rdf = np.array([1.0, -1.0, 1.0])                 # archivo -> Unity al importar en RDF
    pos = _rotate(q, (c - t) * s * rdf)
    fwd = _rotate(q, f * rdf)
    fwd[1] = 0.0
    fwd /= max(np.linalg.norm(fwd), 1e-9)
    return {"pos": [float(v) for v in pos], "adelante": [float(v) for v in fwd]}


_PLY_META = """fileFormatVersion: 2
guid: {guid}
ScriptedImporter:
  internalIDToNameTable: []
  externalObjects: {{}}
  serializedVersion: 2
  userData:
  assetBundleName:
  assetBundleVariant:
  script: {{fileID: 11500000, guid: 7468ea6559404cbc8f3e83b0b1a00683, type: 3}}
  Compression: 1
  OpacityPruneThreshold: 0
  SourceCoordinates: 6
"""
_TEXT_META = """fileFormatVersion: 2
guid: {guid}
TextScriptImporter:
  externalObjects: {{}}
  userData:
  assetBundleName:
  assetBundleVariant:
"""


def exportar_unity(unity_project, nombre, plys, licencia):
    """
    Copia las pirámides (PLY + manifiesto) a <proyecto>/Assets/ScanWorld/Entornos/<nombre>/ con sus .meta ya escritos
    (importador de gsplat en RDF: así se importan bien a la primera). Esa carpeta está ignorada en git.
    """
    import shutil
    import uuid
    dst = os.path.join(unity_project, "Assets", "ScanWorld", "Entornos", nombre)
    os.makedirs(dst, exist_ok=True)
    for ply_path in plys:
        for src, tpl in ((ply_path, _PLY_META), (ply_path[:-4] + ".json", _TEXT_META)):
            target = os.path.join(dst, os.path.basename(src))
            shutil.copyfile(src, target)
            if not os.path.exists(target + ".meta"):
                with open(target + ".meta", "w", encoding="utf-8", newline="\n") as f:
                    f.write(tpl.format(guid=uuid.uuid4().hex))
            print(f"  {target}")
    if licencia:
        target = os.path.join(dst, "LICENCIA.txt")
        with open(target, "w", encoding="utf-8") as f:
            f.write(licencia + "\n")
        if not os.path.exists(target + ".meta"):
            with open(target + ".meta", "w", encoding="utf-8", newline="\n") as f:
                f.write(_TEXT_META.format(guid=uuid.uuid4().hex))


def exportar_texto(unity_project, nombre, path):
    """Copia un JSON (manifiesto, colisiones) a Assets/ScanWorld/Entornos/<nombre>/ con su .meta de texto."""
    import shutil
    import uuid
    dst = os.path.join(unity_project, "Assets", "ScanWorld", "Entornos", nombre)
    os.makedirs(dst, exist_ok=True)
    target = os.path.join(dst, os.path.basename(path))
    shutil.copyfile(path, target)
    if not os.path.exists(target + ".meta"):
        with open(target + ".meta", "w", encoding="utf-8", newline="\n") as f:
            f.write(_TEXT_META.format(guid=uuid.uuid4().hex))
    print(f"  {target}")


def cmd_colision(a):
    """Colisiones del entorno para Unity (plan 15 G5): <entorno>/colision.json + colision_planta.png."""
    from live3d.splats import colision
    t0 = time.time()
    ply = SplatPly.open(a.ply)
    o = colision.Opciones(voxel=a.voxel, min_alfa=a.alfa, min_splats=a.min_splats)
    man, occ, origin = colision.build(ply, o)
    base = entorno_dir(a.ply)
    out = os.path.join(base, "colision.json")
    with open(out, "w", encoding="utf-8") as f:
        json.dump(man, f, ensure_ascii=False)
    colision.vista_planta(occ, origin, o.voxel, os.path.join(base, "colision_planta.png"))
    print(f"{out}: {len(man['cajas']) // 6:,} cajas ({time.time() - t0:.0f} s)")
    if a.unity:
        exportar_texto(a.unity, a.nombre, out)
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
            if a.centro < 1:
                # Solo la zona central (donde mira el ojo): en VR la periferia la emborronan las lentes.
                h, w = r.shape[:2]
                ch, cw = int(h * a.centro / 2), int(w * a.centro / 2)
                r, x = r[h // 2 - ch:h // 2 + ch, w // 2 - cw:w // 2 + cw], x[h // 2 - ch:h // 2 + ch, w // 2 - cw:w // 2 + cw]
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
    out = os.path.join(a.bench, f"comparar_{a.referencia}" + (f"_centro{a.centro:g}" if a.centro < 1 else "") + ".json")
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
    p.add_argument("--recorrido", type=int, default=0,
                   help="en lugar de las vistas fijas, N puntos repartidos por todo el suelo (camaras_recorrido.json)")
    p.add_argument("--semilla", type=int, default=0)
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
    p = sub.add_parser("lod", help="LOD por fusión de splats desde una estación (Quest 2)")
    p.add_argument("ply", help="salida de 'limpiar' (con su .json)")
    p.add_argument("--nombre", required=True)
    g = p.add_mutually_exclusive_group(required=True)
    g.add_argument("--vista", help="centro = posición de esta vista de camaras.json")
    g.add_argument("--centro", help="centro 'x,z' en coordenadas de Unity")
    p.add_argument("--angulos", default="0.3,0.5,0.8", help="lado angular de las celdas en grados (varios)")
    p.add_argument("--radio", type=float, default=0.75, help="radio de la zona de movimiento (m)")
    p.add_argument("--ojos", type=float, default=1.6, help="altura del centro de la rejilla sobre el suelo (m)")
    p.add_argument("--inflar", type=float, default=0.0, help="ensancha cada fusionada (fracción del lado de celda)")
    p.add_argument("--radial", type=float, default=1.0, help="largo de las celdas a lo largo del rayo (× lado)")
    p.add_argument("--sin-oclusion", action="store_true", help="no descartar lo que no se ve desde la estación")
    p.add_argument("--sh", type=int, default=1)
    p.set_defaults(func=cmd_lod)
    p = sub.add_parser("piramide", help="pirámide LOD por trozos de toda la escena (LOD dinámico en Unity)")
    p.add_argument("ply", help="escena limpia (salida de 'limpiar', con su .json)")
    p.add_argument("--nombre", default="piramide")
    p.add_argument("--trozo", type=float, default=3.0, help="lado de los trozos (m)")
    p.add_argument("--voxeles", default="0.04,0.08,0.16,0.32,0.64,1.28", help="vóxel de cada nivel fusionado (m)")
    p.add_argument("--nivel-minimo", type=int, default=0, help="omitir los niveles más finos (Quest: 1)")
    p.add_argument("--sh", type=int, default=1)
    p.set_defaults(func=cmd_piramide)
    p = sub.add_parser("preparar", help="cadena completa para una escena nueva (limpieza + pirámides + Unity)")
    p.add_argument("ply", help="escena 3DGS original (.ply)")
    p.add_argument("--nombre", required=True, help="nombre del entorno (carpeta en Unity y título de la escena)")
    p.add_argument("--unity", help="ruta del proyecto Unity (VR ScanWorld) para exportar las pirámides")
    p.add_argument("--trozo", type=float, help="lado de los trozos (m); por defecto diagonal/20 entre 1 y 3 m")
    p.add_argument("--sh", type=int, default=1)
    p.add_argument("--nivel-quest", type=int, default=1, help="nivel mínimo de la pirámide de Quest 2")
    p.add_argument("--agujas", type=float, default=10.0,
                   help="quitar agujas (σ1/σ2 mayor que esto, > 3 cm y alfa < 0,35); 0 = no filtrar")
    p.add_argument("--vecinos", type=int, default=16,
                   help="quitar splats con menos vecinos (niebla y flotantes de zonas poco vistas)")
    p.add_argument("--colmap", help="carpeta sparse/0 (o images.txt) del modelo COLMAP: suelo, escala y cámara inicial")
    p.add_argument("--altura-camara", type=float, default=1.5, help="altura del móvil al grabar (m), para la escala")
    p.add_argument("--escala", type=float, help="metros por unidad del archivo (si se conoce; anula la estimada)")
    p.add_argument("--gigante", type=float, default=0.05, help="quitar σ máx mayor que esta fracción de la diagonal")
    p.set_defaults(func=cmd_preparar)
    p = sub.add_parser("colision", help="cajas de colisión del entorno para Unity (suelo, paredes, bancos)")
    p.add_argument("ply", help="escena limpia (salida de 'limpiar'): mismo marco que las pirámides")
    p.add_argument("--voxel", type=float, default=0.2, help="lado de los vóxeles (m)")
    p.add_argument("--alfa", type=float, default=0.3, help="opacidad mínima de los splats que cuentan")
    p.add_argument("--min-splats", type=int, default=5, help="splats por vóxel para ocuparlo")
    p.add_argument("--unity", help="ruta del proyecto Unity (VR ScanWorld) para copiar colision.json")
    p.add_argument("--nombre", default="Iglesia", help="carpeta del entorno en Unity")
    p.set_defaults(func=cmd_colision)
    p = sub.add_parser("comparar", help="PSNR/SSIM y tiempos de variantes del banco de Unity")
    p.add_argument("bench", help="carpeta del banco (<entorno>/bench)")
    p.add_argument("--referencia", required=True)
    p.add_argument("--variantes", required=True, help="etiquetas separadas por comas")
    p.add_argument("--hojas", action="store_true", help="guardar hojas referencia | variante | diferencia×4")
    p.add_argument("--centro", type=float, default=1.0, help="medir solo la zona central (fracción de ancho y alto)")
    p.set_defaults(func=cmd_comparar)
    a = ap.parse_args()
    return a.func(a)


if __name__ == "__main__":
    sys.exit(main())
