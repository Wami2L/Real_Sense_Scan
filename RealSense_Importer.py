"""
Importador de los guardados de RealSenseScans (Saves/AAAA-MM-DD/<tipo>_HHMMSS/).

  scan_       nube_puntos.ply, nube_calor_profundidad.ply, captura_color.jpg, metadata_camara.json
              (malla_texturizada.obj si se generó con generar_malla_offline.py)
  dual_scan_  nube_fusionada.ply (marco Cam 1), nube_cam1.ply, nube_cam2.ply (marco Cam 2),
              diagnostico_solape.ply, malla_fusionada.obj, camN_color.jpg, metadata_dual.json
  multi_      nube_fusionada(_calor).ply, malla_fusionada.obj, resumen_multicaptura.json (poses),
              vista_XX/{nube_puntos.ply, captura_color.jpg, metadata_vista.json}

Los archivos están en coordenadas RealSense (x derecha, y abajo, z adelante, metros). Cada
escaneo cuelga de un objeto raíz que los lleva a Blender (Z arriba): (x, y, z) -> (x, z, -y).
Debajo, la Cam 2 (dual) y cada vista (multi) tienen su propio marco con la pose guardada.

Solo usa bpy y la biblioteca estándar: no hace falta instalar nada dentro de Blender.
"""

bl_info = {
    "name": "RealSense Scan Importer",
    "author": "RealSenseScans",
    "version": (2, 0, 0),
    "blender": (4, 0, 0),
    "location": "View3D > Sidebar > RealSense",
    "description": "Importa escaneos de Saves/ (simple, dual y multicaptura) con ejes, color y cámaras.",
    "category": "Import-Export",
}

import json
import math
import os
import shutil
import subprocess

import bpy
from mathutils import Matrix

RS_TO_BLENDER = Matrix.Rotation(-math.pi / 2, 4, 'X')   # (x, y, z)_RS -> (x, z, -y)
RS_CAM_TO_BLENDER_CAM = Matrix.Rotation(math.pi, 4, 'X')  # cámara RS mira a +Z con Y abajo; la de Blender a -Z con Y arriba
POINTS_GROUP = "RS_Puntos"


# --------------------------------------------------------------------------- #
# Lectura de la carpeta
# --------------------------------------------------------------------------- #
def _read_json(path):
    try:
        with open(path, encoding="utf-8") as f:
            return json.load(f)
    except (OSError, ValueError):
        return None


def _existing(folder, *names):
    return [n for n in names if os.path.isfile(os.path.join(folder, n))]


def detect_scan(folder):
    """Tipo de guardado y metadatos. type: 'dual' | 'multi' | 'simple' | None."""
    folder = os.path.normpath(folder) if folder else ""
    info = {"type": None, "folder": folder, "id": os.path.basename(folder) or "Scan", "meta": None}
    if not folder or not os.path.isdir(folder):
        return info
    if _existing(folder, "metadata_dual.json", "nube_cam1.ply"):
        info["type"] = "dual"
        info["meta"] = _read_json(os.path.join(folder, "metadata_dual.json")) or {}
    elif _existing(folder, "resumen_multicaptura.json") or os.path.isdir(os.path.join(folder, "vista_01")):
        info["type"] = "multi"
        info["meta"] = _read_json(os.path.join(folder, "resumen_multicaptura.json")) or {}
        info["views"] = sorted(d for d in os.listdir(folder)
                               if d.startswith("vista_") and os.path.isdir(os.path.join(folder, d)))
    elif _existing(folder, "nube_puntos.ply", "metadata_camara.json"):
        info["type"] = "simple"
        info["meta"] = _read_json(os.path.join(folder, "metadata_camara.json")) or {}
    return info


_INFO_CACHE = {}


def cached_info(folder):
    """detect_scan con caché por fecha de modificación (el panel se redibuja a menudo)."""
    try:
        key = (folder, os.path.getmtime(folder))
    except OSError:
        key = (folder, None)
    if key not in _INFO_CACHE:
        _INFO_CACHE.clear()
        _INFO_CACHE[key] = detect_scan(folder)
    return _INFO_CACHE[key]


def info_lines(info):
    """Resumen legible para el panel."""
    m = info.get("meta") or {}
    t = info["type"]
    if t == "simple":
        intr = m.get("intrinsics") or {}
        res = f" · {intr.get('width')}x{intr.get('height')}" if intr else ""
        return [("Escaneo simple (1 cámara)" + res, 'INFO')]
    if t == "dual":
        lines = [("Escaneo dual (2 cámaras)", 'INFO')]
        if m.get("points_fused"):
            mesh = f" · malla {m['mesh_triangles']:,} triáng." if m.get("mesh_triangles") else ""
            lines.append((f"{m['points_fused']:,} puntos fusionados{mesh}", 'BLANK1'))
        cal = m.get("calibration") or {}
        if cal:
            if not cal.get("is_calibrated"):
                lines.append(("Sin calibrar: la Cam 2 puede estar mal colocada", 'ERROR'))
            else:
                q = cal.get("quality") or "-"
                cons = f" {cal['consistency_pct']} %" if cal.get("consistency_pct") is not None else ""
                lines.append((f"Calibración {q}{cons}", 'CHECKMARK' if q == "buena" else 'BLANK1'))
        if m.get("dt_ms") is not None:
            lines.append((f"Δt entre cámaras {m['dt_ms']} ms" + (f" ({m['dt_source']})" if m.get("dt_source") else ""),
                          'ERROR' if m["dt_ms"] > 50 else 'BLANK1'))
        if m.get("simulated"):
            lines.append(("Captura SIMULADA (sin cámaras reales)", 'ERROR'))
        return lines
    if t == "multi":
        lines = [(f"Multicaptura · {m.get('vistas', len(info.get('views', [])))} vistas", 'INFO')]
        if m.get("fitness_medio") is not None:
            lines.append((f"Fitness medio {m['fitness_medio']} · {m.get('puntos_fusionados', 0):,} puntos", 'BLANK1'))
        return lines
    return [("No es una carpeta de escaneo de RealSenseScans", 'ERROR')]


def _scan_sort_key(path):
    date = os.path.basename(os.path.dirname(path))
    hhmmss = os.path.basename(path).rsplit("_", 1)[-1]
    return date, hhmmss


def latest_scan(saves_dir):
    """Carpeta de escaneo más reciente (ignora carpetas vacías o de otro tipo)."""
    if not saves_dir or not os.path.isdir(saves_dir):
        return None
    found = []
    for date in os.listdir(saves_dir):
        dpath = os.path.join(saves_dir, date)
        if not os.path.isdir(dpath):
            continue
        for name in os.listdir(dpath):
            path = os.path.join(dpath, name)
            if os.path.isdir(path) and name.split("_")[0] in ("scan", "dual", "multi"):
                found.append(path)
    for path in sorted(found, key=_scan_sort_key, reverse=True):
        if detect_scan(path)["type"]:
            return path
    return None


# --------------------------------------------------------------------------- #
# Proyecto, Saves/ y Python con Open3D (sin rutas fijas)
# --------------------------------------------------------------------------- #
def _prefs():
    addon = bpy.context.preferences.addons.get(__name__)
    return addon.preferences if addon else None


def project_dir(scan_folder=""):
    """Raíz de RealSenseScans: preferencia, o subiendo desde el escaneo / desde este archivo."""
    p = _prefs()
    if p and p.project_dir and os.path.isdir(bpy.path.abspath(p.project_dir)):
        return os.path.normpath(bpy.path.abspath(p.project_dir))
    starts = [scan_folder] if scan_folder else []
    starts.append(os.path.dirname(os.path.abspath(__file__)))
    for start in starts:
        d = os.path.normpath(start)
        for _ in range(5):
            if os.path.isfile(os.path.join(d, "generar_malla_offline.py")):
                return d
            parent = os.path.dirname(d)
            if parent == d:
                break
            d = parent
    return None


def saves_dir(scan_folder=""):
    root = project_dir(scan_folder)
    if root and os.path.isdir(os.path.join(root, "Saves")):
        return os.path.join(root, "Saves")
    d = os.path.normpath(scan_folder) if scan_folder else ""
    while d and os.path.dirname(d) != d:
        if os.path.basename(d).lower() == "saves":
            return d
        d = os.path.dirname(d)
    return None


def python_exe(root):
    """Python con las dependencias del proyecto (Open3D): preferencia, .venv del proyecto o el del sistema."""
    p = _prefs()
    if p and p.python_exe and os.path.isfile(bpy.path.abspath(p.python_exe)):
        return bpy.path.abspath(p.python_exe)
    if root:
        for rel in (os.path.join(".venv", "Scripts", "python.exe"), os.path.join(".venv", "bin", "python")):
            cand = os.path.join(root, rel)
            if os.path.isfile(cand):
                return cand
    return shutil.which("python") or shutil.which("python3")


# --------------------------------------------------------------------------- #
# Materiales y puntos
# --------------------------------------------------------------------------- #
def vertex_color_material(attr_name):
    """Material que lee el color por vértice/punto (atributo de geometría)."""
    name = f"RS_Color_{attr_name}"
    mat = bpy.data.materials.get(name)
    if mat:
        return mat
    mat = bpy.data.materials.new(name)
    if mat.node_tree is None:
        mat.use_nodes = True
    nodes, links = mat.node_tree.nodes, mat.node_tree.links
    nodes.clear()
    out = nodes.new('ShaderNodeOutputMaterial')
    bsdf = nodes.new('ShaderNodeBsdfPrincipled')
    attr = nodes.new('ShaderNodeAttribute')
    attr.attribute_type = 'GEOMETRY'
    attr.attribute_name = attr_name
    bsdf.inputs['Roughness'].default_value = 0.9
    links.new(attr.outputs['Color'], bsdf.inputs['Base Color'])
    links.new(bsdf.outputs['BSDF'], out.inputs['Surface'])
    attr.location, bsdf.location, out.location = (-400, 0), (-150, 0), (150, 0)
    return mat


def points_node_group(mat):
    """Geometry Nodes: vértices -> nube de puntos con material (se ve en Cycles y EEVEE).

    Un grupo por material, con el material y el radio fijados en los nodos (no como entradas
    del modificador: su API cambió en Blender 5). El radio es común a todas las nubes.
    """
    name = f"{POINTS_GROUP}_{mat.name if mat else 'sin_color'}"
    ng = bpy.data.node_groups.get(name)
    if ng:
        return ng
    ng = bpy.data.node_groups.new(name, 'GeometryNodeTree')
    ng.interface.new_socket("Geometry", in_out='INPUT', socket_type='NodeSocketGeometry')
    ng.interface.new_socket("Geometry", in_out='OUTPUT', socket_type='NodeSocketGeometry')
    nodes, links = ng.nodes, ng.links
    gin, gout = nodes.new('NodeGroupInput'), nodes.new('NodeGroupOutput')
    to_points = nodes.new('GeometryNodeMeshToPoints')
    to_points.name = "RS_Radio"
    set_mat = nodes.new('GeometryNodeSetMaterial')
    set_mat.inputs['Material'].default_value = mat
    links.new(gin.outputs['Geometry'], to_points.inputs['Mesh'])
    links.new(to_points.outputs['Points'], set_mat.inputs['Geometry'])
    links.new(set_mat.outputs['Geometry'], gout.inputs['Geometry'])
    gin.location, to_points.location, set_mat.location, gout.location = (-450, 0), (-200, 0), (50, 0), (300, 0)
    return ng


def set_points_radius(radius):
    """Radio (m) de todas las nubes importadas."""
    for ng in bpy.data.node_groups:
        node = ng.nodes.get("RS_Radio") if ng.name.startswith(POINTS_GROUP) else None
        if node:
            node.inputs['Radius'].default_value = float(radius)


def _color_attr(obj):
    attrs = getattr(obj.data, "color_attributes", None)
    return attrs[0].name if attrs and len(attrs) else None


def setup_points(obj, radius):
    attr = _color_attr(obj)
    mat = vertex_color_material(attr) if attr else None
    if mat and not obj.data.materials:
        obj.data.materials.append(mat)
    mod = obj.modifiers.new("RS_Puntos", 'NODES')
    mod.node_group = points_node_group(mat)
    set_points_radius(radius)


def setup_mesh(obj):
    attr = _color_attr(obj)
    if attr and not obj.data.materials:  # la malla texturizada trae su propio material (.mtl)
        obj.data.materials.append(vertex_color_material(attr))


# --------------------------------------------------------------------------- #
# Escena: raíz, marcos, cámaras e importación de archivos
# --------------------------------------------------------------------------- #
def _empty(name, coll, parent=None, matrix=None, size=0.1, display='ARROWS'):
    ob = bpy.data.objects.new(name, None)
    ob.empty_display_type, ob.empty_display_size = display, size
    coll.objects.link(ob)
    if parent:
        ob.parent = parent
    if matrix is not None:
        ob.matrix_basis = matrix
    return ob


def _matrix(rows):
    return Matrix([list(map(float, r)) for r in rows])


def add_camera(name, coll, parent, intr, image_path=None, matrix_rs=None):
    """Cámara de Blender con las intrínsecas reales; la foto de la toma como fondo."""
    cd = bpy.data.cameras.new(name)
    cam = bpy.data.objects.new(name, cd)
    coll.objects.link(cam)
    cam.parent = parent
    cam.matrix_basis = (matrix_rs or Matrix.Identity(4)) @ RS_CAM_TO_BLENDER_CAM
    cd.display_size = 0.15
    cd.clip_start = 0.05
    if intr and intr.get("fx") and intr.get("width"):
        w, h = float(intr["width"]), float(intr["height"])
        cd.sensor_fit = 'HORIZONTAL'
        cd.sensor_width = 36.0
        cd.lens = float(intr["fx"]) * cd.sensor_width / w
        cd.shift_x = (w / 2.0 - float(intr["ppx"])) / max(w, h)
        cd.shift_y = (float(intr["ppy"]) - h / 2.0) / max(w, h)
        cam["rs_resolucion"] = [int(w), int(h)]
    if image_path and os.path.isfile(image_path):
        bg = cd.background_images.new()
        bg.image = bpy.data.images.load(image_path, check_existing=True)
        bg.alpha = 0.6
        cd.show_background_images = True
    return cam


def import_file(path, coll, parent, name, kind, radius=0.002, hidden=False):
    """Importa un PLY/OBJ sin conversión de ejes (el raíz la aplica) y lo cuelga de `parent`."""
    if not os.path.isfile(path):
        return None
    before = set(bpy.data.objects)
    if path.lower().endswith(".ply"):
        bpy.ops.wm.ply_import(filepath=path, forward_axis='Y', up_axis='Z')
    else:
        bpy.ops.wm.obj_import(filepath=path, forward_axis='Y', up_axis='Z')
    new = [o for o in bpy.data.objects if o not in before and o.type == 'MESH']
    if not new:
        return None
    if len(new) > 1:  # OBJ con varios grupos: un solo objeto por archivo
        with bpy.context.temp_override(active_object=new[0], selected_editable_objects=new):
            bpy.ops.object.join()
        new = [new[0]]
    obj = new[0]
    obj.name = name
    for c in list(obj.users_collection):
        c.objects.unlink(obj)
    coll.objects.link(obj)
    obj.parent = parent
    obj.matrix_parent_inverse = Matrix.Identity(4)
    obj.matrix_basis = Matrix.Identity(4)
    if kind == 'points':
        setup_points(obj, radius)
    else:
        setup_mesh(obj)
    if hidden:
        obj.hide_set(True)
        obj.hide_render = True
    return obj


def _scan_root(info, scene, blender_axes, create=True):
    """Colección + objeto raíz del escaneo. Con create=False reutiliza el último importado."""
    if not create:
        for ob in reversed(list(bpy.data.objects)):
            if ob.get("rs_carpeta") == info["folder"] and ob.users_collection:
                return ob.users_collection[0], ob
    coll = bpy.data.collections.new(f"RS_{info['id']}")
    scene.collection.children.link(coll)
    root = _empty(f"RS_{info['id']}", coll, size=0.25,
                  matrix=RS_TO_BLENDER if blender_axes else Matrix.Identity(4))
    root["rs_carpeta"] = info["folder"]
    root["rs_tipo"] = info["type"]
    return coll, root


def import_scan(info, opts, scene):
    """Construye el escaneo en la escena. Devuelve (objetos creados, avisos)."""
    f, t, m = info["folder"], info["type"], info.get("meta") or {}
    coll, root = _scan_root(info, scene, opts["blender_axes"])
    made, warnings = [], []
    r = opts["radius"]

    def add(path, name, kind, parent=root, hidden=False):
        ob = import_file(os.path.join(f, path), coll, parent, name, kind, r, hidden)
        if ob is None:
            warnings.append(f"No está {path}")
        else:
            made.append(ob)
        return ob

    if t == "simple":
        if opts["fused"]:
            add("nube_puntos.ply", "nube_puntos", 'points')
        if opts["heat"]:
            add("nube_calor_profundidad.ply", "nube_calor", 'points', hidden=opts["fused"])
        if opts["mesh"] and os.path.isfile(os.path.join(f, "malla_texturizada.obj")):
            add("malla_texturizada.obj", "malla_texturizada", 'mesh')
        if opts["cameras"]:
            made.append(add_camera("Cam", coll, root, m.get("intrinsics"), os.path.join(f, "captura_color.jpg")))

    elif t == "dual":
        T = m.get("extrinsics_cam2_to_cam1")
        cams = m.get("cameras") or {}
        frame2 = _empty("Cam 2 (marco)", coll, root, _matrix(T) if T else Matrix.Identity(4), size=0.08)
        if not T:
            warnings.append("metadata_dual.json sin extrínseca: Cam 2 en el origen")
        if opts["fused"]:
            add("nube_fusionada.ply", "nube_fusionada", 'points')
        if opts["per_camera"]:
            add("nube_cam1.ply", "nube_cam1", 'points', hidden=opts["fused"])
            add("nube_cam2.ply", "nube_cam2", 'points', parent=frame2, hidden=opts["fused"])
        if opts["mesh"]:
            add(m.get("mesh_file") or "malla_fusionada.obj", "malla_fusionada", 'mesh')
        if opts["diagnostic"]:
            add(m.get("overlap_diagnostic_file") or "diagnostico_solape.ply", "diagnostico_solape", 'points')
        if opts["cameras"]:
            made.append(add_camera("Cam 1", coll, root, (cams.get("cam1") or {}).get("intrinsics"),
                                   os.path.join(f, "cam1_color.jpg")))
            made.append(add_camera("Cam 2", coll, frame2, (cams.get("cam2") or {}).get("intrinsics"),
                                   os.path.join(f, "cam2_color.jpg")))

    elif t == "multi":
        if opts["fused"]:
            add("nube_fusionada.ply", "nube_fusionada", 'points')
        if opts["heat"]:
            add("nube_fusionada_calor.ply", "nube_fusionada_calor", 'points', hidden=opts["fused"])
        if opts["mesh"]:
            add("malla_fusionada.obj", "malla_fusionada", 'mesh')
        poses = m.get("poses") or []
        if opts["per_camera"] or opts["cameras"]:
            for i, view in enumerate(info.get("views", [])):
                vdir = os.path.join(f, view)
                pose = _matrix(poses[i]) if i < len(poses) else Matrix.Identity(4)
                if i >= len(poses):
                    warnings.append(f"{view}: sin pose en el resumen")
                frame = _empty(f"{view} (marco)", coll, root, pose, size=0.05)
                if opts["per_camera"]:
                    add(os.path.join(view, "nube_puntos.ply"), f"{view}_nube", 'points', parent=frame,
                        hidden=opts["fused"])
                if opts["cameras"]:
                    vm = _read_json(os.path.join(vdir, "metadata_vista.json")) or {}
                    made.append(add_camera(f"Cam {view}", coll, frame, vm.get("intrinsics"),
                                           os.path.join(vdir, "captura_color.jpg")))
    return made, warnings


def generate_offline_mesh(info):
    """Malla 2.5D de un escaneo simple con generar_malla_offline.py (Python del proyecto)."""
    root = project_dir(info["folder"])
    if not root:
        return False, "No encuentro el proyecto RealSenseScans: indícalo en las preferencias del addon."
    exe = python_exe(root)
    if not exe:
        return False, "No encuentro Python: indica en las preferencias el Python con Open3D."
    env = dict(os.environ, PYTHONIOENCODING="utf-8")  # en Windows el hijo escribiría en cp1252
    proc = subprocess.run([exe, os.path.join(root, "generar_malla_offline.py"), info["folder"]],
                          cwd=root, capture_output=True, text=True, encoding="utf-8", errors="replace", env=env)
    if proc.returncode != 0:
        last = (proc.stderr or proc.stdout or "").strip().splitlines()
        return False, f"generar_malla_offline.py falló ({exe}): {last[-1] if last else proc.returncode}"
    return True, exe


# --------------------------------------------------------------------------- #
# Interfaz
# --------------------------------------------------------------------------- #
def _opts(props):
    return {"fused": props.import_fused, "mesh": props.import_mesh, "per_camera": props.import_per_camera,
            "heat": props.import_heat, "diagnostic": props.import_diagnostic, "cameras": props.add_cameras,
            "radius": props.point_radius, "blender_axes": props.blender_axes}


def _folder(context):
    return bpy.path.abspath(context.scene.realsense_props.folder_path).strip().strip('"\'')


class REALSENSE_OT_latest(bpy.types.Operator):
    bl_idname = "realsense.latest_scan"
    bl_label = "Último escaneo"
    bl_description = "Selecciona el escaneo más reciente de Saves/"

    def execute(self, context):
        sd = saves_dir(_folder(context))
        path = latest_scan(sd)
        if not path:
            self.report({'ERROR'}, "No encuentro Saves/: indica la carpeta del proyecto en las preferencias del addon")
            return {'CANCELLED'}
        context.scene.realsense_props.folder_path = path
        self.report({'INFO'}, f"Último escaneo: {os.path.basename(path)}")
        return {'FINISHED'}


class REALSENSE_OT_import_scan(bpy.types.Operator):
    bl_idname = "realsense.import_scan"
    bl_label = "Importar escaneo"
    bl_description = "Importa nubes, malla y cámaras del escaneo con los ejes de Blender"
    bl_options = {'REGISTER', 'UNDO'}

    def execute(self, context):
        info = detect_scan(_folder(context))
        if not info["type"]:
            self.report({'ERROR'}, f"No es una carpeta de escaneo de RealSenseScans: {info['folder']}")
            return {'CANCELLED'}
        made, warnings = import_scan(info, _opts(context.scene.realsense_props), context.scene)
        for w in warnings:
            self.report({'WARNING'}, w)
        if not made:
            self.report({'ERROR'}, "No se importó nada (revisa las opciones marcadas)")
            return {'CANCELLED'}
        self.report({'INFO'}, f"{info['id']}: {len(made)} objetos importados")
        return {'FINISHED'}


class REALSENSE_OT_generate_mesh(bpy.types.Operator):
    bl_idname = "realsense.generate_mesh"
    bl_label = "Generar malla (offline)"
    bl_description = ("Escaneo simple: genera malla_texturizada.obj con generar_malla_offline.py "
                      "(Python del proyecto) y la importa")
    bl_options = {'REGISTER', 'UNDO'}

    def execute(self, context):
        info = detect_scan(_folder(context))
        if info["type"] != "simple":
            self.report({'ERROR'}, "Solo para escaneos simples: los duales y multicaptura ya traen malla_fusionada.obj")
            return {'CANCELLED'}
        ok, msg = generate_offline_mesh(info)
        if not ok:
            self.report({'ERROR'}, msg)
            return {'CANCELLED'}
        coll, root = _scan_root(info, context.scene, context.scene.realsense_props.blender_axes, create=False)
        for child in list(root.children):  # se acaba de regenerar: sustituir la importada antes
            if child.name.startswith("malla_texturizada"):
                bpy.data.objects.remove(child)
        ob = import_file(os.path.join(info["folder"], "malla_texturizada.obj"), coll, root, "malla_texturizada", 'mesh')
        if ob is None:
            self.report({'ERROR'}, "No se generó malla_texturizada.obj")
            return {'CANCELLED'}
        self.report({'INFO'}, "Malla generada e importada")
        return {'FINISHED'}


class RealSenseProperties(bpy.types.PropertyGroup):
    folder_path: bpy.props.StringProperty(
        name="", description="Carpeta del escaneo (Saves/AAAA-MM-DD/scan_, dual_scan_ o multi_)",
        default="", subtype='DIR_PATH')
    import_fused: bpy.props.BoolProperty(name="Nube principal", default=True,
                                         description="Nube fusionada (dual/multi) o nube del escaneo (simple)")
    import_mesh: bpy.props.BoolProperty(name="Malla", default=True)
    import_per_camera: bpy.props.BoolProperty(
        name="Nubes por cámara / vista", default=False,
        description="Dual: nube_cam1 y nube_cam2 en sus marcos. Multi: la nube de cada vista con su pose")
    import_heat: bpy.props.BoolProperty(name="Nube de calor (profundidad)", default=False)
    import_diagnostic: bpy.props.BoolProperty(
        name="Diagnóstico de solape", default=False,
        description="Dual: verde = solape coherente, rojo/azul = zonas de una sola cámara")
    add_cameras: bpy.props.BoolProperty(name="Cámaras con su foto", default=True)
    point_radius: bpy.props.FloatProperty(name="Radio de punto", default=0.002, min=0.0001, max=0.05,
                                          subtype='DISTANCE', unit='LENGTH', precision=4,
                                          description="Radio de los puntos de todas las nubes importadas",
                                          update=lambda self, ctx: set_points_radius(self.point_radius))
    blender_axes: bpy.props.BoolProperty(
        name="Ejes de Blender (Z arriba)", default=True,
        description="Convierte de RealSense (x der., y abajo, z adelante) a Blender: (x, z, -y)")


class RealSensePreferences(bpy.types.AddonPreferences):
    bl_idname = __name__
    project_dir: bpy.props.StringProperty(
        name="Carpeta del proyecto", subtype='DIR_PATH',
        description="Raíz de RealSenseScans (contiene Saves/ y generar_malla_offline.py). Vacío: se deduce")
    python_exe: bpy.props.StringProperty(
        name="Python con Open3D", subtype='FILE_PATH',
        description="Para generar mallas offline. Vacío: .venv del proyecto o el python del sistema")

    def draw(self, context):
        self.layout.prop(self, "project_dir")
        self.layout.prop(self, "python_exe")


class REALSENSE_PT_panel(bpy.types.Panel):
    bl_label = "RealSense Importer"
    bl_idname = "REALSENSE_PT_panel"
    bl_space_type = 'VIEW_3D'
    bl_region_type = 'UI'
    bl_category = 'RealSense'

    def draw(self, context):
        layout = self.layout
        props = context.scene.realsense_props

        box = layout.box()
        box.label(text="Carpeta del escaneo", icon='FILE_FOLDER')
        box.prop(props, "folder_path")
        box.operator("realsense.latest_scan", icon='TIME')

        folder = _folder(context)
        info = cached_info(folder) if folder else None
        if info:
            col = layout.box().column(align=True)
            for text, icon in info_lines(info):
                col.label(text=text, icon=icon)

        col = layout.column(align=True)
        col.prop(props, "import_fused")
        col.prop(props, "import_mesh")
        if not info or info["type"] in ("dual", "multi"):
            col.prop(props, "import_per_camera")
        if not info or info["type"] in ("simple", "multi"):
            col.prop(props, "import_heat")
        if not info or info["type"] == "dual":
            col.prop(props, "import_diagnostic")
        col.prop(props, "add_cameras")
        col.prop(props, "point_radius")
        col.prop(props, "blender_axes")

        row = layout.row()
        row.scale_y = 1.4
        row.enabled = bool(info and info["type"])
        row.operator("realsense.import_scan", icon='IMPORT')
        if info and info["type"] == "simple":
            layout.operator("realsense.generate_mesh", icon='MESH_CUBE')


classes = (
    RealSenseProperties,
    RealSensePreferences,
    REALSENSE_OT_latest,
    REALSENSE_OT_import_scan,
    REALSENSE_OT_generate_mesh,
    REALSENSE_PT_panel,
)


def register():
    for cls in classes:
        bpy.utils.register_class(cls)
    bpy.types.Scene.realsense_props = bpy.props.PointerProperty(type=RealSenseProperties)


def unregister():
    for cls in reversed(classes):
        bpy.utils.unregister_class(cls)
    del bpy.types.Scene.realsense_props


if __name__ == "__main__":
    register()
