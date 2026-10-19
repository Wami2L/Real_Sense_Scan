"""
Abre en Blender un escaneo con el addon RealSense_Importer.py (sin instalarlo).

    blender --python script_blender.py                         # último escaneo de Saves/
    blender --python script_blender.py -- --objeto <id>        # objeto de Saves/objetos (lo usa /objetos)

También: pestaña Scripting > abrir este archivo > Run Script. Para usar los paneles a diario,
instala el addon (Edit > Preferences > Add-ons > Install from Disk > RealSense_Importer.py).
"""

import importlib.util
import os
import sys

import bpy

HERE = os.path.dirname(os.path.abspath(__file__)) if "__file__" in globals() else ""
if not os.path.isfile(os.path.join(HERE, "RealSense_Importer.py")):
    # Ejecutado desde el editor de texto de Blender: buscar junto al .py abierto
    txt = next((t for t in bpy.data.texts if t.filepath.endswith("script_blender.py")), None)
    HERE = os.path.dirname(bpy.path.abspath(txt.filepath)) if txt else HERE

if "RealSense_Importer" in sys.modules and hasattr(bpy.types.Scene, "realsense_props"):
    rsi = sys.modules["RealSense_Importer"]       # addon ya instalado/registrado
else:
    spec = importlib.util.spec_from_file_location("RealSense_Importer", os.path.join(HERE, "RealSense_Importer.py"))
    rsi = importlib.util.module_from_spec(spec)
    sys.modules["RealSense_Importer"] = rsi
    spec.loader.exec_module(rsi)
    if not hasattr(bpy.types.Scene, "realsense_props"):
        rsi.register()

argv = sys.argv[sys.argv.index("--") + 1:] if "--" in sys.argv else []
oid = argv[argv.index("--objeto") + 1] if "--objeto" in argv and argv.index("--objeto") + 1 < len(argv) else None

if oid:
    # escena limpia: quitar el cubo, la cámara y la luz por defecto si siguen ahí
    for name in ("Cube",):
        ob = bpy.data.objects.get(name)
        if ob:
            bpy.data.objects.remove(ob, do_unlink=True)
    ob, man = rsi.open_library_object(oid, bpy.context)
    print(f"Abierto el objeto «{ob.name}» ({oid}). Panel RealSense > Objetos escaneados > Publicar en Unity.")
    for area in getattr(bpy.context.screen, "areas", []):
        if area.type == 'VIEW_3D':
            with bpy.context.temp_override(area=area, region=next(r for r in area.regions if r.type == 'WINDOW')):
                bpy.ops.view3d.view_selected()
            area.spaces[0].show_region_ui = True
else:
    folder = rsi.latest_scan(os.path.join(HERE, "Saves"))
    if not folder:
        print(f"No hay escaneos en {os.path.join(HERE, 'Saves')}")
    else:
        bpy.context.scene.realsense_props.folder_path = folder
        print(f"Importando el último escaneo: {folder}")
        bpy.ops.realsense.import_scan()
