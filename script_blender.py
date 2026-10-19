"""
Importa en Blender el escaneo más reciente de Saves/ usando el addon RealSense_Importer.py.

Uso: pestaña Scripting > abrir este archivo > Run Script, o
     blender --python script_blender.py
Para elegir otra carpeta, instala el addon (Edit > Preferences > Add-ons > Install from Disk)
y usa el panel RealSense de la barra lateral del visor 3D.
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

spec = importlib.util.spec_from_file_location("RealSense_Importer", os.path.join(HERE, "RealSense_Importer.py"))
rsi = importlib.util.module_from_spec(spec)
sys.modules["RealSense_Importer"] = rsi
spec.loader.exec_module(rsi)
if not hasattr(bpy.types.Scene, "realsense_props"):
    rsi.register()

folder = rsi.latest_scan(os.path.join(HERE, "Saves"))
if not folder:
    print(f"No hay escaneos en {os.path.join(HERE, 'Saves')}")
else:
    bpy.context.scene.realsense_props.folder_path = folder
    print(f"Importando el último escaneo: {folder}")
    bpy.ops.realsense.import_scan()
