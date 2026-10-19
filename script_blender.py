import bpy
import os

# ------------------------------------------------------------------------
# SCRIPT PARA BLENDER: Importador de Escaneos RealSense
# ------------------------------------------------------------------------
# Instrucciones:
# 1. Abre Blender
# 2. Ve a la pestaña "Scripting" (arriba en el menú principal)
# 3. Abre este archivo o copia y pega el código aquí
# 4. Presiona el botón de "Play" (Run Script)
# ------------------------------------------------------------------------

BASE_DIR = r"c:\Users\Usuario\Documents\RealSenseScans\Saves"

def get_latest_scan_folder():
    """Encuentra la subcarpeta del escaneo más reciente automáticamente."""
    if not os.path.exists(BASE_DIR):
        return None
    dates = sorted(os.listdir(BASE_DIR), reverse=True)
    if not dates: return None
    
    latest_date_path = os.path.join(BASE_DIR, dates[0])
    scans = sorted(os.listdir(latest_date_path), reverse=True)
    
    if not scans: return None
    return os.path.join(latest_date_path, scans[0])

def setup_point_cloud_material(obj):
    """Crea un material que lee los colores originales del escáner"""
    mat = bpy.data.materials.new(name=f"Material_{obj.name}")
    mat.use_nodes = True
    nodes = mat.node_tree.nodes
    links = mat.node_tree.links
    
    for n in nodes: nodes.remove(n)
        
    output_node = nodes.new(type='ShaderNodeOutputMaterial')
    bsdf_node = nodes.new(type='ShaderNodeBsdfPrincipled')
    attr_node = nodes.new(type='ShaderNodeAttribute')
    
    # Open3D PLY guarda el color y Blender lo suele mapear a "Col" o "vertex_colors"
    attr_node.attribute_name = "Col" 
    
    links.new(attr_node.outputs['Color'], bsdf_node.inputs['Base Color'])
    links.new(bsdf_node.outputs['BSDF'], output_node.inputs['Surface'])
    
    if len(obj.data.materials) == 0:
        obj.data.materials.append(mat)
    else:
        obj.data.materials[0] = mat
    
    return mat

def convert_to_spheres_geometry_nodes(obj, mat):
    """
    Blender nativamente no 'renderiza' puntos sueltos en el motor Cycles/Eevee.
    Esta función usa Geometry Nodes para instanciar pequeñas esferas en cada 
    punto 3D para que el modelo luzca sólido y se pueda exportar/renderizar.
    """
    modifier = obj.modifiers.new(name="Puntos_a_Malla", type='NODES')
    node_tree = bpy.data.node_groups.new(name=f"GeoNodes_{obj.name}", type='GeometryNodeTree')
    modifier.node_group = node_tree
    
    nodes = node_tree.nodes
    links = node_tree.links
    
    input_node = nodes.new('NodeGroupInput')
    output_node = nodes.new('NodeGroupOutput')
    
    node_tree.interface.new_socket(name="Geometry", in_out='INPUT', socket_type='NodeSocketGeometry')
    node_tree.interface.new_socket(name="Geometry", in_out='OUTPUT', socket_type='NodeSocketGeometry')
    
    instance_node = nodes.new('GeometryNodeInstanceOnPoints')
    
    # Forma del punto (una pequeña esfera icosaédrica)
    sphere_node = nodes.new('GeometryNodeMeshIcoSphere')
    sphere_node.inputs['Radius'].default_value = 0.003 # 3 milímetros de radio por punto
    sphere_node.inputs['Subdivisions'].default_value = 1
    
    # Realizar Instancia
    links.new(input_node.outputs['Geometry'], instance_node.inputs['Points'])
    links.new(sphere_node.outputs['Mesh'], instance_node.inputs['Instance'])
    links.new(instance_node.outputs['Instances'], output_node.inputs['Geometry'])

def import_scan():
    folder_path = get_latest_scan_folder()
    if not folder_path:
        print(f"No se encontró la carpeta: {BASE_DIR}")
        return
        
    print(f"Leyendo último escaneo en: {folder_path}")
    
    # Detectar si es multiscan o escaneo normal
    is_multiscan = os.path.exists(os.path.join(folder_path, "nube_fusionada.ply"))
    
    if is_multiscan:
        files_to_import = ["nube_fusionada.ply", "nube_fusionada_calor.ply"]
    else:
        files_to_import = ["nube_puntos.ply", "nube_calor_profundidad.ply"]
    
    for i, filename in enumerate(files_to_import):
        filepath = os.path.join(folder_path, filename)
        if os.path.exists(filepath):
            bpy.ops.object.select_all(action='DESELECT')
            
            # API de Importación depende de la versión de Blender
            try:
                bpy.ops.wm.ply_import(filepath=filepath) # Blender 4.0+
            except AttributeError:
                bpy.ops.import_mesh.ply(filepath=filepath) # Blender 3.x
                
            obj = bpy.context.active_object
            if obj:
                obj.name = filename.split('.')[0]
                
                # Mover el mapa de calor un poco al lado (eje X) para comparar
                if i == 1:
                    obj.location.x += 1.5 
                
                mat = setup_point_cloud_material(obj)
                convert_to_spheres_geometry_nodes(obj, mat)
                print(f"Modelo importado con éxito: {filename}")
        else:
            print(f"Archivo faltante: {filepath}")

if __name__ == "__main__":
    import_scan()
