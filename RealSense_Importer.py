bl_info = {
    "name": "RealSense Scan Importer",
    "author": "Antigravity",
    "version": (1, 2),
    "blender": (3, 0, 0),
    "location": "View3D > Sidebar > RealSense",
    "description": "Importa escaneos PLY y Genera Mallas 3D texturizadas offline.",
    "category": "Import-Export",
}

import bpy
import os
import subprocess

def setup_point_cloud_material(obj):
    mat = bpy.data.materials.new(name=f"Mat_{obj.name}")
    mat.use_nodes = True
    nodes = mat.node_tree.nodes
    links = mat.node_tree.links
    for n in nodes: nodes.remove(n)
    output_node = nodes.new(type='ShaderNodeOutputMaterial')
    bsdf_node = nodes.new(type='ShaderNodeBsdfPrincipled')
    attr_node = nodes.new(type='ShaderNodeAttribute')
    attr_node.attribute_name = "Col" 
    links.new(attr_node.outputs['Color'], bsdf_node.inputs['Base Color'])
    links.new(bsdf_node.outputs['BSDF'], output_node.inputs['Surface'])
    if len(obj.data.materials) == 0:
        obj.data.materials.append(mat)
    else:
        obj.data.materials[0] = mat
    return mat

def convert_to_spheres_geometry_nodes(obj):
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
    sphere_node = nodes.new('GeometryNodeMeshIcoSphere')
    sphere_node.inputs['Radius'].default_value = 0.003
    sphere_node.inputs['Subdivisions'].default_value = 1
    links.new(input_node.outputs['Geometry'], instance_node.inputs['Points'])
    links.new(sphere_node.outputs['Mesh'], instance_node.inputs['Instance'])
    links.new(instance_node.outputs['Instances'], output_node.inputs['Geometry'])

class REALSENSE_OT_import_scan(bpy.types.Operator):
    bl_idname = "realsense.import_scan"
    bl_label = "Importar Nubes de Puntos"
    bl_description = "Importa los archivos PLY crudos de la carpeta seleccionada"
    
    def execute(self, context):
        folder_path = context.scene.realsense_props.folder_path
        folder_path = bpy.path.abspath(folder_path).strip('"\'')
        if not folder_path or not os.path.exists(folder_path):
            self.report({'ERROR'}, f"Ruta inválida: {folder_path}")
            return {'CANCELLED'}
        files_imported = 0
        # Detectar si es un escaneo simple o un multiscan
        is_multiscan = os.path.exists(os.path.join(folder_path, "nube_fusionada.ply"))
        
        if is_multiscan:
            files_to_import = ["nube_fusionada.ply", "nube_fusionada_calor.ply"]
        else:
            files_to_import = ["nube_puntos.ply", "nube_calor_profundidad.ply"]
            
        scan_name = os.path.basename(os.path.normpath(folder_path))
        if scan_name == "": scan_name = "Scan"
        new_col = bpy.data.collections.new(name=f"Puntos_RS_{scan_name}")
        context.scene.collection.children.link(new_col)
        
        for i, filename in enumerate(files_to_import):
            filepath = os.path.join(folder_path, filename)
            if os.path.exists(filepath):
                bpy.ops.object.select_all(action='DESELECT')
                try:
                    bpy.ops.wm.ply_import(filepath=filepath)
                except AttributeError:
                    try:
                        bpy.ops.import_mesh.ply(filepath=filepath)
                    except Exception as e:
                        self.report({'ERROR'}, f"Fallo al importar: {str(e)}")
                        continue
                obj = bpy.context.active_object
                if obj:
                    obj.name = filename.split('.')[0]
                    if i == 1: obj.location.x += 1.5 
                    for col in obj.users_collection: col.objects.unlink(obj)
                    new_col.objects.link(obj)
                    setup_point_cloud_material(obj)
                    convert_to_spheres_geometry_nodes(obj)
                    files_imported += 1
        
        if files_imported > 0:
            self.report({'INFO'}, f"Se importaron {files_imported} nubes de puntos.")
            return {'FINISHED'}
        else:
            self.report({'WARNING'}, "No se encontraron nubes PLY en esa carpeta.")
            bpy.data.collections.remove(new_col)
            return {'CANCELLED'}


class REALSENSE_OT_generate_mesh(bpy.types.Operator):
    bl_idname = "realsense.generate_mesh"
    bl_label = "Generar Malla Texturizada"
    bl_description = "Ejecuta un script offline para generar y texturizar la malla 3D desde los archivos locales, y la importa a Blender."
    
    def execute(self, context):
        folder_path = context.scene.realsense_props.folder_path
        folder_path = bpy.path.abspath(folder_path).strip('"\'')
        
        if not folder_path or not os.path.exists(folder_path):
            self.report({'ERROR'}, f"Ruta inválida: {folder_path}")
            return {'CANCELLED'}
            
        # Si es un multiscan, la malla ya fue generada
        if os.path.exists(os.path.join(folder_path, "malla_fusionada.obj")):
            obj_path = os.path.join(folder_path, "malla_fusionada.obj")
            self.report({'INFO'}, "Multiscan detectado. Importando malla fusionada directamente...")
        else:
            script_path = r"c:\Users\Usuario\Documents\RealSenseScans\generar_malla_offline.py"
            if not os.path.exists(script_path):
                self.report({'ERROR'}, f"No se encontró el script: {script_path}")
                return {'CANCELLED'}
                
            try:
                subprocess.run(["python", script_path, folder_path], check=True)
            except subprocess.CalledProcessError as e:
                self.report({'ERROR'}, f"Error al generar la malla. Verifica la consola de Python.")
                return {'CANCELLED'}
            except Exception as e:
                self.report({'ERROR'}, f"Fallo en la ejecución: {e}")
                return {'CANCELLED'}
                
            obj_path = os.path.join(folder_path, "malla_texturizada.obj")

        if os.path.exists(obj_path):
            bpy.ops.object.select_all(action='DESELECT')
            try:
                bpy.ops.wm.obj_import(filepath=obj_path)
            except AttributeError:
                bpy.ops.import_scene.obj(filepath=obj_path)
                
            obj = bpy.context.selected_objects[0] if bpy.context.selected_objects else None
            
            # En un multiscan la malla puede venir separada en varios objetos al importar (dependiendo de los grupos/materiales).
            # Para cumplir con "crear un solo archivo/objeto con las capturas", unimos todo lo importado si hay varios.
            if len(bpy.context.selected_objects) > 1:
                bpy.context.view_layer.objects.active = bpy.context.selected_objects[0]
                bpy.ops.object.join()
                obj = bpy.context.active_object
                
            if obj:
                scan_name = os.path.basename(os.path.normpath(folder_path))
                if scan_name == "": scan_name = "Scan"
                new_col = bpy.data.collections.new(name=f"Malla_RS_{scan_name}")
                context.scene.collection.children.link(new_col)
                for col in obj.users_collection: col.objects.unlink(obj)
                new_col.objects.link(obj)
                
                self.report({'INFO'}, "Malla 3D importada con éxito.")
                return {'FINISHED'}
        else:
            self.report({'ERROR'}, "El archivo .obj no se encontró o no se generó.")
            return {'CANCELLED'}


class RealSenseProperties(bpy.types.PropertyGroup):
    folder_path: bpy.props.StringProperty(
        name="",
        description="Selecciona la subcarpeta del escaneo (ej. scan_151829)",
        default="",
        subtype='DIR_PATH'
    )

class REALSENSE_PT_panel(bpy.types.Panel):
    bl_label = "RealSense Importer"
    bl_idname = "REALSENSE_PT_panel"
    bl_space_type = 'VIEW_3D'
    bl_region_type = 'UI'
    bl_category = 'RealSense'
    
    def draw(self, context):
        layout = self.layout
        scene = context.scene
        props = scene.realsense_props
        
        box = layout.box()
        box.label(text="Carpeta del escaneo antiguo:", icon='FILE_FOLDER')
        box.prop(props, "folder_path")
        
        layout.separator()
        layout.label(text="1. Importar Puntos Nativos", icon='FILE_3D')
        row = layout.row()
        row.scale_y = 1.2
        row.operator("realsense.import_scan", icon='GROUP_VERTEX')
        
        layout.separator()
        layout.label(text="2. Construir Superficie", icon='MOD_MESHDEFORM')
        row2 = layout.row()
        row2.scale_y = 1.5
        row2.operator("realsense.generate_mesh", icon='MESH_CUBE')

classes = (
    RealSenseProperties,
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
