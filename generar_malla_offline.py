import sys
import os
import json
import numpy as np
import cv2
import open3d as o3d
from scipy.spatial import Delaunay

def main():
    if len(sys.argv) < 2:
        print("Uso: python generar_malla_offline.py <ruta_del_escaneo>")
        sys.exit(1)
        
    scan_dir = sys.argv[1]
    pcd_path = os.path.join(scan_dir, "nube_puntos.ply")
    img_path = os.path.join(scan_dir, "captura_color.jpg")
    json_path = os.path.join(scan_dir, "metadata_camara.json")
    
    if not os.path.exists(pcd_path) or not os.path.exists(img_path) or not os.path.exists(json_path):
        print("Error: Faltan archivos (ply, jpg o json) en la carpeta seleccionada.")
        sys.exit(1)
        
    print(f"Cargando archivos de: {scan_dir}")
    pcd = o3d.io.read_point_cloud(pcd_path)
    color_bgr = cv2.imread(img_path)
    
    with open(json_path, 'r') as f:
        metadata = json.load(f)
        
    matrix = metadata["intrinsic_matrix"]
    fx, fy = matrix[0][0], matrix[1][1]
    cx, cy = matrix[2][0], matrix[2][1]
    w, h = metadata["width"], metadata["height"]
    
    points = np.asarray(pcd.points)
    
    print("Proyectando puntos a 2D para triangulación perfecta...")
    # 1. Proyectar a coordenadas de imagen (Pixeles)
    z = points[:, 2]
    # Evitar div por cero
    z_safe = np.where(z == 0, 1e-5, z)
    u = (points[:, 0] * fx / z_safe) + cx
    v = (points[:, 1] * fy / z_safe) + cy
    
    uv_pixels = np.column_stack((u, v))
    
    print("Ejecutando Triangulación de Delaunay 2.5D (Esto garantiza geometría limpia)...")
    tri = Delaunay(uv_pixels)
    triangles = tri.simplices
    
    # 2. Filtrar triángulos estirados (aquellos que conectan el fondo con el frente)
    print("Limpiando bordes falsos (límite de 5cm de diferencia de profundidad)...")
    v0 = points[triangles[:, 0]]
    v1 = points[triangles[:, 1]]
    v2 = points[triangles[:, 2]]
    
    # Longitudes de las aristas en 3D
    l0 = np.linalg.norm(v0 - v1, axis=1)
    l1 = np.linalg.norm(v1 - v2, axis=1)
    l2 = np.linalg.norm(v2 - v0, axis=1)
    
    # Borrar cualquier triángulo que tenga una arista de más de 5 cm de largo
    max_edge = 0.05 
    valid_mask = (l0 < max_edge) & (l1 < max_edge) & (l2 < max_edge)
    valid_triangles = triangles[valid_mask]
    
    print(f"Triángulos válidos: {len(valid_triangles)} de {len(triangles)}")
    
    # 3. Construir Malla
    mesh = o3d.geometry.TriangleMesh()
    mesh.vertices = o3d.utility.Vector3dVector(points)
    mesh.triangles = o3d.utility.Vector3iVector(valid_triangles)
    
    # 4. Asignar UVs y Textura
    print("Generando Coordenadas UV para la Textura...")
    # Normalizar UVs
    u_norm = u / w
    v_norm = 1.0 - (v / h) # OBJ requiere Invertir el eje Y
    
    uv_normalized = np.column_stack((u_norm, v_norm))
    # Open3D requiere un UV por cada vértice de cada triángulo (N*3, 2)
    tri_uvs = uv_normalized[valid_triangles.flatten()]
    
    mesh.triangle_uvs = o3d.utility.Vector2dVector(tri_uvs)
    mesh.triangle_material_ids = o3d.utility.IntVector(np.zeros(len(valid_triangles), dtype=np.int32))
    
    color_rgb_img = cv2.cvtColor(color_bgr, cv2.COLOR_BGR2RGB)
    mesh.textures = [o3d.geometry.Image(color_rgb_img)]
    
    # Calcular normales para que Blender no lo muestre plano
    mesh.compute_vertex_normals()
    
    obj_path = os.path.join(scan_dir, "malla_texturizada.obj")
    o3d.io.write_triangle_mesh(obj_path, mesh, write_triangle_uvs=True)
    print(f"EXITO: Malla Perfecta generada en {obj_path}")

if __name__ == "__main__":
    main()
