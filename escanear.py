import open3d as o3d
import json
import time

# 2. Inicializar el sensor RealSense D455
print("Iniciando la cámara Intel RealSense D455...")
rscam = o3d.t.io.RealSenseSensor()
rscam.init_sensor(o3d.t.io.RealSenseSensorConfig())
rscam.start_capture()

print("\n==========================================================")
print("¡CÁMARA CONECTADA!")
print("-> Se abrirá una ventana flotante con tu escaneo 3D.")
print("-> Mueve la cámara suavemente para capturar tu entorno.")
print("-> Para terminar y GUARDAR el escaneo, CIERRA la ventana.")
print("==========================================================\n")

# Crear la ventana visualizadora
vis = o3d.visualization.Visualizer()
vis.create_window("Escaneo en vivo - RealSense D455", width=1280, height=720)

pcd = o3d.geometry.PointCloud()
vis.add_geometry(pcd)
keep_running = True

try:
    while keep_running:
        # Capturar cuadro sin bloquear (wait=False)
        rgbd_frame = rscam.capture_frame(wait=False, align_depth_to_color=True)
        
        if rgbd_frame is None or rgbd_frame.color.is_empty():
            # Pequeña pausa para no saturar el CPU mientras esperamos datos
            time.sleep(0.01)
        else:
            # Convertir datos del sensor a una Nube de Puntos 3D
            intrinsic = rscam.get_metadata().intrinsics
            intrinsic_tensor = o3d.core.Tensor(intrinsic.intrinsic_matrix, o3d.core.Dtype.Float64)
            current_pcd_t = o3d.t.geometry.PointCloud.create_from_rgbd_image(
                rgbd_frame, intrinsic_tensor
            )
            current_pcd = current_pcd_t.to_legacy()
            
            # Actualizar lo que se ve en la pantalla
            pcd.points = current_pcd.points
            pcd.colors = current_pcd.colors
            vis.update_geometry(pcd)
        
        # Estas dos funciones deben llamarse siempre para que la ventana no se congele
        keep_running = vis.poll_events()
        vis.update_renderer()

except KeyboardInterrupt:
    pass

finally:
    # 3. Guardar el archivo 3D resultante
    nombre_archivo = "mi_primer_escaneo.ply"
    print(f"Guardando nube de puntos tridimensional en: {nombre_archivo}...")
    o3d.io.write_point_cloud(nombre_archivo, pcd)
    
    # Apagar la cámara de forma limpia
    rscam.stop_capture()
    vis.destroy_window()
    print("¡Listo! El archivo se guardó correctamente.")
