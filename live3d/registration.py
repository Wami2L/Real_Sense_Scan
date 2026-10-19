"""
Utilidades geométricas del sistema dual: convención de Euler (única para todo el proyecto),
snapshot -> nube, integración TSDF y nube de diagnóstico de solape.

La alineación automática Cam 2 -> Cam 1 vive en live3d/alignment.py.
"""

import math
import numpy as np
import open3d as o3d


def snap_to_pcd(snap, depth_trunc=2.5, min_depth=0.25):
    """Convierte un snapshot de cámara (color RGB, depth uint16, intrinsics) a Open3D PointCloud."""
    col = o3d.geometry.Image(snap["color"])
    dep = o3d.geometry.Image(snap["depth"])
    intr = snap["intrinsics"]
    pinhole = o3d.camera.PinholeCameraIntrinsic(
        int(intr["width"]), int(intr["height"]),
        float(intr["fx"]), float(intr["fy"]),
        float(intr["ppx"]), float(intr["ppy"])
    )
    scale = float(snap.get("depth_scale", 0.001))
    rgbd = o3d.geometry.RGBDImage.create_from_color_and_depth(
        col, dep, depth_scale=1.0 / scale, depth_trunc=depth_trunc, convert_rgb_to_intensity=False
    )
    pcd = o3d.geometry.PointCloud.create_from_rgbd_image(rgbd, pinhole)

    # Filtrar puntos más cercanos que min_depth
    if len(pcd.points) > 0 and min_depth > 0:
        pts = np.asarray(pcd.points)
        mask = pts[:, 2] >= min_depth
        pcd = pcd.select_by_index(np.where(mask)[0])

    return pcd


# Convención de ángulos en el marco de cámara RealSense (x der, y abajo, z delante):
#   Yaw   = giro sobre Y (convergencia horizontal entre cámaras)
#   Pitch = giro sobre X (inclinación arriba/abajo)
#   Roll  = giro sobre Z (eje óptico)
#   R = Ry(yaw) · Rx(pitch) · Rz(roll)   -- idéntica en live3d/web/js/main.js
def matrix_to_euler_translation(T):
    """Descompone matriz 4x4 en traslación (cm) y Yaw/Pitch/Roll (grados)."""
    T = np.asarray(T, dtype=np.float64)
    R, t = T[:3, :3], T[:3, 3]
    sp = float(np.clip(-R[1, 2], -1.0, 1.0))
    pitch = math.asin(sp)
    if abs(math.cos(pitch)) > 1e-6:
        yaw = math.atan2(R[0, 2], R[2, 2])
        roll = math.atan2(R[1, 0], R[1, 1])
    else:  # bloqueo de cardán
        yaw = math.atan2(-R[2, 0], R[0, 0])
        roll = 0.0
    return {
        "translation_cm": {"x": round(float(t[0]) * 100, 2), "y": round(float(t[1]) * 100, 2),
                           "z": round(float(t[2]) * 100, 2)},
        "rotation_deg": {"yaw": round(math.degrees(yaw), 2), "pitch": round(math.degrees(pitch), 2),
                         "roll": round(math.degrees(roll), 2)},
    }


def euler_translation_to_matrix(tx_cm, ty_cm, tz_cm, yaw_deg, pitch_deg, roll_deg):
    """Construye la matriz 4x4 a partir de traslación (cm) y Yaw/Pitch/Roll (grados)."""
    y, p, r = (math.radians(float(a)) for a in (yaw_deg, pitch_deg, roll_deg))
    Ry = np.array([[math.cos(y), 0, math.sin(y)], [0, 1, 0], [-math.sin(y), 0, math.cos(y)]])
    Rx = np.array([[1, 0, 0], [0, math.cos(p), -math.sin(p)], [0, math.sin(p), math.cos(p)]])
    Rz = np.array([[math.cos(r), -math.sin(r), 0], [math.sin(r), math.cos(r), 0], [0, 0, 1]])
    T = np.eye(4, dtype=np.float64)
    T[:3, :3] = Ry @ Rx @ Rz
    T[:3, 3] = [float(tx_cm) / 100.0, float(ty_cm) / 100.0, float(tz_cm) / 100.0]
    return T


def integrate_dual_tsdf(snap1, snap2, T_2_to_1, voxel_length=0.003, sdf_trunc=0.015, depth_trunc=2.5):
    """
    Integra las capturas RGB-D de ambas cámaras en un único volumen TSDF continuo (Curless & Levoy / KinectFusion),
    extrayendo una malla 3D triangular (Marching Cubes) estanca y sin duplicados en la zona de solape.
    
    Args:
        snap1: Snapshot dict de Cámara 1 (color, depth, intrinsics, depth_scale)
        snap2: Snapshot dict de Cámara 2 (color, depth, intrinsics, depth_scale)
        T_2_to_1: Matriz homogénea 4x4 de transformación de Cam 2 a Cam 1
        voxel_length: Tamaño del voxel TSDF en metros (ej. 0.003 = 3 mm)
        sdf_trunc: Distancia de truncamiento de la función con signo (ej. 0.015 = 15 mm)
        depth_trunc: Distancia máxima de profundidad a integrar en metros
        
    Returns:
        mesh: open3d.geometry.TriangleMesh con normales y colores de vértices calculados
        pcd_tsdf: open3d.geometry.PointCloud extraída del volumen TSDF
    """
    volume = o3d.pipelines.integration.ScalableTSDFVolume(
        voxel_length=float(voxel_length),
        sdf_trunc=float(sdf_trunc),
        color_type=o3d.pipelines.integration.TSDFVolumeColorType.RGB8
    )

    def _create_rgbd(snap):
        col = o3d.geometry.Image(snap["color"])
        dep = o3d.geometry.Image(snap["depth"])
        intr = snap["intrinsics"]
        pinhole = o3d.camera.PinholeCameraIntrinsic(
            int(intr["width"]), int(intr["height"]),
            float(intr["fx"]), float(intr["fy"]),
            float(intr["ppx"]), float(intr["ppy"])
        )
        scale = float(snap.get("depth_scale", 0.001))
        rgbd = o3d.geometry.RGBDImage.create_from_color_and_depth(
            col, dep, depth_scale=1.0 / scale, depth_trunc=float(depth_trunc), convert_rgb_to_intensity=False
        )
        return rgbd, pinhole

    rgbd1, pinhole1 = _create_rgbd(snap1)
    rgbd2, pinhole2 = _create_rgbd(snap2)

    # Cámara 1 es el sistema de referencia origen (World) -> extrínseco identidad
    extrinsic1 = np.eye(4, dtype=np.float64)

    # Cámara 2: Open3D requiere matriz World -> Camera
    # Como X_cam1 = T_2_to_1 * X_cam2, entonces X_cam2 = inv(T_2_to_1) * X_cam1
    T_2_to_1_mat = np.asarray(T_2_to_1, dtype=np.float64)
    extrinsic2 = np.linalg.inv(T_2_to_1_mat)

    volume.integrate(rgbd1, pinhole1, extrinsic1)
    volume.integrate(rgbd2, pinhole2, extrinsic2)

    mesh = volume.extract_triangle_mesh()
    if len(mesh.vertices) > 0:
        mesh.compute_vertex_normals()

    pcd_tsdf = volume.extract_point_cloud()
    return mesh, pcd_tsdf


def create_overlap_diagnostic_pcd(pcd1, pcd2, T_2_to_1):
    """
    Crea una nube de diagnóstico visual tiñendo Cam 1 de Cian y Cam 2 de Magenta.
    Al superponerse con calibración correcta, los puntos se entrelazan armónicamente
    sin desfasaje espacial visible (referencia: LiveScan3D).
    """
    import copy
    p1 = copy.deepcopy(pcd1)
    p2 = copy.deepcopy(pcd2)
    p2.transform(np.asarray(T_2_to_1, dtype=np.float64))

    # Cian para Cámara 1: [0.0, 0.95, 0.95]
    p1.paint_uniform_color([0.0, 0.95, 0.95])
    # Magenta para Cámara 2: [0.95, 0.1, 0.85]
    p2.paint_uniform_color([0.95, 0.1, 0.85])

    return p1 + p2

