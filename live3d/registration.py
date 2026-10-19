"""
Módulo de Registro y Calibración Extrínseca Automática sin Tablero para Dual RealSense.
Utiliza Open3D con estimación multiescala Point-to-Plane y Colored ICP para calcular
la matriz rígida homogénea 4x4 T(cam2 -> cam1) a partir de la geometría común en escena.
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


def _prep(pcd, voxel):
    p = pcd.voxel_down_sample(voxel)
    p.estimate_normals(o3d.geometry.KDTreeSearchParamHybrid(radius=voxel * 2.5, max_nn=30))
    p.orient_normals_towards_camera_location(np.zeros(3))
    return p


def _global_ransac(p1, p2, voxel):
    """Registro global sin estimación inicial: FPFH + RANSAC (patrón estándar de Open3D)."""
    reg = o3d.pipelines.registration
    feat = o3d.geometry.KDTreeSearchParamHybrid(radius=voxel * 5, max_nn=100)
    f1 = reg.compute_fpfh_feature(p1, feat)
    f2 = reg.compute_fpfh_feature(p2, feat)
    dist = voxel * 1.5
    res = reg.registration_ransac_based_on_feature_matching(
        p2, p1, f2, f1, True, dist,
        reg.TransformationEstimationPointToPoint(False), 3,
        [reg.CorrespondenceCheckerBasedOnEdgeLength(0.9),
         reg.CorrespondenceCheckerBasedOnDistance(dist)],
        reg.RANSACConvergenceCriteria(100000, 0.999),
    )
    return np.asarray(res.transformation)


def _icp_cascade(pcd1, pcd2, T, voxel_stages, use_colored_icp):
    reg = o3d.pipelines.registration
    T = np.asarray(T, dtype=np.float64)
    for i, v in enumerate(voxel_stages):
        p1, p2 = _prep(pcd1, v), _prep(pcd2, v)
        crit = reg.ICPConvergenceCriteria(relative_fitness=1e-6, relative_rmse=1e-6, max_iteration=50)
        last = i == len(voxel_stages) - 1
        try:
            if last and use_colored_icp and p1.has_colors() and p2.has_colors():
                r = reg.registration_colored_icp(p2, p1, v * 1.5, T, reg.TransformationEstimationForColoredICP(), crit)
            else:
                r = reg.registration_icp(p2, p1, v * 2.0, T, reg.TransformationEstimationPointToPlane(), crit)
            if r.fitness > 0:
                T = r.transformation
        except RuntimeError:
            pass
    return np.asarray(T)


def register_dual_point_clouds(
    pcd1,
    pcd2,
    init_T=None,
    voxel_stages=(0.03, 0.012, 0.006),
    use_colored_icp=True,
    eval_dist=0.01,
    min_fitness=0.20,
    max_rmse_mm=18.0,
):
    """
    Registra pcd2 (Cam 2) sobre pcd1 (Cam 1) y devuelve T(cam2 -> cam1).

    Candidatos evaluados con la misma métrica (fitness = fracción de puntos de Cam 2
    con pareja en Cam 1 a < eval_dist tras alinear):
      1. ICP en cascada desde la calibración actual (init_T), si existe.
      2. FPFH + RANSAC global (no depende de ninguna estimación previa) + ICP en cascada.
    Se elige el de mayor fitness; solo se marca success si supera los umbrales.
    """
    if len(pcd1.points) < 1000 or len(pcd2.points) < 1000:
        raise ValueError("Puntos insuficientes en alguna de las cámaras para registrar la escena.")

    reg = o3d.pipelines.registration
    p1_eval, p2_eval = _prep(pcd1, 0.005), _prep(pcd2, 0.005)

    candidates = []
    if init_T is not None:
        T = _icp_cascade(pcd1, pcd2, init_T, voxel_stages, use_colored_icp)
        candidates.append(("calibración actual + ICP", T))

    # Candidato de barrido sobre el sujeto en primer plano (centrado en la persona, z in [0.3, 1.4] m)
    try:
        pts1 = np.asarray(pcd1.points)
        pts2 = np.asarray(pcd2.points)
        m1 = (pts1[:, 2] >= 0.3) & (pts1[:, 2] <= 1.35) & (np.abs(pts1[:, 0]) <= 0.65)
        m2 = (pts2[:, 2] >= 0.3) & (pts2[:, 2] <= 1.35) & (np.abs(pts2[:, 0]) <= 0.65)
        if np.sum(m1) > 2000 and np.sum(m2) > 2000:
            fg1 = pcd1.select_by_index(np.where(m1)[0]).voxel_down_sample(0.008)
            fg2 = pcd2.select_by_index(np.where(m2)[0]).voxel_down_sample(0.008)
            fg1.estimate_normals(o3d.geometry.KDTreeSearchParamHybrid(radius=0.03, max_nn=30))
            fg2.estimate_normals(o3d.geometry.KDTreeSearchParamHybrid(radius=0.03, max_nn=30))

            c1 = np.mean(fg1.points, axis=0)
            c2 = np.mean(fg2.points, axis=0)

            best_sweep_fit = 0
            best_sweep_T = None
            for yaw_deg in np.linspace(-70, 70, 29):
                for pitch_deg in (-15, -7.5, 0, 7.5, 15):
                    y = np.radians(yaw_deg)
                    p = np.radians(pitch_deg)
                    Ry = np.array([[np.cos(y), 0, np.sin(y)], [0, 1, 0], [-np.sin(y), 0, np.cos(y)]])
                    Rx = np.array([[1, 0, 0], [0, np.cos(p), -np.sin(p)], [0, np.sin(p), np.cos(p)]])
                    R = Ry @ Rx
                    t = c1 - R @ c2
                    T_cand = np.eye(4)
                    T_cand[:3, :3] = R
                    T_cand[:3, 3] = t
                    ev = reg.evaluate_registration(fg2, fg1, 0.04, T_cand)
                    if ev.fitness > best_sweep_fit:
                        best_sweep_fit = ev.fitness
                        best_sweep_T = T_cand

            if best_sweep_T is not None:
                T_fg = _icp_cascade(fg1, fg2, best_sweep_T, (0.02, 0.01, 0.006), use_colored_icp)
                candidates.append(("barrido sujeto primer plano + ICP", T_fg))
    except Exception as e:
        print(f"[Registro] Barrido de sujeto falló: {e}")

    try:
        gv = voxel_stages[1]
        T0 = _global_ransac(_prep(pcd1, gv), _prep(pcd2, gv), gv)
        T = _icp_cascade(pcd1, pcd2, T0, voxel_stages[1:], use_colored_icp)
        candidates.append(("FPFH+RANSAC + ICP", T))
    except Exception as e:  # noqa: BLE001
        print(f"[Registro] RANSAC global falló: {e}")

    best = None
    for name, T in candidates:
        ev = reg.evaluate_registration(p2_eval, p1_eval, eval_dist, T)
        item = (float(ev.fitness), float(ev.inlier_rmse), name, T)
        print(f"[Registro] {name}: fitness={item[0]:.3f} rmse={item[1]*1000:.2f} mm")
        if best is None or item[0] > best[0]:
            best = item

    if best is None:
        raise RuntimeError("No se pudo calcular ningún candidato de alineación.")

    fitness, rmse, name, T = best
    rmse_mm = round(rmse * 1000.0, 2)
    success = fitness >= min_fitness and rmse_mm <= max_rmse_mm
    return {
        "success": success,
        "matrix_4x4": np.asarray(T).tolist(),
        "fitness": round(fitness, 4),
        "inlier_rmse_mm": rmse_mm,
        "method": name,
        "message": (f"Alineación {'exitosa' if success else 'con baja convergencia'} [{name}] "
                    f"(solape {round(fitness * 100, 1)}%, error {rmse_mm} mm)"),
    }


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


def fuse_dual_point_cloud(snap1, snap2, T_2_to_1, voxel=0.0025, depth_trunc=2.5):
    """Nube única de ambas cámaras en el marco de Cam 1, deduplicada por vóxel (sensor virtual)."""
    pcd1 = snap_to_pcd(snap1, depth_trunc=depth_trunc)
    pcd2 = snap_to_pcd(snap2, depth_trunc=depth_trunc)
    pcd2.transform(np.asarray(T_2_to_1, dtype=np.float64))
    fused = (pcd1 + pcd2).voxel_down_sample(voxel)
    if len(fused.points) > 100:
        fused, _ = fused.remove_statistical_outlier(nb_neighbors=20, std_ratio=2.0)
    return fused


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

