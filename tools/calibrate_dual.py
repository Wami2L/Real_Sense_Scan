"""
Herramienta de Calibración Extrínseca Dual RealSense (ChArUco / Checkerboard).
Calcula la matriz rígida homogénea 4x4 T(cam2 -> cam1) para alinear ambas cámaras 3D.

Uso:
    python tools/calibrate_dual.py
"""

import json
import os
import sys
import time

import cv2
import numpy as np

BASE_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
CONFIG_PATH = os.path.join(BASE_DIR, "config", "dual_camera_config.json")


def create_charuco_board(squares_x=9, squares_y=6, square_length_m=0.030, marker_length_m=0.022):
    """Crea la definición del tablero ChArUco."""
    aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_5X5_100)
    board = cv2.aruco.CharucoBoard((squares_x, squares_y), square_length_m, marker_length_m, aruco_dict)
    detector = cv2.aruco.CharucoDetector(board)
    return board, detector


def calibrate_from_images(img1_rgb, img2_rgb, intr1, intr2, board_squares=(9, 6), square_m=0.030, marker_m=0.022):
    """
    Calcula T(cam2 -> cam1) a partir de un par de imágenes RGB simultáneas.
    Devuelve: (T_2_to_1, rmse_error, debug_img1, debug_img2)
    """
    board, detector = create_charuco_board(board_squares[0], board_squares[1], square_m, marker_m)

    gray1 = cv2.cvtColor(img1_rgb, cv2.COLOR_RGB2GRAY)
    gray2 = cv2.cvtColor(img2_rgb, cv2.COLOR_RGB2GRAY)

    corners1, ids1, _, _ = detector.detectBoard(gray1)
    corners2, ids2, _, _ = detector.detectBoard(gray2)

    if corners1 is None or len(corners1) < 6:
        raise ValueError(f"Cámara 1: No se detectaron suficientes esquinas ChArUco ({0 if corners1 is None else len(corners1)} < 6).")
    if corners2 is None or len(corners2) < 6:
        raise ValueError(f"Cámara 2: No se detectaron suficientes esquinas ChArUco ({0 if corners2 is None else len(corners2)} < 6).")

    K1 = np.array([[intr1["fx"], 0, intr1["ppx"]], [0, intr1["fy"], intr1["ppy"]], [0, 0, 1]], dtype=np.float64)
    K2 = np.array([[intr2["fx"], 0, intr2["ppx"]], [0, intr2["fy"], intr2["ppy"]], [0, 0, 1]], dtype=np.float64)
    dist1 = np.zeros(5, dtype=np.float64)
    dist2 = np.zeros(5, dtype=np.float64)

    # Pose 1: Tablero respecto a Cámara 1
    obj_pts1, img_pts1 = board.matchImagePoints(corners1, ids1)
    ok1, rvec1, tvec1 = cv2.solvePnP(obj_pts1, img_pts1, K1, dist1, flags=cv2.SOLVEPNP_ITERATIVE)
    if not ok1:
        raise RuntimeError("Fallo al resolver pose PnP en Cámara 1")
    R1, _ = cv2.Rodrigues(rvec1)
    T1 = np.eye(4, dtype=np.float64)
    T1[:3, :3] = R1
    T1[:3, 3] = tvec1.squeeze()

    # Pose 2: Tablero respecto a Cámara 2
    obj_pts2, img_pts2 = board.matchImagePoints(corners2, ids2)
    ok2, rvec2, tvec2 = cv2.solvePnP(obj_pts2, img_pts2, K2, dist2, flags=cv2.SOLVEPNP_ITERATIVE)
    if not ok2:
        raise RuntimeError("Fallo al resolver pose PnP en Cámara 2")
    R2, _ = cv2.Rodrigues(rvec2)
    T2 = np.eye(4, dtype=np.float64)
    T2[:3, :3] = R2
    T2[:3, 3] = tvec2.squeeze()

    # Transformación relativa T(cam2 -> cam1) = T1 * inv(T2)
    T_2_to_1 = T1 @ np.linalg.inv(T2)

    # Calcular error de reproyección (RMSE)
    proj1, _ = cv2.projectPoints(obj_pts1, rvec1, tvec1, K1, dist1)
    err1 = np.linalg.norm(img_pts1.squeeze() - proj1.squeeze(), axis=1).mean()

    proj2, _ = cv2.projectPoints(obj_pts2, rvec2, tvec2, K2, dist2)
    err2 = np.linalg.norm(img_pts2.squeeze() - proj2.squeeze(), axis=1).mean()
    rmse_px = round((err1 + err2) / 2.0, 2)

    # Imágenes de depuración con esquinas dibujadas
    dbg1 = cv2.cvtColor(gray1, cv2.COLOR_GRAY2BGR)
    cv2.aruco.drawDetectedCornersCharuco(dbg1, corners1, ids1)
    cv2.drawFrameAxes(dbg1, K1, dist1, rvec1, tvec1, 0.08)

    dbg2 = cv2.cvtColor(gray2, cv2.COLOR_GRAY2BGR)
    cv2.aruco.drawDetectedCornersCharuco(dbg2, corners2, ids2)
    cv2.drawFrameAxes(dbg2, K2, dist2, rvec2, tvec2, 0.08)

    return T_2_to_1, rmse_px, dbg1, dbg2


def save_extrinsics_to_config(T_matrix, rmse_px=None, config_path=CONFIG_PATH):
    """Guarda la matriz en config/dual_camera_config.json."""
    data = {}
    if os.path.isfile(config_path):
        try:
            with open(config_path, "r", encoding="utf-8") as f:
                data = json.load(f)
        except Exception:
            pass

    if "extrinsics_cam2_to_cam1" not in data:
        data["extrinsics_cam2_to_cam1"] = {}

    data["extrinsics_cam2_to_cam1"]["is_calibrated"] = True
    data["extrinsics_cam2_to_cam1"]["calibration_date"] = time.strftime("%Y-%m-%d %H:%M:%S")
    data["extrinsics_cam2_to_cam1"]["rmse_px"] = rmse_px
    data["extrinsics_cam2_to_cam1"]["matrix_4x4"] = np.round(T_matrix, 6).tolist()

    os.makedirs(os.path.dirname(config_path), exist_ok=True)
    with open(config_path, "w", encoding="utf-8") as f:
        json.dump(data, f, indent=4)
    print(f"[Calibración] Matriz extrínseca guardada exitosamente en {config_path}")


def run_standalone():
    """Ejecución interactiva desde terminal."""
    import pyrealsense2 as rs

    ctx = rs.context()
    devs = list(ctx.query_devices())
    if len(devs) < 2:
        print("[ERROR] Se necesitan al menos 2 cámaras RealSense conectadas.")
        sys.exit(1)

    s1 = devs[0].get_info(rs.camera_info.serial_number)
    s2 = devs[1].get_info(rs.camera_info.serial_number)
    print(f"Iniciando calibración entre Cam 1 ({s1}) y Cam 2 ({s2})...")
    print("Coloca el tablero ChArUco de forma que sea visible por ambas cámaras.")

    p1, p2 = rs.pipeline(ctx), rs.pipeline(ctx)
    c1, c2 = rs.config(), rs.config()
    c1.enable_device(s1)
    c2.enable_device(s2)
    for c in (c1, c2):
        c.enable_stream(rs.stream.color, 848, 480, rs.format.rgb8, 30)

    p1.start(c1)
    p2.start(c2)

    try:
        print("Esperando 3 segundos para estabilización de autoexposición...")
        for _ in range(30):
            p1.wait_for_frames()
            p2.wait_for_frames()

        f1 = p1.wait_for_frames(5000)
        f2 = p2.wait_for_frames(5000)

        color1 = np.asanyarray(f1.get_color_frame().get_data())
        color2 = np.asanyarray(f2.get_color_frame().get_data())

        ci1 = f1.get_color_frame().profile.as_video_stream_profile().get_intrinsics()
        ci2 = f2.get_color_frame().profile.as_video_stream_profile().get_intrinsics()

        intr1 = {"width": ci1.width, "height": ci1.height, "fx": ci1.fx, "fy": ci1.fy, "ppx": ci1.ppx, "ppy": ci1.ppy}
        intr2 = {"width": ci2.width, "height": ci2.height, "fx": ci2.fx, "fy": ci2.fy, "ppx": ci2.ppx, "ppy": ci2.ppy}

        T, rmse, dbg1, dbg2 = calibrate_from_images(color1, color2, intr1, intr2)

        print("\n¡Calibración exitosa!")
        print(f"Error medio de reproyección: {rmse} px")
        print("Matriz T(cam2 -> cam1):")
        print(T)

        save_extrinsics_to_config(T, rmse)

        out_dbg = os.path.join(BASE_DIR, "config", "calibration_debug.jpg")
        combo = np.hstack([dbg1, dbg2])
        cv2.imwrite(out_dbg, combo)
        print(f"Imagen de verificación guardada en: {out_dbg}")

    finally:
        p1.stop()
        p2.stop()


if __name__ == "__main__":
    run_standalone()
