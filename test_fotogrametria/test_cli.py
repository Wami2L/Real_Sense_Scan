"""
Script CLI para probar la Calibración por Fotogrametría RGB-D en Tiempo Real.
Entorno de prueba aislado (NO modifica archivos de live3d).

Uso:
    python test_fotogrametria/test_cli.py
    python test_fotogrametria/test_cli.py --apply
    python test_fotogrametria/test_cli.py --save-dir Saves/2026-10-16/dual_scan_152833
"""

import os
import sys
import argparse
import asyncio
import json
import time
import urllib.request
import numpy as np
import cv2

# Asegurar path para imports locales
BASE_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if BASE_DIR not in sys.path:
    sys.path.insert(0, BASE_DIR)

from test_fotogrametria.calibrador import RGBDPhotogrammetryCalibrator
from live3d import protocol


async def grab_live_frames(server_url="http://localhost:5002/ws"):
    """Se conecta al WebSocket de Live 3D para capturar un par de cuadros sincronizados sin bloquear el servidor."""
    import aiohttp
    metas = {}
    frames = {}

    print(f"[Conexión] Obteniendo par de cuadros sincronizados desde {server_url} ...")
    async with aiohttp.ClientSession() as session:
        async with session.ws_connect(server_url) as ws:
            await ws.send_str(json.dumps({"type": "subscribe", "depth": "zlib", "color": "jpeg", "ack": False}))
            
            start_wait = time.time()
            async for msg in ws:
                if time.time() - start_wait > 5.0:
                    break
                if msg.type == aiohttp.WSMsgType.TEXT:
                    data = json.loads(msg.data)
                    if data.get("type") == "meta":
                        metas[data.get("meta_id")] = data
                elif msg.type == aiohttp.WSMsgType.BINARY:
                    h = protocol.parse_header(msg.data)
                    cam_idx = h["flags"] & 0x01
                    meta = metas.get(h["meta_id"])
                    if meta:
                        try:
                            decoded = protocol.decode_frame(msg.data, meta)
                            frames[cam_idx] = (decoded, meta)
                            if 0 in frames and 1 in frames:
                                break
                        except Exception as e:
                            pass

    if 0 not in frames or 1 not in frames:
        raise RuntimeError("No se pudieron recibir ambos cuadros en el tiempo límite.")

    f0, m0 = frames[0]
    f1, m1 = frames[1]

    img1 = cv2.imdecode(np.frombuffer(f0["color"], np.uint8), cv2.IMREAD_COLOR)
    img2 = cv2.imdecode(np.frombuffer(f1["color"], np.uint8), cv2.IMREAD_COLOR)
    depth1 = f0["depth"]
    depth2 = f1["depth"]

    intr1 = {
        "fx": m0["color"]["fx"], "fy": m0["color"]["fy"],
        "ppx": m0["color"]["ppx"], "ppy": m0["color"]["ppy"]
    }
    intr2 = {
        "fx": m1["color"]["fx"], "fy": m1["color"]["fy"],
        "ppx": m1["color"]["ppx"], "ppy": m1["color"]["ppy"]
    }
    scale1 = float(m0["depth"]["scale"])
    scale2 = float(m1["depth"]["scale"])

    return img1, depth1, intr1, scale1, img2, depth2, intr2, scale2


def load_saved_frames(folder_path):
    """Carga cuadros y metadatos de un escaneo guardado previo."""
    print(f"[Offline] Cargando cuadros guardados desde {folder_path} ...")
    meta_path = os.path.join(folder_path, "metadata_dual.json")
    if not os.path.isfile(meta_path):
        raise FileNotFoundError(f"No existe metadata_dual.json en {folder_path}")

    with open(meta_path, "r", encoding="utf-8") as f:
        meta = json.load(f)

    img1_path = os.path.join(folder_path, "cam1_color.jpg")
    img2_path = os.path.join(folder_path, "cam2_color.jpg")
    img1 = cv2.imread(img1_path)
    img2 = cv2.imread(img2_path)

    # Reconstruir mapas de profundidad desde nubes PLY si no hay mapa crudo
    import open3d as o3d
    pcd1_path = os.path.join(folder_path, "nube_cam1.ply")
    pcd2_path = os.path.join(folder_path, "nube_cam2.ply")
    pcd1 = o3d.io.read_point_cloud(pcd1_path)
    pcd2 = o3d.io.read_point_cloud(pcd2_path)

    intr1 = meta["cameras"]["cam1"]["intrinsics"]
    intr2 = meta["cameras"]["cam2"]["intrinsics"]

    # Crear mapa de profundidad proyectando la nube
    w1, h1 = intr1["width"], intr1["height"]
    depth1 = np.zeros((h1, w1), dtype=np.uint16)
    pts1 = np.asarray(pcd1.points)
    if len(pts1) > 0:
        z = pts1[:, 2]
        u = np.round(pts1[:, 0] / z * intr1["fx"] + intr1["ppx"]).astype(int)
        v = np.round(pts1[:, 1] / z * intr1["fy"] + intr1["ppy"]).astype(int)
        valid = (u >= 0) & (u < w1) & (v >= 0) & (v < h1) & (z > 0)
        depth1[v[valid], u[valid]] = (z[valid] * 1000.0).astype(np.uint16)

    w2, h2 = intr2["width"], intr2["height"]
    depth2 = np.zeros((h2, w2), dtype=np.uint16)
    pts2 = np.asarray(pcd2.points)
    if len(pts2) > 0:
        z = pts2[:, 2]
        u = np.round(pts2[:, 0] / z * intr2["fx"] + intr2["ppx"]).astype(int)
        v = np.round(pts2[:, 1] / z * intr2["fy"] + intr2["ppy"]).astype(int)
        valid = (u >= 0) & (u < w2) & (v >= 0) & (v < h2) & (z > 0)
        depth2[v[valid], u[valid]] = (z[valid] * 1000.0).astype(np.uint16)

    return img1, depth1, intr1, 0.001, img2, depth2, intr2, 0.001, pcd1, pcd2


def apply_extrinsics_to_live_viewer(matrix_4x4, server_api="http://localhost:5002/api/dual/extrinsics"):
    """Envía la matriz calibrada al servidor en ejecución en el puerto 5002."""
    print(f"\n[Live 3D] Enviando matriz calibrada a {server_api} ...")
    payload = json.dumps({"matrix_4x4": matrix_4x4}).encode("utf-8")
    req = urllib.request.Request(server_api, data=payload, headers={"Content-Type": "application/json"}, method="POST")
    try:
        with urllib.request.urlopen(req, timeout=3.0) as resp:
            data = json.loads(resp.read().decode("utf-8"))
            print(f"  [OK] Servidor actualizó las extrínsecas en caliente: {data.get('status')}")
            return True
    except Exception as e:
        print(f"  [AVISO] No se pudo conectar con el servidor: {e}")
        return False


def main():
    parser = argparse.ArgumentParser(description="Calibración Fotogramétrica RGB-D en Tiempo Real")
    parser.add_argument("--save-dir", type=str, default="", help="Ruta a una carpeta de escaneo guardado (offline)")
    parser.add_argument("--apply", action="store_true", help="Aplica automáticamente la calibración al visor Live 3D")
    parser.add_argument("--method", type=str, default="SIFT", choices=["SIFT", "ORB"], help="Extractor de características")
    parser.add_argument("--features", type=int, default=2000, help="Número de características por cuadro")
    args = parser.parse_args()

    print("=" * 70)
    print("  ENTORNO DE PRUEBA: CALIBRACIÓN POR FOTOGRAMETRÍA RGB-D")
    print("=" * 70)

    pcd1, pcd2 = None, None
    if args.save_dir:
        img1, depth1, intr1, s1, img2, depth2, intr2, s2, pcd1, pcd2 = load_saved_frames(args.save_dir)
    else:
        try:
            img1, depth1, intr1, s1, img2, depth2, intr2, s2 = asyncio.run(grab_live_frames())
        except Exception as e:
            print(f"\n[ERROR] Falló la captura en vivo: {e}")
            print("Consejo: Asegúrate de que app.py esté corriendo en el puerto 5002.")
            print("También puedes probar en modo offline con: --save-dir Saves/2026-10-16/dual_scan_152833")
            return

        calibrator = RGBDPhotogrammetryCalibrator(method=args.method, max_features=args.features)
        out_dir = os.path.dirname(os.path.abspath(__file__))
        cv2.imwrite(os.path.join(out_dir, "live_cam1.jpg"), img1)
        cv2.imwrite(os.path.join(out_dir, "live_cam2.jpg"), img2)
        print(f"[Captura] Cuadros actuales guardados en test_fotogrametria/live_cam1.jpg y live_cam2.jpg")
        print(f"\n[Procesando] Ejecutando matching fotogramétrico ({args.method}) + SVD de Kabsch + RANSAC...")

    res = calibrator.calibrate(
        img1_bgr=img1, depth1_u16=depth1, intr1=intr1,
        img2_bgr=img2, depth2_u16=depth2, intr2=intr2,
        scale1=s1, scale2=s2, do_icp=True, pcd1=pcd1, pcd2=pcd2
    )

    if not res["success"]:
        print(f"\n[FALLO] {res['message']}")
        return

    # Imprimir métricas
    t = res["components"]["translation_cm"]
    r = res["components"]["rotation_deg"]

    print("\n" + "-" * 70)
    print("  RESULTADOS DE CALIBRACIÓN FOTOGRAMÉTRICA")
    print("-" * 70)
    print(f"  Método:                 {res['method']}")
    print(f"  Tiempo de Detección:    {res['detect_time_ms']} ms")
    print(f"  Tiempo RANSAC + SVD:    {res['ransac_time_ms']} ms")
    print(f"  Tiempo Total:           {res['total_time_ms']} ms")
    print(f"  Features Cam 1 / Cam 2: {res['features_cam1']} / {res['features_cam2']}")
    print(f"  Correspondencias 3D:    {res['pts_with_depth']}")
    print(f"  Inliers Geométricos:    {res['inliers_count']} ({res['inlier_ratio_pct']}%)")
    print(f"  Error Residual (RMSE):  {res['rmse_mm']} mm")
    print("-" * 70)
    print("  POSICIÓN RELATIVA DE CÁMARA 2 RESPECTO A CÁMARA 1:")
    print(f"    Traslación X: {t['x']:+7.2f} cm  (lateral: negativo = izquierda)")
    print(f"    Traslación Y: {t['y']:+7.2f} cm  (vertical)")
    print(f"    Traslación Z: {t['z']:+7.2f} cm  (profundidad)")
    print(f"    Rotación Yaw: {r['yaw']:+7.2f}°  (ángulo horizontal de convergencia)")
    print(f"    Pitch:        {r['pitch']:+7.2f}°")
    print(f"    Roll:         {r['roll']:+7.2f}°")
    print("-" * 70)

    # Guardar imagen con correspondencias
    out_dir = os.path.dirname(os.path.abspath(__file__))
    out_img_path = os.path.join(out_dir, "resultado_matches.jpg")
    cv2.imwrite(out_img_path, res["debug_image"])
    print(f"\n[Guardado] Imagen visual con correspondencias guardada en:")
    print(f"  {out_img_path}")

    # Aplicar al visor si se solicitó
    if args.apply:
        apply_extrinsics_to_live_viewer(res["matrix_4x4"])
    else:
        print("\n[Tip] Para aplicar esta calibración inmediatamente al visor 3D, corre:")
        print("  python test_fotogrametria/test_cli.py --apply")


if __name__ == "__main__":
    main()
