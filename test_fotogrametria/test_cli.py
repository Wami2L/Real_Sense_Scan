"""
CLI de alineación Cam 2 -> Cam 1 con el motor de live3d/alignment.py.

Uso:
    python test_fotogrametria/test_cli.py                     # cuadros en vivo del servidor 5002
    python test_fotogrametria/test_cli.py --apply             # y aplica el resultado si es fiable
    python test_fotogrametria/test_cli.py --save-dir Saves/2026-10-16/dual_scan_152833
    python test_fotogrametria/test_cli.py --all-saves         # evalúa todos los escaneos duales guardados
"""

import argparse
import asyncio
import glob
import json
import os
import sys
import time
import urllib.request

import cv2
import numpy as np

BASE_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if BASE_DIR not in sys.path:
    sys.path.insert(0, BASE_DIR)

from live3d import protocol  # noqa: E402
from live3d.alignment import RGBDView, align_pair, draw_matches, pose_delta, public_result  # noqa: E402


async def grab_live_views(server_url="http://localhost:5002/ws", timeout=5.0):
    """Primer par de cuadros (uno por cámara) del stream /ws, como vistas RGB-D."""
    import aiohttp
    metas, frames = {}, {}
    async with aiohttp.ClientSession() as session:
        async with session.ws_connect(server_url, max_msg_size=0) as ws:
            await ws.send_str(json.dumps({"type": "subscribe", "client": "cli", "depth": "zlib",
                                          "color": "jpeg", "ack": False}))
            t_end = time.time() + timeout
            while time.time() < t_end and not (0 in frames and 1 in frames):
                msg = await ws.receive(timeout=t_end - time.time())
                if msg.type == aiohttp.WSMsgType.TEXT:
                    data = json.loads(msg.data)
                    if data.get("type") == "meta":
                        metas[data["meta_id"]] = data
                elif msg.type == aiohttp.WSMsgType.BINARY:
                    h = protocol.parse_header(msg.data)
                    meta = metas.get(h["meta_id"])
                    if meta:
                        frames[h["flags"] & 1] = (protocol.decode_frame(msg.data, meta), meta)
    if not (0 in frames and 1 in frames):
        raise RuntimeError("No llegaron cuadros de ambas cámaras (¿servidor en modo dual?)")
    views = []
    for i in (0, 1):
        fr, meta = frames[i]
        img = cv2.imdecode(np.frombuffer(fr["color"], np.uint8), cv2.IMREAD_COLOR)
        views.append(RGBDView.from_meta(img, fr["depth"].copy(), meta, fr["timestamp"], f"Cam {i + 1}"))
    return views


def load_saved_views(folder_path):
    """Vistas desde un escaneo dual guardado (profundidad reconstruida desde nube_camN.ply)."""
    import open3d as o3d
    with open(os.path.join(folder_path, "metadata_dual.json"), "r", encoding="utf-8") as f:
        meta = json.load(f)
    views = []
    for i, key in enumerate(("cam1", "cam2")):
        intr = meta["cameras"][key]["intrinsics"]
        img = cv2.imread(os.path.join(folder_path, f"{key}_color.jpg"))
        pts = np.asarray(o3d.io.read_point_cloud(os.path.join(folder_path, f"nube_{key}.ply")).points)
        depth = np.zeros((intr["height"], intr["width"]), np.uint16)
        if len(pts):
            z = pts[:, 2]
            u = np.round(pts[:, 0] / z * intr["fx"] + intr["ppx"]).astype(int)
            v = np.round(pts[:, 1] / z * intr["fy"] + intr["ppy"]).astype(int)
            ok = (u >= 0) & (u < intr["width"]) & (v >= 0) & (v < intr["height"]) & (z > 0)
            depth[v[ok], u[ok]] = (z[ok] * 1000.0).astype(np.uint16)
        views.append(RGBDView(img, depth, intr, 0.001, label=f"Cam {i + 1}"))
    return views, np.asarray(meta["extrinsics_cam2_to_cam1"])


def apply_extrinsics_to_live_viewer(matrix_4x4, server_api="http://localhost:5002/api/dual/extrinsics",
                                    calibration=None):
    """Envía la matriz al servidor 5002, que la aplica y la guarda."""
    payload = json.dumps({"matrix_4x4": matrix_4x4, "calibration": calibration or {}}).encode("utf-8")
    req = urllib.request.Request(server_api, data=payload, headers={"Content-Type": "application/json"},
                                 method="POST")
    try:
        with urllib.request.urlopen(req, timeout=3.0) as resp:
            return json.loads(resp.read().decode("utf-8")).get("status") == "success"
    except Exception as e:  # noqa: BLE001
        print(f"  [AVISO] No se pudo aplicar en 5002: {e}")
        return False


def print_result(res, reference=None):
    m = res.get("metrics", {})
    print(f"  {res['quality'].upper():<12} {res['message']}")
    print(f"  SIFT: {m.get('matches', 0)} pares · {m.get('lifted', 0)} con profundidad · "
          f"{m.get('inliers', 0)} inliers 3D ({m.get('inlier_ratio_pct', 0)} %) · dt {m.get('dt_ms')} ms · "
          f"{res['total_time_ms']:.0f} ms")
    for c in res.get("candidates", []):
        print(f"    - {c['name']:<26} consistencia {c['consistency_pct']:>5} %  comunes {c['overlap_points']:>6} "
              f"({c['overlap_frac_pct']} %) ocultos {c['hidden_points']:>6}  RMSE {c['rmse_mm']} mm  "
              f"acuerdo SIFT {c['feature_agreement_pct']} %")
    for w in res.get("warnings", []):
        print(f"  ! {w}")
    if res.get("components"):
        t, r = res["components"]["translation_cm"], res["components"]["rotation_deg"]
        print(f"  T(2->1): t = ({t['x']:+.1f}, {t['y']:+.1f}, {t['z']:+.1f}) cm · "
              f"yaw {r['yaw']:+.1f}° pitch {r['pitch']:+.1f}° roll {r['roll']:+.1f}°")
        if reference is not None:
            d = pose_delta(reference, res["matrix_4x4"])
            print(f"  Diferencia con la calibración usada en la toma: {d['translation_cm']} cm · {d['rotation_deg']}°")


def main():
    p = argparse.ArgumentParser(description="Alineación Cam 2 -> Cam 1 por fotogrametría RGB-D")
    p.add_argument("--save-dir", default="", help="carpeta de un escaneo dual guardado")
    p.add_argument("--all-saves", action="store_true", help="evaluar todos los Saves/*/dual_scan_*")
    p.add_argument("--apply", action="store_true", help="aplicar en 5002 si el resultado es fiable")
    p.add_argument("--no-prior", action="store_true", help="no usar la calibración guardada como semilla")
    args = p.parse_args()

    if args.all_saves:
        folders = sorted(glob.glob(os.path.join(BASE_DIR, "Saves", "*", "dual_scan_*")))
        for folder in folders:
            if not os.path.isfile(os.path.join(folder, "metadata_dual.json")):
                continue
            print(f"\n== {os.path.relpath(folder, BASE_DIR)}")
            (v1, v2), ref = load_saved_views(folder)
            print_result(align_pair(v1, v2, prior_T=None if args.no_prior else ref), ref)
        return

    ref = None
    if args.save_dir:
        (v1, v2), ref = load_saved_views(args.save_dir)
    else:
        v1, v2 = asyncio.run(grab_live_views())
    res = align_pair(v1, v2, prior_T=None if args.no_prior else ref)
    print_result(res, ref)
    out = os.path.join(os.path.dirname(os.path.abspath(__file__)), "resultado_matches.jpg")
    cv2.imwrite(out, draw_matches(v1, v2, res))
    print(f"  Diagnóstico: {out}")
    if args.apply and res["success"]:
        info = {k: res["metrics"].get(k) for k in ("consistency_pct", "rmse_mm", "overlap_points", "dt_ms")}
        ok = apply_extrinsics_to_live_viewer(res["matrix_4x4"], calibration={"method": res["method"],
                                                                             "quality": res["quality"], **info})
        print("  Aplicada en 5002." if ok else "  No se aplicó.")
    elif args.apply:
        print("  No se aplica: el resultado no es fiable.")
    return public_result(res)


if __name__ == "__main__":
    main()
