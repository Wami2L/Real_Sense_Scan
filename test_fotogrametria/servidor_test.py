"""
Servidor Web de Prueba para Alineación Virtual RGB-D en Tiempo Real.
Puerto: 5005 (Totalmente independiente, NO modifica ningún archivo de live3d ni app.py).

Características:
- Receptor persistente WebSocket con el servidor principal (puerto 5002).
- Proxy transparente /viewer3d para control instantáneo y en caliente del visor 3D Three.js (Mismo origen).
- Fusión 2D Superpuesta (Reproyección 3D de Cam 2 sobre Cam 1 con control de opacidad).
- Anaglifo 3D Estéreo (Rojo / Cian) y Modo de Nivel Epipolar Horizontal.
- Transmisión en vivo de ambas cámaras en tiempo real (MJPEG Stream).
- Capturas de instantáneas manuales personalizadas por el usuario con galería múltiple.
- Alineación Virtual multi-hipótesis + Point-to-Plane ICP sobre la captura elegida.
- Sliders manuales de ajuste fino a 60 Hz sincronizados en vivo con Three.js.
"""

import os
import sys
import json
import asyncio
import time
import urllib.request
from aiohttp import web
import aiohttp
import cv2
import numpy as np

BASE_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if BASE_DIR not in sys.path:
    sys.path.insert(0, BASE_DIR)

from test_fotogrametria.calibrador import (
    AlineadorVirtual,
    matrix_to_euler_translation,
    euler_translation_to_matrix
)
from test_fotogrametria.test_cli import apply_extrinsics_to_live_viewer
from live3d import protocol

PORT = 5005
STATIC_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), "web")
CAPTURES_DIR = os.path.join(STATIC_DIR, "captures")
LIVE3D_WEB_DIR = os.path.join(BASE_DIR, "live3d", "web")
os.makedirs(CAPTURES_DIR, exist_ok=True)


class LiveCameraReceiver:
    """Mantiene conexión WebSocket permanente con puerto 5002 para cuadros en vivo con latencia 0 ms."""

    def __init__(self, ws_url="http://localhost:5002/ws"):
        self.ws_url = ws_url
        self.metas = {}
        self.latest_frames = {}
        self.latest_preview_jpeg = None
        self.running = False
        self.fps = 0.0
        self.frame_count = 0
        self.last_update_time = time.time()

    async def run(self):
        self.running = True
        print(f"[Receiver] Conectando a {self.ws_url} ...")
        while self.running:
            try:
                async with aiohttp.ClientSession() as session:
                    async with session.ws_connect(self.ws_url, timeout=10.0) as ws:
                        await ws.send_str(json.dumps({
                            "type": "subscribe",
                            "depth": "zlib",
                            "color": "jpeg",
                            "ack": False
                        }))
                        print("[Receiver] Suscripción activa al stream dual de cámaras.")

                        async for msg in ws:
                            if not self.running:
                                break
                            if msg.type == aiohttp.WSMsgType.TEXT:
                                data = json.loads(msg.data)
                                if data.get("type") == "meta":
                                    self.metas[data.get("meta_id")] = data
                            elif msg.type == aiohttp.WSMsgType.BINARY:
                                h = protocol.parse_header(msg.data)
                                cam_idx = h["flags"] & 0x01
                                meta = self.metas.get(h["meta_id"])
                                if meta:
                                    try:
                                        decoded = protocol.decode_frame(msg.data, meta)
                                        self.latest_frames[cam_idx] = (decoded, meta)
                                        if 0 in self.latest_frames and 1 in self.latest_frames:
                                            self._update_preview()
                                            self.frame_count += 1
                                            now = time.time()
                                            if now - self.last_update_time >= 1.0:
                                                self.fps = self.frame_count / (now - self.last_update_time)
                                                self.frame_count = 0
                                                self.last_update_time = now
                                    except Exception:
                                        pass
            except Exception as e:
                print(f"[Receiver] Reconectando en 1.5s: {e}")
                await asyncio.sleep(1.5)

    def _update_preview(self):
        """Genera un cuadro side-by-side optimizado para el stream en vivo."""
        try:
            f0, _ = self.latest_frames[0]
            f1, _ = self.latest_frames[1]
            img1 = cv2.imdecode(np.frombuffer(f0["color"], np.uint8), cv2.IMREAD_COLOR)
            img2 = cv2.imdecode(np.frombuffer(f1["color"], np.uint8), cv2.IMREAD_COLOR)
            if img1 is None or img2 is None:
                return

            h, w = 320, 565
            im1_res = cv2.resize(img1, (w, h))
            im2_res = cv2.resize(img2, (w, h))

            cv2.rectangle(im1_res, (0, 0), (w, 32), (18, 18, 24), -1)
            cv2.circle(im1_res, (16, 16), 6, (0, 255, 120), -1)
            cv2.putText(im1_res, "CAM 1 (MASTER) - EN VIVO", (30, 22),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255, 255, 255), 1, cv2.LINE_AA)

            cv2.rectangle(im2_res, (0, 0), (w, 32), (18, 18, 24), -1)
            cv2.circle(im2_res, (16, 16), 6, (255, 200, 0), -1)
            cv2.putText(im2_res, "CAM 2 (SLAVE) - EN VIVO", (30, 22),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255, 255, 255), 1, cv2.LINE_AA)

            combined = np.hstack([im1_res, im2_res])
            cv2.line(combined, (w, 0), (w, h), (0, 200, 255), 2)

            _, buf = cv2.imencode('.jpg', combined, [cv2.IMWRITE_JPEG_QUALITY, 80])
            self.latest_preview_jpeg = buf.tobytes()
        except Exception:
            pass

    def get_instant_pair(self):
        """Retorna el par de imágenes y nubes de profundidad instantáneas actuales."""
        if 0 not in self.latest_frames or 1 not in self.latest_frames:
            return None

        f0, m0 = self.latest_frames[0]
        f1, m1 = self.latest_frames[1]

        img1 = cv2.imdecode(np.frombuffer(f0["color"], np.uint8), cv2.IMREAD_COLOR)
        img2 = cv2.imdecode(np.frombuffer(f1["color"], np.uint8), cv2.IMREAD_COLOR)
        depth1 = f0["depth"].copy()
        depth2 = f1["depth"].copy()

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

        return {
            "img1": img1, "depth1": depth1, "intr1": intr1, "scale1": scale1,
            "img2": img2, "depth2": depth2, "intr2": intr2, "scale2": scale2,
            "timestamp": time.strftime("%H:%M:%S")
        }


def compute_camera_alignment_views(img1_bgr, depth1_u16, intr1, scale1,
                                   img2_bgr, depth2_u16, intr2, scale2,
                                   T_matrix, alpha=0.5):
    """
    Reproyecta la Cámara 2 sobre la perspectiva de la Cámara 1 usando la matriz de calibración T(2->1).
    Genera vistas para que el alineado se aprecie directamente en el entorno de las cámaras:
    1. Overlay fusionado (Cam 1 + Cam 2 reproyectada al 50% de opacidad).
    2. Anaglifo estéreo 3D (Cam 1 en Rojo, Cam 2 en Cian).
    3. Vista lado a lado con líneas de nivel horizontal epipolares.
    """
    h1, w1 = img1_bgr.shape[:2]
    h2, w2 = img2_bgr.shape[:2]

    T = np.asarray(T_matrix, dtype=np.float64)
    R = T[:3, :3]
    t = T[:3, 3].reshape(3, 1)

    # Nube 3D densa de Cam 2
    u2 = np.arange(w2)
    v2 = np.arange(h2)
    uu2, vv2 = np.meshgrid(u2, v2)
    z2 = depth2_u16.astype(np.float64) * scale2
    valid2 = (z2 > 0.3) & (z2 < 2.5)

    z_vals = z2[valid2]
    x_vals = (uu2[valid2] - intr2["ppx"]) / intr2["fx"] * z_vals
    y_vals = (vv2[valid2] - intr2["ppy"]) / intr2["fy"] * z_vals

    pts2 = np.vstack((x_vals, y_vals, z_vals))
    pts1 = R @ pts2 + t

    z1 = pts1[2, :]
    valid1 = z1 > 0.1

    u1 = np.round(pts1[0, valid1] / z1[valid1] * intr1["fx"] + intr1["ppx"]).astype(int)
    v1 = np.round(pts1[1, valid1] / z1[valid1] * intr1["fy"] + intr1["ppy"]).astype(int)

    in_bounds = (u1 >= 0) & (u1 < w1) & (v1 >= 0) & (v1 < h1)

    u1_in = u1[in_bounds]
    v1_in = v1[in_bounds]
    colors2 = img2_bgr[vv2[valid2][valid1][in_bounds], uu2[valid2][valid1][in_bounds]]

    # Canvas de Cam 2 reproyectada
    warped_cam2 = np.zeros_like(img1_bgr)
    mask_cam2 = np.zeros((h1, w1), dtype=np.uint8)

    warped_cam2[v1_in, u1_in] = colors2
    mask_cam2[v1_in, u1_in] = 255

    kernel = np.ones((3, 3), np.uint8)
    warped_cam2 = cv2.dilate(warped_cam2, kernel, iterations=1)
    mask_cam2 = cv2.dilate(mask_cam2, kernel, iterations=1)

    # 1. Overlay Blend con Cam 1
    overlay = img1_bgr.copy()
    overlap_mask = mask_cam2 > 0
    overlay[overlap_mask] = cv2.addWeighted(
        img1_bgr[overlap_mask], 1.0 - alpha,
        warped_cam2[overlap_mask], alpha, 0
    )

    decomp = matrix_to_euler_translation(T)
    tx = decomp["translation_cm"]["x"]
    yaw = decomp["rotation_deg"]["yaw"]
    cv2.rectangle(overlay, (0, 0), (w1, 36), (15, 23, 42), -1)
    cv2.putText(overlay, f"FUSION 2D EN PERSPECTIVA: Tx={tx:+.1f}cm, Yaw={yaw:+.1f}deg (CAM1 + CAM2)",
                (14, 24), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 255, 200), 1, cv2.LINE_AA)

    # 2. Anaglifo 3D Estéreo
    anaglyph = np.zeros_like(img1_bgr)
    anaglyph[:, :, 2] = img1_bgr[:, :, 2]
    anaglyph[:, :, 0] = warped_cam2[:, :, 0]
    anaglyph[:, :, 1] = warped_cam2[:, :, 1]
    cv2.rectangle(anaglyph, (0, 0), (w1, 36), (15, 23, 42), -1)
    cv2.putText(anaglyph, "ANAGLIFO 3D ESTEREO (ROJO: CAM 1, CIAN: CAM 2 FUSIONADA)",
                (14, 24), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255, 255, 0), 1, cv2.LINE_AA)

    # 3. Vista Epipolar con líneas horizontales
    epipolar = np.hstack([img1_bgr, img2_bgr])
    eh, ew = epipolar.shape[:2]
    for y_line in range(40, eh, 45):
        cv2.line(epipolar, (0, y_line), (ew, y_line), (0, 255, 255), 1, cv2.LINE_AA)
        cv2.circle(epipolar, (10, y_line), 3, (0, 255, 255), -1)

    cv2.rectangle(epipolar, (0, 0), (ew, 32), (18, 18, 24), -1)
    cv2.putText(epipolar, "NIVEL EPIPOLAR HORIZONTAL (LINEAS GUIA DE ALTURA ENTRE CAM 1 Y CAM 2)",
                (18, 22), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 255, 255), 1, cv2.LINE_AA)

    return overlay, anaglyph, epipolar


# Instancias globales
receiver = LiveCameraReceiver()
alineador = AlineadorVirtual()
captures_store = []
active_capture_id = None
latest_alignment_result = {}
capture_counter = 0


# ==========================================
# RUTAS DE PROXY PARA EL VISOR 3D (SAME ORIGIN)
# ==========================================
async def handle_viewer3d(request):
    """Sirve el visor 3D Three.js en el mismo origen (puerto 5005) para control directo de Three.js."""
    try:
        index_file = os.path.join(LIVE3D_WEB_DIR, "index.html")
        with open(index_file, "r", encoding="utf-8") as f:
            html = f.read()
        return web.Response(text=html, content_type="text/html")
    except Exception as e:
        return web.Response(text=f"Error cargando visor 3D: {e}", status=502)


async def handle_assets_main_js(request):
    """Entrega main.js conectando websocket a 5002 y exponiendo window.viewer para 60 FPS."""
    try:
        main_js_file = os.path.join(LIVE3D_WEB_DIR, "js", "main.js")
        with open(main_js_file, "r", encoding="utf-8") as f:
            js = f.read()

        js = js.replace('const wsUrl = `${proto}//${location.host}/ws`;',
                        'const wsUrl = "ws://localhost:5002/ws";')

        js = js.replace("fetch('/api/", "fetch('http://localhost:5002/api/")
        js = js.replace('fetch("/api/', 'fetch("http://localhost:5002/api/')

        js += """
window.viewer = viewer;
window.updateSlidersFromEuler = updateSlidersFromEuler;
window.addEventListener('message', (e) => {
    if (e.data && e.data.type === 'SET_EXTRINSICS' && window.viewer) {
        window.viewer.setExtrinsics(e.data.matrix_4x4);
        if (typeof updateSlidersFromEuler === 'function' && e.data.components) {
            updateSlidersFromEuler(e.data.components);
        }
    }
});
"""
        return web.Response(text=js, content_type="application/javascript")
    except Exception as e:
        return web.Response(text=f"// Error: {e}", status=500)


# ==========================================
# API DE ESTADO Y CAPTURA
# ==========================================
async def api_status(request):
    """Verifica estado de servidores, cámaras en vivo y capturas realizadas."""
    is_live_ok = False
    extrinsics_current = None
    try:
        req = urllib.request.Request("http://localhost:5002/api/status")
        with urllib.request.urlopen(req, timeout=1.0) as resp:
            data = json.loads(resp.read().decode("utf-8"))
            is_live_ok = data.get("type") == "status" or "camera" in data

        req_ext = urllib.request.Request("http://localhost:5002/api/dual/extrinsics")
        with urllib.request.urlopen(req_ext, timeout=1.0) as resp_ext:
            ext_data = json.loads(resp_ext.read().decode("utf-8"))
            extrinsics_current = ext_data.get("components")
    except Exception:
        pass

    captures_summary = [
        {
            "id": c["id"],
            "title": c["title"],
            "timestamp": c["timestamp"],
            "preview_url": c["preview_url"],
            "matches_url": c.get("matches_url"),
            "overlay_url": c.get("overlay_url"),
            "anaglyph_url": c.get("anaglyph_url"),
            "epipolar_url": c.get("epipolar_url"),
            "aligned": c.get("aligned", False),
            "fitness_pct": c.get("fitness_pct", 0),
            "rmse_mm": c.get("rmse_mm", 0)
        }
        for c in captures_store
    ]

    return web.json_response({
        "status": "ok",
        "live3d_online": is_live_ok,
        "receiver_fps": round(receiver.fps, 1),
        "has_live_feed": receiver.latest_preview_jpeg is not None,
        "current_pose": extrinsics_current,
        "captures_count": len(captures_store),
        "active_capture_id": active_capture_id,
        "captures": captures_summary,
        "has_last_alignment": bool(latest_alignment_result),
        "latest_alignment": latest_alignment_result
    })


async def api_live_stream(request):
    """Stream de video MJPEG de ambas cámaras en vivo side-by-side."""
    response = web.StreamResponse(
        status=200,
        reason='OK',
        headers={
            'Content-Type': 'multipart/x-mixed-replace; boundary=frame',
            'Cache-Control': 'no-cache, no-store, must-revalidate',
            'Pragma': 'no-cache',
            'Expires': '0'
        }
    )
    await response.prepare(request)
    try:
        while True:
            jpeg_data = receiver.latest_preview_jpeg
            if jpeg_data:
                await response.write(
                    b'--frame\r\n'
                    b'Content-Type: image/jpeg\r\n\r\n' + jpeg_data + b'\r\n'
                )
            await asyncio.sleep(0.066)
    except (ConnectionResetError, asyncio.CancelledError, Exception):
        pass
    return response


async def api_live_frame(request):
    """Retorna un único cuadro JPEG actual del stream en vivo."""
    jpeg_data = receiver.latest_preview_jpeg
    if not jpeg_data:
        dummy = np.zeros((320, 800, 3), dtype=np.uint8)
        cv2.putText(dummy, "Conectando al stream de camaras...", (180, 160),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.8, (200, 200, 200), 2)
        _, buf = cv2.imencode('.jpg', dummy)
        jpeg_data = buf.tobytes()

    return web.Response(body=jpeg_data, content_type='image/jpeg')


async def api_capture(request):
    """Toma una nueva captura instantánea de ambas cámaras y la agrega a la galería."""
    global capture_counter, active_capture_id
    pair = receiver.get_instant_pair()
    if pair is None:
        return web.json_response({
            "status": "error",
            "message": "Las cámaras aún no han enviado cuadros sincronizados. Espera 1 segundo."
        }, status=503)

    capture_counter += 1
    cid = capture_counter
    ts_str = time.strftime("%H:%M:%S")
    title = f"Toma #{cid}"

    img1 = pair["img1"]
    img2 = pair["img2"]

    h1, w1 = img1.shape[:2]
    preview_comb = np.hstack([img1, img2])

    pw = preview_comb.shape[1]
    cv2.rectangle(preview_comb, (0, h1 - 42), (pw, h1), (20, 20, 28), -1)
    cv2.putText(preview_comb, f"CAPTURA #{cid} · {ts_str} | CAM 1 (MASTER) y CAM 2 (SLAVE)",
                (20, h1 - 14), cv2.FONT_HERSHEY_SIMPLEX, 0.75, (0, 230, 255), 2, cv2.LINE_AA)

    preview_filename = f"capture_{cid}_preview.jpg"
    preview_disk_path = os.path.join(CAPTURES_DIR, preview_filename)
    cv2.imwrite(preview_disk_path, preview_comb)

    record = {
        "id": cid,
        "title": title,
        "timestamp": ts_str,
        "preview_url": f"/captures/{preview_filename}?t={int(time.time()*1000)}",
        "matches_url": None,
        "overlay_url": None,
        "anaglyph_url": None,
        "epipolar_url": None,
        "aligned": False,
        "fitness_pct": 0,
        "rmse_mm": 0,
        "pair_data": pair
    }

    captures_store.insert(0, record)
    active_capture_id = cid

    if len(captures_store) > 15:
        old = captures_store.pop()
        for suffix in ["preview", "matches", "overlay", "anaglyph", "epipolar"]:
            old_path = os.path.join(CAPTURES_DIR, f"capture_{old['id']}_{suffix}.jpg")
            if os.path.exists(old_path):
                try:
                    os.remove(old_path)
                except Exception:
                    pass

    print(f"[Captura] Nueva Toma #{cid} guardada con éxito a las {ts_str}")

    return web.json_response({
        "status": "success",
        "message": f"¡Toma #{cid} capturada con éxito!",
        "capture": {
            "id": record["id"],
            "title": record["title"],
            "timestamp": record["timestamp"],
            "preview_url": record["preview_url"],
            "aligned": False
        },
        "active_capture_id": active_capture_id,
        "total_captures": len(captures_store)
    })


async def api_align(request):
    """Ejecuta la Alineación Virtual y genera las vistas de Fusión 2D Superpuesta en las cámaras."""
    global latest_alignment_result, active_capture_id
    try:
        data = {}
        if request.can_read_body:
            try:
                data = await request.json()
            except Exception:
                pass

        target_id = data.get("capture_id")
        target_rec = None

        if target_id is not None:
            for c in captures_store:
                if c["id"] == int(target_id):
                    target_rec = c
                    break
        elif active_capture_id is not None:
            for c in captures_store:
                if c["id"] == active_capture_id:
                    target_rec = c
                    break

        if target_rec is None:
            pair = receiver.get_instant_pair()
            if pair is None:
                return web.json_response({
                    "status": "error",
                    "message": "No hay capturas disponibles. Toma una foto primero."
                }, status=400)
            await api_capture(request)
            target_rec = captures_store[0]

        active_capture_id = target_rec["id"]
        pair = target_rec["pair_data"]

        initial_guess = None
        if latest_alignment_result and "matrix_4x4" in latest_alignment_result:
            initial_guess = latest_alignment_result["matrix_4x4"]

        t0 = time.time()
        res = alineador.align(
            img1_bgr=pair["img1"], depth1_u16=pair["depth1"], intr1=pair["intr1"],
            img2_bgr=pair["img2"], depth2_u16=pair["depth2"], intr2=pair["intr2"],
            scale1=pair["scale1"], scale2=pair["scale2"],
            initial_guess=initial_guess
        )
        t_total = int((time.time() - t0) * 1000)

        if not res["success"]:
            return web.json_response({
                "status": "error",
                "message": res["message"]
            }, status=400)

        cid = target_rec["id"]
        now_ts = int(time.time() * 1000)

        # 1. Guardar imagen diagnóstica con correspondencias 3D
        matches_fn = f"capture_{cid}_matches.jpg"
        cv2.imwrite(os.path.join(CAPTURES_DIR, matches_fn), res["debug_image"])
        cv2.imwrite(os.path.join(STATIC_DIR, "latest_matches.jpg"), res["debug_image"])
        matches_url = f"/captures/{matches_fn}?t={now_ts}"

        # 2. Generar Vistas de Fusión 2D Superpuesta en las Cámaras
        overlay_img, anaglyph_img, epipolar_img = compute_camera_alignment_views(
            img1_bgr=pair["img1"], depth1_u16=pair["depth1"], intr1=pair["intr1"], scale1=pair["scale1"],
            img2_bgr=pair["img2"], depth2_u16=pair["depth2"], intr2=pair["intr2"], scale2=pair["scale2"],
            T_matrix=res["matrix_4x4"], alpha=0.5
        )

        overlay_fn = f"capture_{cid}_overlay.jpg"
        anaglyph_fn = f"capture_{cid}_anaglyph.jpg"
        epipolar_fn = f"capture_{cid}_epipolar.jpg"

        cv2.imwrite(os.path.join(CAPTURES_DIR, overlay_fn), overlay_img)
        cv2.imwrite(os.path.join(CAPTURES_DIR, anaglyph_fn), anaglyph_img)
        cv2.imwrite(os.path.join(CAPTURES_DIR, epipolar_fn), epipolar_img)

        overlay_url = f"/captures/{overlay_fn}?t={now_ts}"
        anaglyph_url = f"/captures/{anaglyph_fn}?t={now_ts}"
        epipolar_url = f"/captures/{epipolar_fn}?t={now_ts}"

        # Actualizar registro de la captura
        target_rec["aligned"] = True
        target_rec["matches_url"] = matches_url
        target_rec["overlay_url"] = overlay_url
        target_rec["anaglyph_url"] = anaglyph_url
        target_rec["epipolar_url"] = epipolar_url
        target_rec["fitness_pct"] = res["fitness_pct"]
        target_rec["rmse_mm"] = res["rmse_mm"]
        target_rec["matrix_4x4"] = res["matrix_4x4"]
        target_rec["components"] = res["components"]

        # Aplicar en caliente al visor 3D Three.js
        apply_extrinsics_to_live_viewer(res["matrix_4x4"])

        latest_alignment_result = {
            "status": "success",
            "capture_id": cid,
            "timestamp": time.strftime("%H:%M:%S"),
            "total_time_ms": res.get("total_time_ms", t_total),
            "fitness_pct": res["fitness_pct"],
            "rmse_mm": res["rmse_mm"],
            "components": res["components"],
            "matrix_4x4": res["matrix_4x4"],
            "matches_url": matches_url,
            "overlay_url": overlay_url,
            "anaglyph_url": anaglyph_url,
            "epipolar_url": epipolar_url,
            "image_url": overlay_url
        }

        return web.json_response(latest_alignment_result)

    except Exception as e:
        import traceback
        traceback.print_exc()
        return web.json_response({"status": "error", "message": str(e)}, status=500)


async def api_reproject(request):
    """Re-genera la fusión 2D en perspectiva con una opacidad alpha específica."""
    try:
        data = await request.json()
        cid = int(data.get("capture_id", active_capture_id or 1))
        alpha = float(data.get("alpha", 0.5))

        target_rec = None
        for c in captures_store:
            if c["id"] == cid:
                target_rec = c
                break

        if not target_rec or "matrix_4x4" not in target_rec:
            return web.json_response({"status": "error", "message": "Captura sin alineación previa"}, status=400)

        pair = target_rec["pair_data"]
        overlay_img, _, _ = compute_camera_alignment_views(
            pair["img1"], pair["depth1"], pair["intr1"], pair["scale1"],
            pair["img2"], pair["depth2"], pair["intr2"], pair["scale2"],
            target_rec["matrix_4x4"], alpha=alpha
        )

        overlay_fn = f"capture_{cid}_overlay.jpg"
        cv2.imwrite(os.path.join(CAPTURES_DIR, overlay_fn), overlay_img)
        now_ts = int(time.time() * 1000)

        return web.json_response({
            "status": "success",
            "overlay_url": f"/captures/{overlay_fn}?t={now_ts}"
        })
    except Exception as e:
        return web.json_response({"status": "error", "message": str(e)}, status=400)


async def api_select_capture(request):
    """Selecciona qué captura visualizar en el visor."""
    global active_capture_id
    try:
        data = await request.json()
        cid = int(data.get("id"))
        for c in captures_store:
            if c["id"] == cid:
                active_capture_id = cid
                return web.json_response({
                    "status": "success",
                    "active_capture_id": cid,
                    "capture": {
                        "id": c["id"],
                        "title": c["title"],
                        "timestamp": c["timestamp"],
                        "preview_url": c["preview_url"],
                        "matches_url": c.get("matches_url"),
                        "overlay_url": c.get("overlay_url"),
                        "anaglyph_url": c.get("anaglyph_url"),
                        "epipolar_url": c.get("epipolar_url"),
                        "aligned": c.get("aligned", False),
                        "fitness_pct": c.get("fitness_pct", 0),
                        "rmse_mm": c.get("rmse_mm", 0)
                    }
                })
        return web.json_response({"status": "error", "message": "Captura no encontrada."}, status=404)
    except Exception as e:
        return web.json_response({"status": "error", "message": str(e)}, status=400)


async def api_delete_capture(request):
    """Elimina una captura del historial."""
    global active_capture_id
    try:
        data = await request.json()
        cid = int(data.get("id"))
        idx = -1
        for i, c in enumerate(captures_store):
            if c["id"] == cid:
                idx = i
                break
        if idx >= 0:
            captures_store.pop(idx)
            for suffix in ["preview", "matches", "overlay", "anaglyph", "epipolar"]:
                p = os.path.join(CAPTURES_DIR, f"capture_{cid}_{suffix}.jpg")
                if os.path.exists(p):
                    try:
                        os.remove(p)
                    except Exception:
                        pass
            if active_capture_id == cid:
                active_capture_id = captures_store[0]["id"] if captures_store else None
            return web.json_response({
                "status": "success",
                "active_capture_id": active_capture_id,
                "remaining_count": len(captures_store)
            })
        return web.json_response({"status": "error", "message": "No encontrada."}, status=404)
    except Exception as e:
        return web.json_response({"status": "error", "message": str(e)}, status=400)


async def api_set_manual(request):
    """Permite ajustar manualmente la pose mediante sliders a 60 FPS."""
    global latest_alignment_result
    try:
        data = await request.json()
        tx = float(data.get("tx", -60.0))
        ty = float(data.get("ty", 2.0))
        tz = float(data.get("tz", 20.0))
        yaw = float(data.get("yaw", 45.0))
        pitch = float(data.get("pitch", 0.0))
        roll = float(data.get("roll", 0.0))

        T = euler_translation_to_matrix(tx, ty, tz, yaw, pitch, roll)
        apply_extrinsics_to_live_viewer(T.tolist())

        decomp = matrix_to_euler_translation(T)
        latest_alignment_result["matrix_4x4"] = T.tolist()
        latest_alignment_result["components"] = decomp

        return web.json_response({
            "status": "success",
            "components": decomp,
            "matrix_4x4": T.tolist()
        })
    except Exception as e:
        return web.json_response({"status": "error", "message": str(e)}, status=400)


async def api_apply(request):
    """Re-aplica la última pose calculada al visor Live 3D."""
    if not latest_alignment_result or "matrix_4x4" not in latest_alignment_result:
        return web.json_response({"status": "error", "message": "No hay alineación previa para aplicar."}, status=400)

    ok = apply_extrinsics_to_live_viewer(latest_alignment_result["matrix_4x4"])
    if ok:
        return web.json_response({"status": "success", "message": "¡Alineación aplicada exitosamente en el visor 3D!"})
    else:
        return web.json_response({"status": "error", "message": "No se pudo comunicar con el puerto 5002."}, status=502)


async def handle_index(request):
    return web.FileResponse(os.path.join(STATIC_DIR, "index.html"))


async def on_startup(app):
    app["receiver_task"] = asyncio.create_task(receiver.run())


async def on_cleanup(app):
    receiver.running = False
    if "receiver_task" in app:
        app["receiver_task"].cancel()
        try:
            await app["receiver_task"]
        except (asyncio.CancelledError, Exception):
            pass


def create_app():
    os.makedirs(STATIC_DIR, exist_ok=True)
    os.makedirs(CAPTURES_DIR, exist_ok=True)

    app = web.Application()
    app.on_startup.append(on_startup)
    app.on_cleanup.append(on_cleanup)

    # 3D Viewport Proxy (Mismo origen)
    app.router.add_get("/viewer3d", handle_viewer3d)
    app.router.add_get("/assets/js/main.js", handle_assets_main_js)
    app.router.add_static("/assets", LIVE3D_WEB_DIR)

    app.router.add_get("/", handle_index)
    app.router.add_get("/api/status", api_status)
    app.router.add_get("/api/live_stream", api_live_stream)
    app.router.add_get("/api/live_frame", api_live_frame)
    app.router.add_post("/api/capture", api_capture)
    app.router.add_post("/api/align", api_align)
    app.router.add_post("/api/calibrate", api_align)
    app.router.add_post("/api/reproject", api_reproject)
    app.router.add_post("/api/captures/select", api_select_capture)
    app.router.add_post("/api/captures/delete", api_delete_capture)
    app.router.add_post("/api/set_manual", api_set_manual)
    app.router.add_post("/api/apply", api_apply)

    app.router.add_static("/captures", CAPTURES_DIR)
    app.router.add_static("/", STATIC_DIR)
    return app


if __name__ == "__main__":
    print("=" * 70)
    print("  SERVIDOR DE PRUEBA: ALINEACIÓN VIRTUAL Y ESTUDIO DUAL")
    print("  Puerto: 5005 (Totalmente Independiente)")
    print("=" * 70)
    print("  Abre tu navegador en: http://localhost:5005")
    print("=" * 70)

    app = create_app()
    web.run_app(app, host="0.0.0.0", port=PORT)
