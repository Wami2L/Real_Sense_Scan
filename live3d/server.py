"""
Servidor RealSense Live 3D.

    python -m live3d                 # cámara RealSense
    python -m live3d --synthetic     # escena sintética (sin cámara)
    python -m live3d --host 0.0.0.0  # accesible desde la red (p. ej. Blender en otro PC)

Arquitectura (pensada para varios consumidores: navegador hoy, Blender mañana):

  [hilo captura] --CapturedFrame--> codifica SOLO las variantes pedidas
        |                           (zlib/raw x jpeg/raw/none), una vez por cuadro
        v
  loop asyncio: Hub.publish() -> cada cliente guarda solo el ÚLTIMO paquete
        |
        v
  tarea emisora por cliente: envía meta (si cambió) + último cuadro.
     - ack=true : máx. MAX_INFLIGHT cuadros sin confirmar (clientes lentos
                  como Blender reciben siempre el cuadro más reciente, sin colas).
     - ack=false: backpressure del socket (drain) + descarte de cuadros viejos.

NOTA: una RealSense solo puede abrirse desde un proceso. No ejecutes app.py y este
servidor a la vez con la misma cámara.
"""

import argparse
import asyncio
import collections
import itertools
import json
import os
import time

import cv2
import numpy as np
from aiohttp import WSMsgType, web

from . import protocol
from .capture import DualRealSenseSource, RealSenseSource, Settings, SettingsStore, SyntheticSource, SyntheticDualSource

WEB_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), "web")
MAX_INFLIGHT = 4  # 2 por cámara en modo dual
STATUS_PERIOD = 1.0


# --------------------------------------------------------------------------- #
# Clientes y hub
# --------------------------------------------------------------------------- #
class Client:
    _ids = itertools.count(1)

    def __init__(self, ws, remote):
        self.id = next(self._ids)
        self.ws = ws
        self.remote = remote
        self.kind = "unknown"
        self.depth_codec = "zlib"
        self.color_codec = "jpeg"
        self.ack = False
        self.subscribed = False
        self.inflight = 0
        self.latest = {}       # cam_idx -> último paquete pendiente (uno por cámara)
        self.texts = collections.deque()
        self.event = asyncio.Event()
        self.meta_sent = {}    # cam_idx -> meta_id ya enviado
        # estadísticas
        self.bytes_sent = 0
        self.frames_sent = 0
        self.dropped = 0
        self._last = (time.time(), 0, 0, 0)
        self.rate = {"mbps": 0.0, "fps": 0.0, "drop_fps": 0.0}

    def queue_text(self, payload):
        self.texts.append(payload if isinstance(payload, str) else json.dumps(payload, separators=(",", ":")))
        self.event.set()

    def update_rates(self):
        t0, b0, f0, d0 = self._last
        now = time.time()
        dt = max(1e-3, now - t0)
        self.rate = {
            "mbps": round((self.bytes_sent - b0) * 8 / dt / 1e6, 2),
            "fps": round((self.frames_sent - f0) / dt, 1),
            "drop_fps": round((self.dropped - d0) / dt, 1),
        }
        self._last = (now, self.bytes_sent, self.frames_sent, self.dropped)


class Hub:
    def __init__(self):
        self.clients = set()
        # Leído desde el hilo de captura: se reemplaza atómicamente (frozenset)
        self.variants = frozenset()
        self.last_meta = None
        self.encode_ms = 0.0

    def add(self, c):
        self.clients.add(c)

    def remove(self, c):
        self.clients.discard(c)
        self.recompute()

    def recompute(self):
        self.variants = frozenset((c.depth_codec, c.color_codec) for c in self.clients if c.subscribed)

    def publish(self, meta, packets, encode_ms):
        """Se ejecuta en el loop asyncio. `packets`: {(depth_codec, color_codec): bytes}."""
        self.last_meta = meta
        self.encode_ms = encode_ms
        cam = int(meta.get("cam_idx", 0))
        meta_json = None
        for c in self.clients:
            if not c.subscribed:
                continue
            pkt = packets.get((c.depth_codec, c.color_codec))
            if pkt is None:
                continue
            if c.meta_sent.get(cam) != meta["meta_id"]:
                meta_json = meta_json or protocol.meta_to_json(meta)
                c.queue_text(meta_json)
                c.meta_sent[cam] = meta["meta_id"]
            if cam in c.latest:
                c.dropped += 1  # el anterior de esta cámara no llegó a salir: se descarta
            c.latest[cam] = pkt
            c.event.set()

    def broadcast(self, payload):
        text = json.dumps(payload, separators=(",", ":"))
        for c in self.clients:
            c.queue_text(text)


# --------------------------------------------------------------------------- #
# Codificación (se ejecuta en el hilo de captura, fuera del loop)
# --------------------------------------------------------------------------- #
def make_frame_handler(hub, loop):
    def on_frame(frame):
        variants = hub.variants
        if not variants:
            return
        t0 = time.perf_counter()
        depth_enc, color_enc = {}, {}
        packets = {}
        for dc, cc in variants:
            if dc not in depth_enc:
                depth_enc[dc] = protocol.encode_depth(frame.depth, dc)
            eff_cc = cc if frame.color is not None else "none"
            if eff_cc not in color_enc:
                color_enc[eff_cc] = encode_color(frame.color, eff_cc, frame.settings.jpeg_quality)
            packets[(dc, cc)] = protocol.pack_frame(
                frame.seq, frame.timestamp, frame.meta["meta_id"],
                dc, depth_enc[dc], eff_cc, color_enc[eff_cc],
                flags=int(frame.meta.get("cam_idx", 0)) & 1)
        enc_ms = (time.perf_counter() - t0) * 1000.0
        try:
            loop.call_soon_threadsafe(hub.publish, frame.meta, packets, enc_ms)
        except RuntimeError:
            pass  # loop cerrado durante el apagado
    return on_frame


def encode_color(rgb, codec, quality):
    if codec == "none" or rgb is None:
        return b""
    if codec == "raw":
        return np.ascontiguousarray(rgb).tobytes()
    bgr = cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR)
    ok, buf = cv2.imencode(".jpg", bgr, [cv2.IMWRITE_JPEG_QUALITY, int(quality)])
    return buf.tobytes() if ok else b""


# --------------------------------------------------------------------------- #
# WebSocket
# --------------------------------------------------------------------------- #
async def sender_loop(c):
    ws = c.ws
    try:
        while not ws.closed:
            await c.event.wait()
            c.event.clear()
            while c.texts and not ws.closed:
                await ws.send_str(c.texts.popleft())
            while c.latest and not ws.closed and not (c.ack and c.inflight >= MAX_INFLIGHT):
                cam = next(iter(c.latest))
                data = c.latest.pop(cam)
                await ws.send_bytes(data)  # espera al drain del socket: backpressure natural
                c.inflight += 1
                c.frames_sent += 1
                c.bytes_sent += len(data)
    except (ConnectionResetError, asyncio.CancelledError):
        pass
    except Exception as e:  # noqa: BLE001
        print(f"[Live3D] Cliente {c.id}: error enviando: {e}")


def status_payload(app, c=None):
    src = app["source"]
    hub = app["hub"]
    payload = {
        "type": "status",
        "camera": src.status(),
        "settings": app["store"].get().public(),
        "clients": len(hub.clients),
        "encode_ms": round(hub.encode_ms, 2),
        "server_time": time.time(),
    }
    if c is not None:
        payload["you"] = {"id": c.id, **c.rate, "depth": c.depth_codec,
                          "color": c.color_codec, "ack": c.ack}
    return payload


async def ws_handler(request):
    app = request.app
    hub = app["hub"]
    # compress=False: los datos ya van comprimidos (zlib/JPEG); permessage-deflate
    # solo añadiría latencia y CPU.
    ws = web.WebSocketResponse(compress=False, heartbeat=15.0, max_msg_size=1 << 20)
    await ws.prepare(request)
    c = Client(ws, request.remote)
    hub.add(c)
    sender = asyncio.create_task(sender_loop(c))
    c.queue_text({"type": "hello", "protocol": protocol.PROTOCOL_VERSION,
                  "header_size": protocol.HEADER_SIZE, "client_id": c.id,
                  "max_inflight": MAX_INFLIGHT})
    c.queue_text(status_payload(app, c))
    print(f"[Live3D] Cliente {c.id} conectado ({c.remote})")
    try:
        async for msg in ws:
            if msg.type != WSMsgType.TEXT:
                if msg.type == WSMsgType.ERROR:
                    break
                continue
            try:
                data = json.loads(msg.data)
            except ValueError:
                continue
            t = data.get("type")
            if t == "ack":
                c.inflight = max(0, c.inflight - 1)
                c.event.set()
            elif t == "subscribe":
                c.depth_codec = data.get("depth", "zlib") if data.get("depth") in protocol.DEPTH_CODECS else "zlib"
                c.color_codec = data.get("color", "jpeg") if data.get("color") in protocol.COLOR_CODECS else "jpeg"
                c.ack = bool(data.get("ack", False))
                c.kind = str(data.get("client", "unknown"))[:32]
                c.inflight = 0
                c.meta_sent = {}  # reenviar meta
                c.subscribed = True
                hub.recompute()
                print(f"[Live3D] Cliente {c.id} ({c.kind}) suscrito: depth={c.depth_codec} "
                      f"color={c.color_codec} ack={c.ack}")
            elif t == "unsubscribe":
                c.subscribed = False
                hub.recompute()
            elif t == "settings":
                changes = {k: v for k, v in data.items() if k != "type"}
                try:
                    _, profile_changed = app["store"].update(changes)
                except (TypeError, ValueError) as e:
                    c.queue_text({"type": "error", "message": f"Ajuste inválido: {e}"})
                    continue
                if profile_changed:
                    app["source"].request_restart()
                for other in hub.clients:
                    other.queue_text(status_payload(app, other))
    finally:
        hub.remove(c)
        sender.cancel()
        print(f"[Live3D] Cliente {c.id} desconectado")
    return ws


async def status_task(app):
    hub = app["hub"]
    while True:
        await asyncio.sleep(STATUS_PERIOD)
        for c in list(hub.clients):
            c.update_rates()
            c.queue_text(status_payload(app, c))


# --------------------------------------------------------------------------- #
# HTTP
# --------------------------------------------------------------------------- #
async def index(_request):
    return web.FileResponse(os.path.join(WEB_DIR, "index.html"))


async def api_status(request):
    return web.json_response(status_payload(request.app))


async def api_client_log(request):
    try:
        data = await request.json()
        print(f"[CLIENT LOG] {data}", flush=True)
    except Exception:
        pass
    return web.json_response({"ok": True})


ROOT_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
SAVES_DIR = os.path.join(ROOT_DIR, "Saves")
os.makedirs(SAVES_DIR, exist_ok=True)

import sys
if ROOT_DIR not in sys.path:
    sys.path.insert(0, ROOT_DIR)
try:
    from multiscan import MultiScanSession
except ImportError:
    MultiScanSession = None


async def api_meta(request):
    return web.json_response(request.app["hub"].last_meta or {})


async def api_capture(request):
    src = request.app.get("source")
    if not src:
        return web.json_response({"status": "error", "message": "Fuente de cámara no disponible"}, status=503)
    loop = asyncio.get_running_loop()
    try:
        res = await loop.run_in_executor(None, src.capture_scan)
        return web.json_response(res)
    except Exception as e:
        return web.json_response({"status": "error", "message": str(e)}, status=500)


def connected_serials():
    try:
        import pyrealsense2 as rs
        return [d.get_info(rs.camera_info.serial_number) for d in rs.context().query_devices()
                if d.supports(rs.camera_info.serial_number)]
    except Exception:  # noqa: BLE001  (sin pyrealsense2 o sin driver)
        return []


def choose_source(single=False):
    """Elige la fuente según lo conectado. Devuelve (clase, mensaje, es_respaldo_sintético).

    Con 2 cámaras configuradas en config/dual_camera_config.json basta con que UNA esté
    conectada para arrancar en modo dual: la que falta se muestra como 'no conectada' en vez
    de cambiar silenciosamente de modo.
    """
    found = connected_serials()
    configured = []
    try:
        from .capture import DUAL_CONFIG_PATH
        with open(DUAL_CONFIG_PATH, encoding="utf-8") as f:
            configured = [c["serial"] for c in json.load(f).get("cameras", []) if c.get("serial")]
    except (OSError, ValueError):
        pass
    if not single and (len(found) >= 2 or (len(configured) >= 2 and set(found) & set(configured))):
        missing = [s for s in configured[:2] if s not in found]
        msg = f"Modo dual ({len(found)} RealSense conectadas)" + (f"; falta {', '.join(missing)}" if missing else "")
        return DualRealSenseSource, msg, False
    if found:
        return RealSenseSource, f"Modo cámara única ({found[0]})", False
    if single:
        return SyntheticSource, "Sin cámaras físicas: simulación de 1 cámara", True
    return SyntheticDualSource, "Sin cámaras físicas: simulación de 2 cámaras", True


async def switch_source(app, cls, message, fallback):
    loop = asyncio.get_running_loop()
    hub = app["hub"]
    old_src = app.get("source")
    if old_src:
        await loop.run_in_executor(None, old_src.stop)
    new_src = cls(app["store"], make_frame_handler(hub, loop), has_demand=lambda: bool(hub.variants))
    app["source"] = new_src
    app["source_fallback"] = fallback
    for c in hub.clients:  # los meta_id pueden repetirse entre fuentes: forzar reenvío del meta
        c.meta_sent = {}
    new_src.start()
    print(f"[Live3D] Fuente: {new_src.name} · {message}", flush=True)
    hub.broadcast({"type": "source_changed", "source": new_src.name, "message": message,
                   "simulated": bool(getattr(new_src, "simulated", False))})
    return new_src


async def device_watch_task(app, period=3.0):
    """Si se arrancó en simulación por falta de cámaras, pasa a las reales en cuanto se conectan."""
    loop = asyncio.get_running_loop()
    while True:
        await asyncio.sleep(period)
        if not app.get("source_fallback"):
            continue
        found = await loop.run_in_executor(None, connected_serials)
        if found:
            await asyncio.sleep(2.0)  # dar tiempo a que enumere la segunda cámara
            cls, msg, fallback = await loop.run_in_executor(None, choose_source, app["args_single"])
            if not fallback:
                await switch_source(app, cls, "Cámara detectada: " + msg, fallback)


async def api_reload_camera(request):
    app = request.app
    loop = asyncio.get_running_loop()
    cls, msg, fallback = await loop.run_in_executor(None, choose_source, app["args_single"])
    new_src = await switch_source(app, cls, msg, fallback)
    return web.json_response({"status": "success", "message": msg, "source": new_src.name,
                              "simulated": bool(getattr(new_src, "simulated", False))})


# ----------------- MULTI-CAPTURA -----------------
async def api_multi_start(request):
    app = request.app
    if MultiScanSession is None:
        return web.json_response({"status": "error", "message": "Módulo multiscan no disponible"}, status=500)
    async with app["multi_lock"]:
        sess = app.get("multi_session")
        if sess and sess.state == "capturing":
            sess.discard()
        app["multi_session"] = MultiScanSession(ROOT_DIR)
        return web.json_response({"status": "success", "session_id": app["multi_session"].id})


async def api_multi_add(request):
    app = request.app
    src = app.get("source")
    if not src:
        return web.json_response({"status": "error", "message": "Cámara no disponible"}, status=503)
    
    loop = asyncio.get_running_loop()
    try:
        snap = await loop.run_in_executor(None, src.get_snapshot, 5.0)
    except Exception as e:
        return web.json_response({"status": "error", "message": f"Error capturando snapshot: {e}"}, status=500)

    async with app["multi_lock"]:
        sess = app.get("multi_session")
        if not sess or sess.state != "capturing":
            return web.json_response({"status": "error", "message": "No hay sesión multi-captura activa"}, status=400)
        try:
            res = await loop.run_in_executor(None, sess.add_view, snap)
            return web.json_response({"status": "success", "view": res})
        except Exception as e:
            import traceback
            traceback.print_exc()
            return web.json_response({"status": "error", "message": str(e)}, status=500)


async def api_multi_undo(request):
    app = request.app
    async with app["multi_lock"]:
        sess = app.get("multi_session")
        if not sess:
            return web.json_response({"status": "error", "message": "Sin sesión activa"}, status=400)
        try:
            sess.undo()
            return web.json_response({"status": "success"})
        except Exception as e:
            return web.json_response({"status": "error", "message": str(e)}, status=500)


async def api_multi_discard(request):
    app = request.app
    async with app["multi_lock"]:
        sess = app.get("multi_session")
        if sess:
            sess.discard()
            app["multi_session"] = None
        return web.json_response({"status": "success"})


async def api_multi_finish(request):
    app = request.app
    async with app["multi_lock"]:
        sess = app.get("multi_session")
        if not sess:
            return web.json_response({"status": "error", "message": "Sin sesión activa"}, status=400)
        try:
            sess.finish()  # Corre en hilo interno de fondo en multiscan.py
            return web.json_response({"status": "success", "session_id": sess.id})
        except Exception as e:
            return web.json_response({"status": "error", "message": str(e)}, status=500)


async def api_multi_status(request):
    app = request.app
    sess = app.get("multi_session")
    if not sess:
        return web.json_response({"active": False})
    return web.json_response({"active": True, "session": sess.summary(with_thumbs=True)})


# ----------------- HISTORIAL DE SAVES -----------------
async def api_list_saves(_request):
    if not os.path.isdir(SAVES_DIR):
        return web.json_response({"saves": []})
    
    items = []
    try:
        dates = sorted(os.listdir(SAVES_DIR), reverse=True)
        for date_name in dates:
            date_path = os.path.join(SAVES_DIR, date_name)
            if not os.path.isdir(date_path):
                continue
            scans = sorted(os.listdir(date_path), reverse=True)
            for scan_name in scans:
                scan_path = os.path.join(date_path, scan_name)
                if not os.path.isdir(scan_path):
                    continue
                is_multi = scan_name.startswith("multi_")
                files = os.listdir(scan_path)
                
                thumb_url = None
                if "captura_color.jpg" in files:
                    thumb_url = f"/saves/{date_name}/{scan_name}/captura_color.jpg"
                elif "cam1_color.jpg" in files:
                    thumb_url = f"/saves/{date_name}/{scan_name}/cam1_color.jpg"
                elif "vistas" in files:
                    vistas_dir = os.path.join(scan_path, "vistas")
                    if os.path.isdir(vistas_dir):
                        v_dirs = sorted(os.listdir(vistas_dir))
                        if v_dirs:
                            thumb_url = f"/saves/{date_name}/{scan_name}/vistas/{v_dirs[0]}/color.jpg"
                
                items.append({
                    "id": f"{date_name}/{scan_name}",
                    "date": date_name,
                    "name": scan_name,
                    "is_multi": is_multi,
                    "files": [f for f in files if f.endswith((".ply", ".obj", ".jpg", ".json", ".npz"))],
                    "thumb": thumb_url,
                    "full_path": scan_path
                })
    except Exception as e:
        print(f"[Live3D] Error listando Saves: {e}")
    return web.json_response({"saves": items})


# ----------------- DUAL CAMERA -----------------
async def api_dual_status(request):
    src = request.app.get("source")
    if hasattr(src, "cameras_status"):
        return web.json_response({
            "is_dual": True,
            "simulated": src.simulated,
            "cameras": src.serials,
            "cams": src.cameras_status(),
            "active_index": src._active_cam_idx,
            "view": src.view_mode,
            "extrinsics": src.get_extrinsics().tolist(),
            "calibration": src.calibration_info(),
        })
    return web.json_response({"is_dual": False, "simulated": bool(getattr(src, "simulated", False))})


async def api_dual_select(request):
    """Body: {"mode": "dual"} para transmitir ambas cámaras, o {"index": 0|1} para una sola."""
    src = request.app.get("source")
    if hasattr(src, "set_active_camera"):
        try:
            data = await request.json()
            if data.get("mode") == "dual":
                src.set_view_mode("dual")
            else:
                src.set_active_camera(int(data.get("index", 0)))
                src.set_view_mode("solo")
            return web.json_response({"status": "success", "view": src.view_mode,
                                      "active_index": src._active_cam_idx})
        except Exception as e:
            return web.json_response({"status": "error", "message": str(e)}, status=400)
    return web.json_response({"status": "error", "message": "No está en modo dual"}, status=400)


async def api_dual_capture(request):
    src = request.app.get("source")
    loop = asyncio.get_running_loop()
    if hasattr(src, "capture_dual_scan"):
        try:
            res = await loop.run_in_executor(None, src.capture_dual_scan)
            return web.json_response(res)
        except Exception as e:
            import traceback
            traceback.print_exc()
            return web.json_response({"status": "error", "message": str(e)}, status=500)
    elif hasattr(src, "capture_scan"):
        try:
            res = await loop.run_in_executor(None, src.capture_scan)
            return web.json_response(res)
        except Exception as e:
            return web.json_response({"status": "error", "message": str(e)}, status=500)
    return web.json_response({"status": "error", "message": "Fuente de cámara no disponible"}, status=503)


async def api_dual_calibrate(request):
    src = request.app.get("source")
    if not hasattr(src, "get_snapshot") or not hasattr(src, "serials") or len(src.serials) < 2:
        return web.json_response({"status": "error", "message": "Se requieren 2 cámaras activas para calibrar"}, status=400)
    
    loop = asyncio.get_running_loop()
    try:
        dual_snap = await loop.run_in_executor(None, src.get_snapshot, 5.0)
        snap1 = dual_snap.get("cam1")
        snap2 = dual_snap.get("cam2")
        if not snap1 or not snap2:
            return web.json_response({"status": "error", "message": "No se pudieron obtener cuadros sincronizados"}, status=500)
        
        from tools.calibrate_dual import calibrate_from_images
        
        T_matrix, rmse_px, dbg1, dbg2 = calibrate_from_images(
            snap1["color"], snap2["color"], snap1["intrinsics"], snap2["intrinsics"]
        )
        
        src.set_extrinsics(T_matrix, method="ChArUco", rmse_px=float(rmse_px), source="charuco")

        return web.json_response({
            "status": "success",
            "message": f"Calibración exitosa (Error reproyección: {rmse_px} px)",
            "rmse_px": rmse_px,
            "matrix_4x4": T_matrix.tolist()
        })
    except Exception as e:
        return web.json_response({"status": "error", "message": str(e)}, status=400)


async def api_dual_auto_align(request):
    """Alineación automática Cam 2 -> Cam 1 (fotogrametría RGB-D + ICP, live3d/alignment.py)
    sobre un snapshot nativo emparejado (no sobre el stream comprimido). Solo se guarda si el
    motor la considera fiable; si no, se conserva la calibración vigente."""
    src = request.app.get("source")
    if not hasattr(src, "cameras_status"):
        return web.json_response({"status": "error", "message": "Se requieren 2 cámaras activas para auto-alinear"}, status=400)
    not_live = [c for c in src.cameras_status() if c["state"] != "streaming"]
    if not_live:
        msg = "; ".join(f"Cam {c['idx'] + 1}: {c['message']}" for c in not_live)
        return web.json_response({"status": "error", "message": f"Ambas cámaras deben estar en vivo ({msg})"}, status=409)

    loop = asyncio.get_running_loop()
    try:
        dual_snap = await loop.run_in_executor(None, src.get_snapshot, 6.0)
        from .alignment import RGBDView, align_pair, public_result
        v1 = RGBDView.from_snapshot(dual_snap["cam1"], "Cam 1")
        v2 = RGBDView.from_snapshot(dual_snap["cam2"], "Cam 2")
        prior = src.get_extrinsics() if src.calibration_info().get("is_calibrated") else None
        res = await loop.run_in_executor(None, align_pair, v1, v2, prior)
        out = public_result(res)
        out["simulated"] = src.simulated
        if res["success"]:
            m = res["metrics"]
            src.set_extrinsics(res["matrix_4x4"], method=res["method"], quality=res["quality"],
                               consistency_pct=m["consistency_pct"], rmse_mm=m["rmse_mm"],
                               overlap_points=m["overlap_points"], dt_ms=m["dt_ms"],
                               feature_inliers=m.get("inliers"), source="auto_align")
            out["status"] = "success"
            out["fitness"] = m["consistency_pct"] / 100.0   # compatibilidad con el visor
            out["rmse_mm"] = m["rmse_mm"]
        else:
            out["status"] = "warning"
        return web.json_response(out)
    except Exception as e:  # noqa: BLE001
        import traceback
        traceback.print_exc()
        return web.json_response({"status": "error", "message": str(e)}, status=400)


async def api_get_extrinsics(request):
    src = request.app.get("source")
    mat = src.get_extrinsics() if hasattr(src, "get_extrinsics") else np.eye(4)
    from .registration import matrix_to_euler_translation
    decomp = matrix_to_euler_translation(mat)
    return web.json_response({
        "extrinsics": mat.tolist(),
        "components": decomp
    })


async def api_set_extrinsics(request):
    """Body: {"matrix_4x4": [[...]]} o {"components": {...}}; opcional "calibration": {método y
    métricas} para dejar constancia de cómo se obtuvo."""
    src = request.app.get("source")
    if not hasattr(src, "set_extrinsics"):
        return web.json_response({"status": "error", "message": "No está en modo dual"}, status=400)
    try:
        data = await request.json()
        mat = data.get("matrix_4x4")
        if not mat and "components" in data:
            from .registration import euler_translation_to_matrix
            c = data["components"]
            t = c.get("translation_cm", {})
            r = c.get("rotation_deg", {})
            mat = euler_translation_to_matrix(
                t.get("x", 0.0), t.get("y", 0.0), t.get("z", 0.0),
                r.get("yaw", 0.0), r.get("pitch", 0.0), r.get("roll", 0.0)
            ).tolist()
        if not mat:
            raise ValueError("falta matrix_4x4 o components")
        info = {"method": "ajuste manual", **(data.get("calibration") or {})}
        entry = src.set_extrinsics(mat, **info)
        return web.json_response({"status": "success", "message": "Extrínsecas actualizadas",
                                  "matrix_4x4": entry["matrix_4x4"], "persisted": bool(src.config_path)})
    except (ValueError, TypeError) as e:
        return web.json_response({"status": "error", "message": f"Matriz inválida: {e}"}, status=400)


async def api_extrinsics_history(request):
    """Calibración vigente y anteriores (más reciente primero) con sus componentes legibles."""
    src = request.app.get("source")
    if not hasattr(src, "calibration_history"):
        return web.json_response({"status": "error", "message": "No está en modo dual"}, status=400)
    from .registration import matrix_to_euler_translation
    items = []
    for i, e in enumerate(src.calibration_history()):
        meta = {k: v for k, v in e.items() if k != "matrix_4x4"}
        items.append({"index": i, **meta, "components": matrix_to_euler_translation(e["matrix_4x4"])})
    return web.json_response({"status": "success", "current": src.calibration_info(),
                              "current_components": matrix_to_euler_translation(src.get_extrinsics()),
                              "history": items})


async def api_extrinsics_restore(request):
    """Body: {"index": 0} -> vuelve a esa calibración del historial (0 = deshacer el último cambio)."""
    src = request.app.get("source")
    if not hasattr(src, "restore_extrinsics"):
        return web.json_response({"status": "error", "message": "No está en modo dual"}, status=400)
    try:
        data = await request.json() if request.can_read_body else {}
        entry = src.restore_extrinsics(int(data.get("index", 0)))
    except (IndexError, ValueError, TypeError) as e:
        return web.json_response({"status": "error", "message": str(e)}, status=400)
    return web.json_response({"status": "success", "message": "Calibración restaurada",
                              "matrix_4x4": entry["matrix_4x4"]})


STUDIO_ORIGINS = {"http://localhost:5005", "http://127.0.0.1:5005"}


@web.middleware
async def no_cache(request, handler):
    origin = request.headers.get("Origin")
    if request.method == "OPTIONS" and origin in STUDIO_ORIGINS:
        resp = web.Response(status=204)
    else:
        resp = await handler(request)
    resp.headers["Cache-Control"] = "no-cache"
    if origin in STUDIO_ORIGINS:  # el estudio de calibración (5005) consulta esta API
        resp.headers["Access-Control-Allow-Origin"] = origin
        resp.headers["Access-Control-Allow-Methods"] = "GET, POST, OPTIONS"
        resp.headers["Access-Control-Allow-Headers"] = "Content-Type"
    return resp


def build_app(args):
    app = web.Application(middlewares=[no_cache])
    hub = Hub()
    s = Settings(depth_res=args.depth, color_res=args.color, fps=args.fps, decimation=args.decimation)
    store = SettingsStore(s)
    app["hub"] = hub
    app["store"] = store
    app["multi_session"] = None
    app["multi_lock"] = asyncio.Lock()

    async def on_startup(app):
        app["args_single"] = bool(getattr(args, "single", False))
        if args.synthetic:
            cls = SyntheticDualSource if getattr(args, "dual", False) else SyntheticSource
            msg, fallback = "Simulación forzada (--synthetic)", False
        else:
            cls, msg, fallback = choose_source(app["args_single"])
        await switch_source(app, cls, msg, fallback)
        app["status_task"] = asyncio.create_task(status_task(app))
        app["watch_task"] = asyncio.create_task(device_watch_task(app))

    async def on_shutdown(app):
        for c in list(hub.clients):
            await c.ws.close()

    async def on_cleanup(app):
        app["status_task"].cancel()
        app["watch_task"].cancel()
        await asyncio.get_running_loop().run_in_executor(None, app["source"].stop)

    app.on_startup.append(on_startup)
    app.on_shutdown.append(on_shutdown)
    app.on_cleanup.append(on_cleanup)
    app.router.add_get("/", index)
    app.router.add_get("/ws", ws_handler)
    app.router.add_get("/api/status", api_status)
    app.router.add_get("/api/meta", api_meta)
    app.router.add_post("/api/capture", api_capture)
    app.router.add_post("/api/reload_camera", api_reload_camera)
    app.router.add_post("/api/client_log", api_client_log)

    # Rutas MultiScan
    app.router.add_post("/api/multi/start", api_multi_start)
    app.router.add_post("/api/multi/add", api_multi_add)
    app.router.add_post("/api/multi/undo", api_multi_undo)
    app.router.add_post("/api/multi/discard", api_multi_discard)
    app.router.add_post("/api/multi/finish", api_multi_finish)
    app.router.add_get("/api/multi/status", api_multi_status)

    # Rutas Dual Camera
    app.router.add_get("/api/dual/status", api_dual_status)
    app.router.add_post("/api/dual/select", api_dual_select)
    app.router.add_post("/api/dual/capture", api_dual_capture)
    app.router.add_post("/api/dual/calibrate", api_dual_calibrate)
    app.router.add_post("/api/dual/auto_align", api_dual_auto_align)
    app.router.add_get("/api/dual/extrinsics", api_get_extrinsics)
    app.router.add_post("/api/dual/extrinsics", api_set_extrinsics)
    app.router.add_get("/api/dual/extrinsics/history", api_extrinsics_history)
    app.router.add_post("/api/dual/extrinsics/restore", api_extrinsics_restore)

    # Rutas Saves
    app.router.add_get("/api/saves", api_list_saves)
    app.router.add_static("/saves", SAVES_DIR)

    app.router.add_static("/assets", WEB_DIR)
    return app


def _res(text):
    w, h = text.lower().split("x")
    return int(w), int(h)


def find_available_port(host, start_port, max_tries=10):
    import socket
    target_host = "127.0.0.1" if host in ("0.0.0.0", "localhost") else host
    for port in range(start_port, start_port + max_tries):
        with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
            try:
                s.bind((target_host, port))
                return port
            except OSError:
                continue
    return start_port


def main(argv=None):
    p = argparse.ArgumentParser(description="RealSense Live 3D: streaming de geometría en tiempo real")
    p.add_argument("--host", default="127.0.0.1")
    p.add_argument("--port", type=int, default=5002)
    p.add_argument("--synthetic", action="store_true", help="usar escena sintética (sin cámara)")
    p.add_argument("--depth", type=_res, default=(848, 480), help="resolución de profundidad, p. ej. 848x480")
    p.add_argument("--color", type=_res, default=(848, 480), help="resolución de color, p. ej. 848x480")
    p.add_argument("--fps", type=int, default=30)
    p.add_argument("--decimation", type=int, default=2)
    p.add_argument("--single", action="store_true", help="forzar modo cámara única aunque haya múltiples conectadas")
    p.add_argument("--dual", action="store_true", help="forzar modo cámara dual")
    args = p.parse_args(argv)

    actual_port = find_available_port(args.host, args.port)
    if actual_port != args.port:
        print(f"[Live3D] Puerto {args.port} ocupado/en espera. Usando puerto alternativo {actual_port}.", flush=True)
        args.port = actual_port

    print(f"[Live3D] Modo: {'Sintético' if args.synthetic else ('Cámara Única (--single)' if args.single else 'Auto Dual/Multi-Cámara')}", flush=True)
    print(f"[Live3D] Visor:     http://{'localhost' if args.host in ('127.0.0.1', '0.0.0.0') else args.host}:{args.port}", flush=True)
    print(f"[Live3D] WebSocket: ws://{args.host}:{args.port}/ws", flush=True)
    web.run_app(build_app(args), host=args.host, port=args.port, print=None)


if __name__ == "__main__":
    main()
