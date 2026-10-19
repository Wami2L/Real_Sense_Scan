"""
Estudio de calibración dual (puerto 5005), cliente del servidor Live 3D (puerto 5002).

- Receptor WebSocket persistente del stream dual de 5002 (último cuadro de cada cámara).
- Estado veraz: servidor caído, simulación, cámara caída o congelada (sin cuadros nuevos).
- Galería de tomas emparejadas (solo si ambas cámaras están en vivo y sincronizadas).
- Alineación Cam 2 -> Cam 1 con el motor único live3d/alignment.py (el mismo que usa 5002).
  Solo se aplica y se guarda en 5002 si el resultado es fiable.
- Vistas de verificación: fusión 2D, anaglifo, nivel epipolar y correspondencias.
- Sliders: previsualizan en el visor 3D; guardar es una acción explícita.

NOTA: es transitorio. El plan 10 (Fase 3) lo integra en el servidor 5002 como /studio.
"""

import asyncio
import json
import os
import sys
import time

import aiohttp
import cv2
import numpy as np
from aiohttp import web

BASE_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if BASE_DIR not in sys.path:
    sys.path.insert(0, BASE_DIR)

from live3d import protocol  # noqa: E402
from live3d.alignment import (RGBDView, align_pair, deproject, draw_matches, public_result,  # noqa: E402
                              transform)
from live3d.registration import euler_translation_to_matrix, matrix_to_euler_translation  # noqa: E402

PORT = 5005
LIVE3D = "http://localhost:5002"
STATIC_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), "web")
CAPTURES_DIR = os.path.join(STATIC_DIR, "captures")
LIVE3D_WEB_DIR = os.path.join(BASE_DIR, "live3d", "web")
STALE_S = 1.5          # sin cuadros de una cámara durante más tiempo -> congelada
MAX_PAIR_DT_S = 0.25   # cuadros de las dos cámaras más separados -> no forman una toma
MAX_CAPTURES = 15
os.makedirs(CAPTURES_DIR, exist_ok=True)


class PairUnavailable(RuntimeError):
    pass


class LiveCameraReceiver:
    """Mantiene el último cuadro de cada cámara del stream dual de 5002."""

    def __init__(self, ws_url=LIVE3D.replace("http", "ws", 1) + "/ws"):
        self.ws_url = ws_url
        self.metas = {}
        self.latest = {}          # cam_idx -> (cuadro decodificado, meta, hora de recepción)
        self.connected = False
        self.last_error = None
        self.running = False
        self.fps = 0.0
        self._n, self._t0 = 0, time.time()
        self._seq = 0
        self._preview, self._preview_seq = None, -1

    async def run(self):
        self.running = True
        while self.running:
            try:
                async with aiohttp.ClientSession() as session:
                    async with session.ws_connect(self.ws_url, timeout=10.0, max_msg_size=0) as ws:
                        await ws.send_str(json.dumps({"type": "subscribe", "client": "estudio",
                                                      "depth": "zlib", "color": "jpeg", "ack": False}))
                        self.connected, self.last_error = True, None
                        print("[Receptor] Suscrito al stream dual de 5002.")
                        async for msg in ws:
                            if not self.running:
                                break
                            if msg.type == aiohttp.WSMsgType.TEXT:
                                self._on_text(json.loads(msg.data))
                            elif msg.type == aiohttp.WSMsgType.BINARY:
                                self._on_frame(msg.data)
            except Exception as e:  # noqa: BLE001
                self.last_error = str(e) or e.__class__.__name__
            self.connected = False
            if self.running:
                await asyncio.sleep(1.5)

    def _on_text(self, data):
        t = data.get("type")
        if t == "meta":
            self.metas[data["meta_id"]] = data
        elif t == "source_changed":  # cambió la fuente en 5002: nada de lo anterior es válido
            self.metas.clear()
            self.latest.clear()

    def _on_frame(self, buf):
        h = protocol.parse_header(buf)
        meta = self.metas.get(h["meta_id"])
        if not meta:
            return
        try:
            fr = protocol.decode_frame(buf, meta)
        except ValueError:
            return
        self.latest[h["flags"] & 1] = (fr, meta, time.time())
        self._seq += 1
        self._n += 1
        now = time.time()
        if now - self._t0 >= 1.0:
            self.fps, self._n, self._t0 = self._n / (now - self._t0), 0, now

    def cam_ages(self):
        now = time.time()
        return {i: round(now - self.latest[i][2], 2) for i in self.latest}

    def preview_jpeg(self):
        """Vista lado a lado bajo demanda (solo cuando alguien la mira) y cacheada por cuadro."""
        if self._preview_seq == self._seq and self._preview is not None:
            return self._preview
        ages = self.cam_ages()
        tiles = []
        for i in (0, 1):
            tile = np.full((320, 565, 3), 24, np.uint8)
            if i in self.latest:
                img = cv2.imdecode(np.frombuffer(self.latest[i][0]["color"], np.uint8), cv2.IMREAD_COLOR)
                if img is not None:
                    tile = cv2.resize(img, (565, 320))
            stale = ages.get(i) is None or ages[i] > STALE_S
            label = f"CAM {i + 1} - " + ("SIN SENAL" if i not in ages else
                                          f"CONGELADA ({ages[i]:.0f} s)" if stale else "EN VIVO")
            cv2.rectangle(tile, (0, 0), (565, 32), (18, 18, 24), -1)
            cv2.circle(tile, (16, 16), 6, (60, 60, 230) if stale else (0, 255, 120), -1)
            cv2.putText(tile, label, (30, 22), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255, 255, 255), 1, cv2.LINE_AA)
            tiles.append(tile)
        combined = np.hstack(tiles)
        cv2.line(combined, (565, 0), (565, 320), (0, 200, 255), 2)
        self._preview = cv2.imencode(".jpg", combined, [cv2.IMWRITE_JPEG_QUALITY, 80])[1].tobytes()
        self._preview_seq = self._seq
        return self._preview

    def get_instant_pair(self):
        """Par de vistas RGB-D de ambas cámaras. Lanza PairUnavailable con el motivo concreto."""
        if not self.connected:
            raise PairUnavailable("Sin conexión con el servidor 5002" +
                                  (f" ({self.last_error})" if self.last_error else ""))
        missing = [i for i in (0, 1) if i not in self.latest]
        if missing:
            raise PairUnavailable("No llegan cuadros de " + " ni ".join(f"Cam {i + 1}" for i in missing) +
                                  ". ¿El servidor 5002 está en modo dual con ambas cámaras?")
        ages = self.cam_ages()
        frozen = [i for i in (0, 1) if ages[i] > STALE_S]
        if frozen:
            raise PairUnavailable(", ".join(f"Cam {i + 1} congelada ({ages[i]:.1f} s sin cuadros)" for i in frozen))
        views = []
        for i in (0, 1):
            fr, meta, _ = self.latest[i]
            img = cv2.imdecode(np.frombuffer(fr["color"], np.uint8), cv2.IMREAD_COLOR)
            if img is None:
                raise PairUnavailable(f"Cam {i + 1}: el cuadro llegó sin color")
            views.append(RGBDView.from_meta(img, fr["depth"].copy(), meta, fr["timestamp"], f"Cam {i + 1}"))
        dt = abs(views[0].timestamp - views[1].timestamp)
        if dt > MAX_PAIR_DT_S:
            raise PairUnavailable(f"Cuadros desincronizados: Δt = {dt * 1000:.0f} ms entre cámaras")
        return {"views": views, "dt_ms": round(dt * 1000, 1), "timestamp": time.strftime("%H:%M:%S")}


def compute_camera_alignment_views(view1, view2, T_matrix, alpha=0.5, title="POSE"):
    """Reproyecta Cam 2 en la imagen de color de Cam 1 con T(2->1) (marcos de profundidad):
    fusión 2D, anaglifo (rojo Cam 1 / cian Cam 2) y nivel epipolar lado a lado."""
    img1, img2 = view1.color, view2.color
    ci1, ci2 = view1.cintr, view2.cintr
    h1, w1 = img1.shape[:2]

    P2 = deproject(view2.depth_m(), view2.depth_intr, 0.3, 4.0)          # marco profundidad Cam 2
    # color de cada punto: proyectarlo en la imagen de color de Cam 2
    Pc2 = transform(view2.E, P2)
    u2 = np.round(Pc2[:, 0] / Pc2[:, 2] * ci2["fx"] + ci2["ppx"]).astype(np.int64)
    v2 = np.round(Pc2[:, 1] / Pc2[:, 2] * ci2["fy"] + ci2["ppy"]).astype(np.int64)
    ok = (u2 >= 0) & (u2 < img2.shape[1]) & (v2 >= 0) & (v2 < img2.shape[0])
    P2, u2, v2 = P2[ok], u2[ok], v2[ok]
    # a la imagen de color de Cam 1
    Pc1 = transform(view1.E @ np.asarray(T_matrix, np.float64), P2)
    z = Pc1[:, 2]
    front = z > 0.1
    u1 = np.round(Pc1[front, 0] / z[front] * ci1["fx"] + ci1["ppx"]).astype(np.int64)
    v1 = np.round(Pc1[front, 1] / z[front] * ci1["fy"] + ci1["ppy"]).astype(np.int64)
    zf, u2f, v2f = z[front], u2[front], v2[front]
    inb = (u1 >= 0) & (u1 < w1) & (v1 >= 0) & (v1 < h1)
    order = np.argsort(-zf[inb])  # z-buffer: el más cercano se escribe el último
    uu, vv = u1[inb][order], v1[inb][order]
    warped = np.zeros_like(img1)
    mask = np.zeros((h1, w1), np.uint8)
    warped[vv, uu] = img2[v2f[inb][order], u2f[inb][order]]
    mask[vv, uu] = 255
    k = np.ones((3, 3), np.uint8)
    warped, mask = cv2.dilate(warped, k), cv2.dilate(mask, k)

    overlay = img1.copy()
    m = mask > 0
    overlay[m] = cv2.addWeighted(img1[m], 1.0 - alpha, warped[m], alpha, 0)
    d = matrix_to_euler_translation(T_matrix)
    cv2.rectangle(overlay, (0, 0), (w1, 36), (15, 23, 42), -1)
    cv2.putText(overlay, f"FUSION 2D ({title}): Tx={d['translation_cm']['x']:+.1f}cm Yaw={d['rotation_deg']['yaw']:+.1f}deg",
                (14, 24), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 255, 200), 1, cv2.LINE_AA)

    anaglyph = np.zeros_like(img1)
    anaglyph[:, :, 2] = img1[:, :, 2]
    anaglyph[:, :, 0] = warped[:, :, 0]
    anaglyph[:, :, 1] = warped[:, :, 1]
    cv2.rectangle(anaglyph, (0, 0), (w1, 36), (15, 23, 42), -1)
    cv2.putText(anaglyph, f"ANAGLIFO ({title}): ROJO CAM 1, CIAN CAM 2", (14, 24),
                cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255, 255, 0), 1, cv2.LINE_AA)

    img2r = cv2.resize(img2, (int(img2.shape[1] * h1 / img2.shape[0]), h1))
    epipolar = np.hstack([img1, img2r])
    for y in range(40, h1, 45):
        cv2.line(epipolar, (0, y), (epipolar.shape[1], y), (0, 255, 255), 1, cv2.LINE_AA)
    cv2.rectangle(epipolar, (0, 0), (epipolar.shape[1], 32), (18, 18, 24), -1)
    cv2.putText(epipolar, "NIVEL HORIZONTAL (GUIA DE ALTURAS ENTRE CAM 1 Y CAM 2)", (18, 22),
                cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 255, 255), 1, cv2.LINE_AA)
    return overlay, anaglyph, epipolar


# Estado global
receiver = LiveCameraReceiver()
captures_store = []
active_capture_id = None
latest_alignment_result = {}
capture_counter = 0


# ==========================================
# Comunicación con 5002
# ==========================================
async def live3d_get(path, timeout=1.5):
    try:
        async with aiohttp.ClientSession() as s:
            async with s.get(LIVE3D + path, timeout=aiohttp.ClientTimeout(total=timeout)) as r:
                return await r.json() if r.status == 200 else None
    except Exception:  # noqa: BLE001
        return None


async def live3d_set_extrinsics(matrix_4x4, calibration):
    try:
        async with aiohttp.ClientSession() as s:
            async with s.post(LIVE3D + "/api/dual/extrinsics", json={"matrix_4x4": matrix_4x4,
                                                                     "calibration": calibration},
                              timeout=aiohttp.ClientTimeout(total=3)) as r:
                data = await r.json()
                return r.status == 200 and data.get("status") == "success", data.get("message")
    except Exception as e:  # noqa: BLE001
        return False, str(e)


async def current_calibration():
    """(matriz, info) de la calibración vigente en 5002, o (None, None)."""
    dual = await live3d_get("/api/dual/status")
    if not dual or not dual.get("is_dual"):
        return None, None
    return dual.get("extrinsics"), dual.get("calibration") or {}


def system_state(live, dual):
    """Resumen veraz para la cabecera del estudio."""
    ages = receiver.cam_ages()
    if live is None:
        return {"state": "offline", "label": "Servidor 5002 caído",
                "detail": "Arranca el servidor Live 3D (python app.py).", "can_capture": False}
    cam = live.get("camera", {})
    simulated = bool(cam.get("simulated"))
    if not (dual or {}).get("is_dual"):
        return {"state": "error", "label": "5002 no está en modo dual", "simulated": simulated,
                "detail": cam.get("message", ""), "can_capture": False}
    if cam.get("state") in ("no_device", "error", "starting", "stopped"):
        return {"state": "error" if cam.get("state") != "starting" else "starting",
                "label": {"no_device": "Cámaras no conectadas", "starting": "Abriendo cámaras..."}.get(
                    cam.get("state"), "Error de cámaras"),
                "detail": cam.get("message", ""), "simulated": simulated, "can_capture": False}
    frozen = [i for i in (0, 1) if ages.get(i) is None or ages[i] > STALE_S]
    if frozen:
        return {"state": "stale", "simulated": simulated,
                "label": "Sin cuadros de " + " y ".join(f"Cam {i + 1}" for i in frozen),
                "detail": receiver.last_error or "El estudio no recibe cuadros nuevos del stream.",
                "can_capture": False}
    if cam.get("state") == "degraded":
        return {"state": "degraded", "label": "Solo 1 cámara en vivo", "detail": cam.get("message", ""),
                "simulated": simulated, "can_capture": False}
    if simulated:
        return {"state": "simulated", "label": "Simulación (sin cámaras)", "simulated": True,
                "detail": "No hay RealSense conectadas: las tomas son sintéticas. En cuanto conectes las "
                          "cámaras, 5002 cambiará a ellas automáticamente.", "can_capture": True}
    return {"state": "live", "label": "2 cámaras en vivo", "detail": cam.get("message", ""),
            "simulated": False, "can_capture": True}


def _capture_summary(c):
    keys = ("id", "title", "timestamp", "preview_url", "matches_url", "overlay_url", "anaglyph_url",
            "epipolar_url", "aligned", "fitness_pct", "rmse_mm", "quality", "dt_ms", "simulated")
    return {k: c.get(k) for k in keys}


# ==========================================
# VISOR 3D EMBEBIDO (mismo origen)
# ==========================================
async def handle_viewer3d(request):
    with open(os.path.join(LIVE3D_WEB_DIR, "index.html"), "r", encoding="utf-8") as f:
        return web.Response(text=f.read(), content_type="text/html")


async def handle_assets_main_js(request):
    """main.js del visor apuntando a 5002 y exponiendo window.viewer para los sliders."""
    with open(os.path.join(LIVE3D_WEB_DIR, "js", "main.js"), "r", encoding="utf-8") as f:
        js = f.read()
    js = js.replace('const wsUrl = `${proto}//${location.host}/ws`;', f'const wsUrl = "{LIVE3D.replace("http", "ws", 1)}/ws";')
    js = js.replace("fetch('/api/", f"fetch('{LIVE3D}/api/").replace('fetch("/api/', f'fetch("{LIVE3D}/api/')
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


# ==========================================
# API
# ==========================================
async def api_status(request):
    live, dual = await asyncio.gather(live3d_get("/api/status"), live3d_get("/api/dual/status"))
    pose, calib = None, None
    if dual and dual.get("is_dual"):
        pose = matrix_to_euler_translation(dual["extrinsics"])
        calib = dual.get("calibration")
    return web.json_response({
        "status": "ok",
        "live3d_online": live is not None,
        "system": system_state(live, dual),
        "cams": (dual or {}).get("cams", []),
        "receiver": {"connected": receiver.connected, "fps": round(receiver.fps, 1),
                     "frame_age_s": {f"cam{i + 1}": a for i, a in receiver.cam_ages().items()}},
        "current_pose": pose,
        "calibration": calib,
        "captures_count": len(captures_store),
        "active_capture_id": active_capture_id,
        "captures": [_capture_summary(c) for c in captures_store],
        "has_last_alignment": bool(latest_alignment_result),
        "latest_alignment": latest_alignment_result,
    })


async def api_live_stream(request):
    response = web.StreamResponse(headers={
        "Content-Type": "multipart/x-mixed-replace; boundary=frame",
        "Cache-Control": "no-cache, no-store, must-revalidate"})
    await response.prepare(request)
    try:
        while True:
            jpeg = receiver.preview_jpeg()
            await response.write(b"--frame\r\nContent-Type: image/jpeg\r\n\r\n" + jpeg + b"\r\n")
            await asyncio.sleep(0.066)
    except (ConnectionResetError, asyncio.CancelledError):
        pass
    return response


async def api_live_frame(request):
    return web.Response(body=receiver.preview_jpeg(), content_type="image/jpeg")


async def api_capture(request):
    """Congela una toma emparejada de ambas cámaras (rechaza si alguna no está en vivo)."""
    global capture_counter, active_capture_id
    live = await live3d_get("/api/status")
    try:
        pair = receiver.get_instant_pair()
    except PairUnavailable as e:
        return web.json_response({"status": "error", "message": f"No se puede tomar la captura: {e}"}, status=409)
    if live and live.get("camera", {}).get("state") == "degraded":
        return web.json_response({"status": "error", "message": "Solo una cámara está en vivo: "
                                  + live["camera"].get("message", "")}, status=409)
    simulated = bool((live or {}).get("camera", {}).get("simulated"))

    capture_counter += 1
    cid = capture_counter
    v1, v2 = pair["views"]
    img2r = cv2.resize(v2.color, (int(v2.color.shape[1] * v1.color.shape[0] / v2.color.shape[0]), v1.color.shape[0]))
    preview = np.hstack([v1.color, img2r])
    h1 = v1.color.shape[0]
    cv2.rectangle(preview, (0, h1 - 30), (preview.shape[1], h1), (20, 20, 28), -1)
    cv2.putText(preview, f"TOMA #{cid} {pair['timestamp']} | dt {pair['dt_ms']} ms" + (" | SIMULADA" if simulated else ""),
                (12, h1 - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 230, 255), 1, cv2.LINE_AA)
    fn = f"capture_{cid}_preview.jpg"
    cv2.imwrite(os.path.join(CAPTURES_DIR, fn), preview)

    record = {"id": cid, "title": f"Toma #{cid}", "timestamp": pair["timestamp"],
              "preview_url": f"/captures/{fn}?t={int(time.time() * 1000)}", "aligned": False,
              "fitness_pct": 0, "rmse_mm": 0, "quality": None, "dt_ms": pair["dt_ms"], "simulated": simulated,
              "pair_data": pair}
    captures_store.insert(0, record)
    active_capture_id = cid
    while len(captures_store) > MAX_CAPTURES:
        old = captures_store.pop()
        _remove_capture_files(old["id"])
    return web.json_response({"status": "success",
                              "message": f"Toma #{cid} capturada (Δt {pair['dt_ms']} ms entre cámaras)"
                                         + (" · simulada" if simulated else ""),
                              "capture": _capture_summary(record), "active_capture_id": cid,
                              "total_captures": len(captures_store)})


def _remove_capture_files(cid):
    for suffix in ("preview", "matches", "overlay", "anaglyph", "epipolar"):
        p = os.path.join(CAPTURES_DIR, f"capture_{cid}_{suffix}.jpg")
        if os.path.exists(p):
            try:
                os.remove(p)
            except OSError:
                pass


def _find_capture(cid):
    return next((c for c in captures_store if c["id"] == cid), None)


async def api_align(request):
    """Alinea la toma activa con el motor único. Aplica en 5002 solo si el resultado es fiable."""
    global latest_alignment_result, active_capture_id
    data = {}
    if request.can_read_body:
        try:
            data = await request.json()
        except ValueError:
            pass
    rec = _find_capture(int(data["capture_id"])) if data.get("capture_id") is not None else _find_capture(active_capture_id)
    if rec is None:
        resp = await api_capture(request)
        if resp.status != 200:
            return resp
        rec = captures_store[0]
    active_capture_id = rec["id"]
    v1, v2 = rec["pair_data"]["views"]

    prior, calib = await current_calibration()
    if prior is not None and not (calib or {}).get("is_calibrated"):
        prior = None
    loop = asyncio.get_running_loop()
    res = await loop.run_in_executor(None, align_pair, v1, v2, prior)

    cid, now_ts = rec["id"], int(time.time() * 1000)
    urls = {}
    matches_fn = f"capture_{cid}_matches.jpg"
    cv2.imwrite(os.path.join(CAPTURES_DIR, matches_fn), draw_matches(v1, v2, res))
    urls["matches_url"] = f"/captures/{matches_fn}?t={now_ts}"

    # Vistas de verificación: con la pose nueva si es fiable; si no, con la calibración vigente
    view_T, view_title = (res["matrix_4x4"], "NUEVA") if res["success"] else (prior, "VIGENTE")
    if view_T is not None:
        imgs = await loop.run_in_executor(None, compute_camera_alignment_views, v1, v2, view_T, 0.5, view_title)
        for name, img in zip(("overlay", "anaglyph", "epipolar"), imgs):
            fn = f"capture_{cid}_{name}.jpg"
            cv2.imwrite(os.path.join(CAPTURES_DIR, fn), img)
            urls[f"{name}_url"] = f"/captures/{fn}?t={now_ts}"
        rec["view_T"] = view_T

    metrics = res.get("metrics", {})
    rec.update(urls)
    rec["quality"] = res.get("quality")
    out = public_result(res)
    out.update(urls)
    out["fitness_pct"] = metrics.get("consistency_pct", 0)
    out["rmse_mm"] = metrics.get("rmse_mm")
    out["capture_id"] = cid
    out["timestamp"] = time.strftime("%H:%M:%S")
    if not res["success"]:
        out["status"] = "warning"
        return web.json_response(out)

    calibration = {"method": res["method"], "quality": res["quality"], "source": "estudio",
                   "consistency_pct": metrics.get("consistency_pct"), "rmse_mm": metrics.get("rmse_mm"),
                   "overlap_points": metrics.get("overlap_points"), "dt_ms": metrics.get("dt_ms"),
                   "feature_inliers": metrics.get("inliers"), "simulated": rec.get("simulated", False)}
    applied, msg = await live3d_set_extrinsics(res["matrix_4x4"], calibration)
    rec.update(aligned=True, fitness_pct=out["fitness_pct"], rmse_mm=out["rmse_mm"],
               matrix_4x4=res["matrix_4x4"], components=res["components"])
    out["status"] = "success"
    out["applied"] = applied
    if not applied:
        out["warnings"] = out.get("warnings", []) + [f"No se pudo guardar en 5002: {msg}"]
    latest_alignment_result = {k: out.get(k) for k in (
        "status", "capture_id", "timestamp", "total_time_ms", "fitness_pct", "rmse_mm", "quality",
        "components", "matrix_4x4", "matches_url", "overlay_url", "anaglyph_url", "epipolar_url")}
    return web.json_response(out)


async def api_reproject(request):
    data = await request.json()
    rec = _find_capture(int(data.get("capture_id", active_capture_id or 0)))
    if not rec or rec.get("view_T") is None:
        return web.json_response({"status": "error", "message": "Toma sin vistas de alineación"}, status=400)
    v1, v2 = rec["pair_data"]["views"]
    loop = asyncio.get_running_loop()
    overlay, _, _ = await loop.run_in_executor(None, compute_camera_alignment_views, v1, v2, rec["view_T"],
                                               float(data.get("alpha", 0.5)), "AJUSTE")
    fn = f"capture_{rec['id']}_overlay.jpg"
    cv2.imwrite(os.path.join(CAPTURES_DIR, fn), overlay)
    return web.json_response({"status": "success", "overlay_url": f"/captures/{fn}?t={int(time.time() * 1000)}"})


async def api_select_capture(request):
    global active_capture_id
    data = await request.json()
    rec = _find_capture(int(data.get("id")))
    if not rec:
        return web.json_response({"status": "error", "message": "Captura no encontrada."}, status=404)
    active_capture_id = rec["id"]
    return web.json_response({"status": "success", "active_capture_id": rec["id"], "capture": _capture_summary(rec)})


async def api_delete_capture(request):
    global active_capture_id
    data = await request.json()
    rec = _find_capture(int(data.get("id")))
    if not rec:
        return web.json_response({"status": "error", "message": "No encontrada."}, status=404)
    captures_store.remove(rec)
    _remove_capture_files(rec["id"])
    if active_capture_id == rec["id"]:
        active_capture_id = captures_store[0]["id"] if captures_store else None
    return web.json_response({"status": "success", "active_capture_id": active_capture_id,
                              "remaining_count": len(captures_store)})


def _pose_from_body(data):
    keys = ("tx", "ty", "tz", "yaw", "pitch", "roll")
    missing = [k for k in keys if k not in data]
    if missing:
        raise ValueError(f"faltan {', '.join(missing)}")
    return euler_translation_to_matrix(*(float(data[k]) for k in keys))


async def api_set_manual(request):
    """Pose de los sliders -> matriz para previsualizar en el visor. NO se guarda."""
    try:
        T = _pose_from_body(await request.json())
    except (ValueError, TypeError) as e:
        return web.json_response({"status": "error", "message": str(e)}, status=400)
    rec = _find_capture(active_capture_id)
    if rec is not None:
        rec["view_T"] = T.tolist()  # la fusión 2D de la toma activa refleja el ajuste
    return web.json_response({"status": "success", "components": matrix_to_euler_translation(T),
                              "matrix_4x4": T.tolist()})


async def api_save_pose(request):
    """Guarda en 5002 la pose de los sliders (acción explícita del usuario)."""
    try:
        T = _pose_from_body(await request.json())
    except (ValueError, TypeError) as e:
        return web.json_response({"status": "error", "message": str(e)}, status=400)
    ok, msg = await live3d_set_extrinsics(T.tolist(), {"method": "ajuste manual (estudio)", "source": "estudio"})
    if not ok:
        return web.json_response({"status": "error", "message": f"No se pudo guardar en 5002: {msg}"}, status=502)
    return web.json_response({"status": "success", "message": "Pose manual guardada como calibración vigente."})


async def api_apply(request):
    """Re-aplica en 5002 la última alineación fiable."""
    if not latest_alignment_result.get("matrix_4x4"):
        return web.json_response({"status": "error", "message": "No hay alineación fiable para aplicar."}, status=400)
    ok, msg = await live3d_set_extrinsics(latest_alignment_result["matrix_4x4"],
                                          {"method": "re-aplicada desde el estudio", "source": "estudio",
                                           "quality": latest_alignment_result.get("quality")})
    if ok:
        return web.json_response({"status": "success", "message": "Alineación aplicada en el visor 3D."})
    return web.json_response({"status": "error", "message": f"No se pudo comunicar con 5002: {msg}"}, status=502)


async def handle_index(request):
    return web.FileResponse(os.path.join(STATIC_DIR, "index.html"))


async def on_startup(app):
    app["receiver_task"] = asyncio.create_task(receiver.run())


async def on_cleanup(app):
    receiver.running = False
    app["receiver_task"].cancel()
    try:
        await app["receiver_task"]
    except (asyncio.CancelledError, Exception):  # noqa: BLE001
        pass


@web.middleware
async def no_cache(request, handler):
    resp = await handler(request)
    resp.headers["Cache-Control"] = "no-cache"  # la UI cambia con el código: nunca servir una versión vieja
    return resp


def create_app():
    app = web.Application(middlewares=[no_cache])
    app.on_startup.append(on_startup)
    app.on_cleanup.append(on_cleanup)
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
    app.router.add_post("/api/save_pose", api_save_pose)
    app.router.add_post("/api/apply", api_apply)
    app.router.add_static("/captures", CAPTURES_DIR)
    app.router.add_static("/", STATIC_DIR)
    return app


if __name__ == "__main__":
    print("=" * 70)
    print("  ESTUDIO DE CALIBRACIÓN DUAL · http://localhost:5005 (cliente de 5002)")
    print("=" * 70)
    web.run_app(create_app(), host="0.0.0.0", port=PORT)
