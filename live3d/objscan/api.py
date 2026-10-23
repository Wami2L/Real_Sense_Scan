"""
API del escáner de objetos dentro del servidor Live 3D (5002) y su página /objetos.

  GET  /objetos                          página del escáner (pasos: objeto · tomas · procesar · resultado)
  POST /api/objscan/nueva                {nombre, hoja: "A3"|"A4"|"sin_base", radio_cm, altura_cm}
  GET  /api/objscan/estado               sesión actual (o {"activa": false})
  GET  /api/objscan/encuadre             diagnóstico + imagen del encuadre actual (objeto resaltado)
  POST /api/objscan/volumen              {radio_cm, altura_cm}
  POST /api/objscan/tomar                añade una toma (simple o dual)
  POST /api/objscan/deshacer | descartar | procesar
  POST /api/objetos/<id>/blender         abre el objeto en Blender (addon del proyecto)
  + rutas de la biblioteca (las mismas que el enlace Unity, ver live3d/unity_link.py)
"""

import asyncio
import base64
import glob
import os
import subprocess

import cv2
import numpy as np
from aiohttp import web

from .base import LAYOUTS
from .library import ROOT_DIR, Library
from .rafaga import capture_burst
from .optica import downscale_snapshot
from .segment import WorkVolume, analyze, preview_image
from .session import ObjectScanSession, view_angles

WEB_DIR = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "web", "objetos")


def _jpeg_data_url(bgr, quality=78):
    ok, buf = cv2.imencode(".jpg", bgr, [cv2.IMWRITE_JPEG_QUALITY, quality])
    return "data:image/jpeg;base64," + base64.b64encode(buf.tobytes()).decode() if ok else None


def find_blender():
    """Ruta de blender.exe (la versión más nueva instalada) o None."""
    env = os.environ.get("BLENDER_EXE")
    if env and os.path.isfile(env):
        return env
    cands = glob.glob(r"C:\Program Files\Blender Foundation\Blender *\blender.exe")
    cands += glob.glob(os.path.expanduser(r"~\AppData\Local\Programs\Blender Foundation\Blender *\blender.exe"))

    def ver(p):
        try:
            return tuple(int(x) for x in os.path.basename(os.path.dirname(p)).split()[-1].split("."))
        except ValueError:
            return (0,)
    cands.sort(key=ver, reverse=True)
    return cands[0] if cands else None


# Ajustes de profundidad para escanear objetos de cerca (medidos con la D455, plan 14 §7): con
# "high_accuracy" o el láser al máximo, el metal brillante satura el IR y se pierde el objeto.
SCAN_DEPTH_OPTIONS = {"preset": "default", "laser_power": 150}


def setup_objscan_routes(app, library=None):
    app["objscan_T21"] = []          # últimas medidas cam2->cam1 por la hoja (las dos cámaras la ven)
    lib = library or Library()
    app["objscan"] = None
    app["objscan_lock"] = asyncio.Lock()

    def _session():
        return app.get("objscan")

    def _snap_cam1(snap):
        return snap.get("cam1") or snap.get("cam2") or snap

    def _calibration_report(src, T21, res):
        """Calibración dual vigente y, si las dos cámaras ven la hoja, cuánto se aparta de lo que
        miden los marcadores en este cuadro (y esa medida, para aplicarla con un clic)."""
        cfg = (getattr(src, "config", None) or {}).get("extrinsics_cam2_to_cam1") or {}
        rep = {"calibrada": bool(cfg.get("is_calibrated")), "fecha": cfg.get("calibration_date"),
               "metodo": cfg.get("method")}
        Ts = {k: inf["T_cam_to_base"] for k, inf, _ in res if inf.get("origen_pose") == "base"}
        if "cam1" in Ts and "cam2" in Ts:
            meas = np.linalg.inv(Ts["cam1"]) @ Ts["cam2"]          # cam2 -> cam1 según la hoja
            hist = app["objscan_T21"]
            hist.append(meas)
            del hist[:-8]
            if T21 is not None:
                d = np.linalg.inv(np.asarray(T21, np.float64)) @ meas
                rep["diferencia_mm"] = round(float(np.linalg.norm(d[:3, 3])) * 1000, 1)
                rep["diferencia_deg"] = round(float(np.degrees(np.arccos(np.clip((np.trace(d[:3, :3]) - 1) / 2, -1, 1)))), 2)
            rep["medible"] = True
        else:
            app["objscan_T21"].clear()
            rep["medible"] = False
        return rep

    async def page(_r):
        return web.FileResponse(os.path.join(WEB_DIR, "index.html"))

    async def nueva(r):
        data = await r.json() if r.can_read_body else {}
        async with app["objscan_lock"]:
            old = _session()
            if old and old.state == "capturing" and not old.views:
                old.discard()
            hoja = data.get("hoja", "A3")
            vol = WorkVolume(radius_m=float(data.get("radio_cm", 15)) / 100, height_m=float(data.get("altura_cm", 35)) / 100)
            s = ObjectScanSession(data.get("nombre") or "Objeto", layout=hoja if hoja in LAYOUTS else "A3",
                                  volume=vol, use_base=hoja != "sin_base", library=lib)
            app["objscan"] = s
            return web.json_response({"activa": True, **s.summary()})

    async def estado(_r):
        s = _session()
        return web.json_response({"activa": False} if s is None else {"activa": True, **s.summary()})

    async def encuadre(r):
        src = app.get("source")
        if src is None:
            return web.json_response({"error": "sin fuente de cámara"}, status=503)
        s = _session()
        layout = s.layout if s else LAYOUTS.get(r.query.get("hoja", "A3"), LAYOUTS["A3"])
        use_base = s.use_base if s else r.query.get("hoja", "A3") != "sin_base"
        vol = s.volume if s else WorkVolume(float(r.query.get("radio_cm", 15)) / 100,
                                            float(r.query.get("altura_cm", 35)) / 100)
        loop = asyncio.get_running_loop()
        try:
            snap = await loop.run_in_executor(None, src.get_snapshot, 5.0)
        except Exception as e:  # noqa: BLE001
            st = src.status() if hasattr(src, "status") else {}
            why = st.get("message") if st.get("state") not in (None, "streaming") else str(e)
            return web.json_response({"error": f"No se pudo leer la cámara: {why}"}, status=503)
        parts = [(k, snap[k]) for k in ("cam1", "cam2") if snap.get(k) is not None] or [("cam1", snap)]
        T21 = snap.get("extrinsics")
        scale = s.scale if s else 1.0

        def work():
            res = []
            T1 = None
            for k, sp in parts:
                fb = None
                if k == "cam2" and T1 is not None and T21 is not None:
                    fb = T1 @ np.asarray(T21, np.float64) @ (s.cam2_corr if s else np.eye(4))
                info = analyze(downscale_snapshot(sp), layout, vol, use_base=use_base, scale=scale, T_fallback=fb)
                if k == "cam1" and info.get("origen_pose") == "base":
                    T1 = info["T_cam_to_base"]
                res.append((k, info, preview_image(info["snap"], info["mask"], info)))
            return res
        res = await loop.run_in_executor(None, work)
        k0, info, img = res[0]
        out = {k: v for k, v in info.items() if k not in ("mask", "T_cam_to_base", "snap")}
        out["camaras"] = [{"camara": k, "imagen": _jpeg_data_url(im, 72), "origen_pose": inf.get("origen_pose"),
                           "base": inf.get("base"), "ok": inf.get("ok"), "puntos": inf.get("points"),
                           "extent": inf.get("extent"), "distance_m": inf.get("distance_m")}
                          for k, inf, im in res]
        out["calibracion"] = _calibration_report(src, T21, res)
        T = info.get("T_cam_to_base")
        if T is not None and (info.get("base") or {}).get("ok"):
            # para la captura automática: desde dónde mira la cámara y cuánto difiere de lo ya tomado
            out["vista"] = view_angles(T)
            out["vista"]["T"] = np.round(T, 5).tolist()
            if s is not None:
                out["vista"]["novedad_deg"] = round(s.novelty_deg(T), 1)
        out["imagen"] = out["camaras"][0]["imagen"]
        out["dual"] = snap.get("cam1") is not None and snap.get("cam2") is not None
        if snap.get("degraded"):
            off = [c for c in (src.cameras_status() if hasattr(src, "cameras_status") else []) if c.get("state") != "streaming"]
            who = ", ".join(f"Cam {c['idx'] + 1} ({str(c.get('serial', ''))[-4:]})" for c in off) or "Una cámara"
            out["warnings"] = out.get("warnings", []) + [f"{who} no está en vivo (revisa su cable USB 3): se escanea "
                                                         "con la otra."]
        out["simulada"] = bool(getattr(src, "simulated", False))
        return web.json_response(out)

    async def volumen(r):
        s = _session()
        if s is None:
            return web.json_response({"error": "no hay sesión"}, status=400)
        data = await r.json()
        loop = asyncio.get_running_loop()
        vol = await loop.run_in_executor(None, lambda: s.set_volume(
            float(data["radio_cm"]) / 100 if "radio_cm" in data else None,
            float(data["altura_cm"]) / 100 if "altura_cm" in data else None))
        return web.json_response({"volumen": vol})

    async def tomar(_r):
        s, src = _session(), app.get("source")
        if s is None:
            return web.json_response({"error": "Empieza un objeto primero"}, status=400)
        loop = asyncio.get_running_loop()
        try:
            snap = await loop.run_in_executor(None, capture_burst, src, 6, 6.0)
            res = await loop.run_in_executor(None, s.add_snapshot, snap)
        except ValueError as e:
            return web.json_response({"error": str(e)}, status=422)
        except Exception as e:  # noqa: BLE001
            return web.json_response({"error": f"No se pudo tomar: {e}"}, status=500)
        return web.json_response({"toma": res, "sesion": s.summary()})

    async def deshacer(_r):
        s = _session()
        try:
            s.undo()
        except (RuntimeError, AttributeError) as e:
            return web.json_response({"error": str(e)}, status=400)
        return web.json_response(s.summary())

    async def descartar(_r):
        s = _session()
        if s is not None:
            try:
                s.discard()
            except RuntimeError as e:
                return web.json_response({"error": str(e)}, status=400)
        app["objscan"] = None
        return web.json_response({"activa": False})

    async def procesar(_r):
        s = _session()
        try:
            s.finish(background=True)
        except (RuntimeError, AttributeError) as e:
            return web.json_response({"error": str(e)}, status=400)
        return web.json_response(s.summary())

    async def sensor_opts(r):
        """Preset visual, potencia del láser y emisor de las cámaras. Sin cuerpo: los de escaneo."""
        src = app.get("source")
        if src is None or not hasattr(src, "set_depth_options"):
            return web.json_response({"error": "la fuente no admite ajustes de profundidad"}, status=400)
        data = await r.json() if r.can_read_body else {}
        opts = {k: data[k] for k in ("preset", "laser_power", "emitter") if k in data} or SCAN_DEPTH_OPTIONS
        try:
            return web.json_response({"opciones": src.set_depth_options(**opts)})
        except ValueError as e:
            return web.json_response({"error": str(e)}, status=400)

    async def calibrar_con_hoja(_r):
        """Calibración dual con la hoja de escaneo: mediana de las últimas medidas del encuadre en las
        que las DOS cámaras veían los marcadores (pose de cada una respecto a la hoja)."""
        src = app.get("source")
        hist = app["objscan_T21"]
        if len(hist) < 3:
            return web.json_response({"error": "Las dos cámaras tienen que ver la hoja unos segundos"}, status=400)
        if not hasattr(src, "set_extrinsics"):
            return web.json_response({"error": "La fuente actual no es dual"}, status=400)
        Ts = np.stack(hist[-8:])
        t = np.median(Ts[:, :3, 3], axis=0)
        U, _, Vt = np.linalg.svd(Ts[:, :3, :3].mean(0))     # media de rotaciones (proyección a SO(3))
        Rm = U @ Vt
        if np.linalg.det(Rm) < 0:
            U[:, 2] *= -1
            Rm = U @ Vt
        T = np.eye(4)
        T[:3, :3], T[:3, 3] = Rm, t
        spread = float(np.max(np.linalg.norm(Ts[:, :3, 3] - t, axis=1))) * 1000
        try:
            src.set_extrinsics(T, method="hoja de escaneo vista por las dos cámaras (marcadores + plano)",
                               source="objetos", muestras=len(Ts), dispersion_mm=round(spread, 1))
        except ValueError as e:
            return web.json_response({"error": str(e)}, status=400)
        return web.json_response({"ok": True, "muestras": len(Ts), "dispersion_mm": round(spread, 1)})

    async def aplicar_calibracion(_r):
        """Aplica a la calibración dual la corrección que midió el escaneo (Cam 2 vs marcadores)."""
        s, src = _session(), app.get("source")
        info = ((s.result or {}).get("correccion_calibracion_dual") or {}) if s else {}
        if not info.get("aplicada"):
            return web.json_response({"error": "Este escaneo no midió una corrección fiable de la calibración"}, status=400)
        if not hasattr(src, "set_extrinsics"):
            return web.json_response({"error": "La fuente actual no es dual"}, status=400)
        try:
            src.set_extrinsics(info["T_2a1_corregida"], method="corrección por escaneo de objeto (marcadores + ICP)",
                               source="objetos", correccion_mm=info["mm"], correccion_deg=info["deg"],
                               muestras=info["muestras"], objeto=s.result.get("id"))
        except ValueError as e:
            return web.json_response({"error": str(e)}, status=400)
        return web.json_response({"ok": True, "mm": info["mm"], "deg": info["deg"]})

    async def abrir_blender(r):
        oid = r.match_info["oid"]
        if not lib.exists(oid):
            return web.json_response({"error": "objeto no encontrado"}, status=404)
        exe = find_blender()
        if not exe:
            return web.json_response({"error": "No se encontró Blender. Define BLENDER_EXE con la ruta a blender.exe"}, status=404)
        script = os.path.join(ROOT_DIR, "script_blender.py")
        subprocess.Popen([exe, "--python", script, "--", "--objeto", oid], cwd=ROOT_DIR,
                         creationflags=getattr(subprocess, "CREATE_NEW_PROCESS_GROUP", 0))
        return web.json_response({"ok": True, "blender": exe})

    async def hoja_pdf(r):
        name = r.match_info["hoja"]
        if name not in LAYOUTS:
            raise web.HTTPNotFound()
        import sys
        if ROOT_DIR not in sys.path:
            sys.path.insert(0, ROOT_DIR)
        from tools.generar_base_escaneo import base_sheet_pdf
        loop = asyncio.get_running_loop()
        pdf = await loop.run_in_executor(None, base_sheet_pdf, name)
        return web.Response(body=pdf, content_type="application/pdf",
                            headers={"Content-Disposition": f'inline; filename="base_escaneo_{name}.pdf"'})

    from ..unity_link import setup_routes
    setup_routes(app, lib)
    app.router.add_get("/objetos", page)
    app.router.add_get("/objetos-ui/base_escaneo_{hoja}.pdf", hoja_pdf)
    app.router.add_static("/objetos-ui", WEB_DIR)
    app.router.add_post("/api/objscan/nueva", nueva)
    app.router.add_get("/api/objscan/estado", estado)
    app.router.add_get("/api/objscan/encuadre", encuadre)
    app.router.add_post("/api/objscan/volumen", volumen)
    app.router.add_post("/api/objscan/tomar", tomar)
    app.router.add_post("/api/objscan/deshacer", deshacer)
    app.router.add_post("/api/objscan/descartar", descartar)
    app.router.add_post("/api/objscan/procesar", procesar)
    app.router.add_post("/api/objetos/{oid}/blender", abrir_blender)
    app.router.add_post("/api/objscan/aplicar_calibracion", aplicar_calibracion)
    app.router.add_post("/api/objscan/sensor", sensor_opts)
    app.router.add_post("/api/objscan/calibrar_con_hoja", calibrar_con_hoja)
