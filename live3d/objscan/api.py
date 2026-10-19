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
from aiohttp import web

from .base import LAYOUTS
from .library import ROOT_DIR, Library
from .segment import WorkVolume, analyze, preview_image
from .session import ObjectScanSession

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


def setup_objscan_routes(app, library=None):
    lib = library or Library()
    app["objscan"] = None
    app["objscan_lock"] = asyncio.Lock()

    def _session():
        return app.get("objscan")

    def _snap_cam1(snap):
        return snap.get("cam1", snap)

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
            return web.json_response({"error": f"No se pudo leer la cámara: {e}"}, status=503)
        cam1 = _snap_cam1(snap)

        def work():
            info = analyze(cam1, layout, vol, use_base=use_base, scale=s.scale if s else 1.0)
            img = preview_image(cam1, info["mask"], info)
            return info, img
        info, img = await loop.run_in_executor(None, work)
        out = {k: v for k, v in info.items() if k not in ("mask", "T_cam_to_base")}
        out["imagen"] = _jpeg_data_url(img)
        out["dual"] = snap.get("cam2") is not None
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
            snap = await loop.run_in_executor(None, src.get_snapshot, 6.0)
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
