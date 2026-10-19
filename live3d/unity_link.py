"""
Enlace privado con Unity (VR ScanWorld): sirve los objetos de la biblioteca.

    python -m live3d.unity_link                  # solo este PC (127.0.0.1:5010)
    python -m live3d.unity_link --red            # también la red local (exige token)

No necesita cámaras ni el servidor Live 3D. Unity consulta la lista cada pocos segundos y
descarga los modelos nuevos o actualizados (campo "version").

Seguridad: desde este mismo PC no hace falta token. Desde otro equipo (p. ej. un Quest en la
misma wifi) todas las peticiones deben llevar la cabecera  X-ScanWorld-Token: <token>.
El token se genera la primera vez en config/unity_link.json (no se sube a git).

API (JSON salvo los binarios):
  GET  /api/estado                       nombre, versión del protocolo, nº de objetos
  GET  /api/objetos[?todos=1&desde=<t>]  publicados (o todos), opcionalmente solo los cambiados
  GET  /api/objetos/<id>                 manifiesto
  GET  /api/objetos/<id>/modelo.rsm      malla para Unity (se regenera si Blender la cambió)
  GET  /api/objetos/<id>/miniatura.jpg
  POST /api/objetos/<id>/publicar        {"publicado": true|false}
  POST /api/objetos/<id>/nombre          {"nombre": "..."}
"""

import argparse
import hmac
import ipaddress
import json
import os
import secrets
import time

from aiohttp import web

from .objscan.library import Library, ROOT_DIR

PROTOCOL = 1
CONFIG_PATH = os.path.join(ROOT_DIR, "config", "unity_link.json")
DEFAULT_PORT = 5010


def load_config(path=CONFIG_PATH):
    """Config del enlace; crea el token la primera vez."""
    cfg = {}
    if os.path.isfile(path):
        with open(path, encoding="utf-8") as f:
            cfg = json.load(f)
    if not cfg.get("token"):
        cfg["token"] = secrets.token_urlsafe(18)
        cfg.setdefault("puerto", DEFAULT_PORT)
        os.makedirs(os.path.dirname(path), exist_ok=True)
        with open(path, "w", encoding="utf-8") as f:
            json.dump(cfg, f, indent=2)
    return cfg


def _is_local(remote):
    try:
        return ipaddress.ip_address((remote or "").split("%")[0]).is_loopback
    except ValueError:
        return False


def make_auth_middleware(token):
    @web.middleware
    async def auth(request, handler):
        if not _is_local(request.remote):
            given = request.headers.get("X-ScanWorld-Token", "")
            if not hmac.compare_digest(given, token):
                return web.json_response({"error": "token requerido o incorrecto"}, status=401)
        resp = await handler(request)
        resp.headers["Cache-Control"] = "no-cache"
        return resp
    return auth


def _public(man):
    keys = ("id", "nombre", "estado", "publicado", "version", "actualizado", "creado", "tamano_m",
            "vertices", "triangulos", "fuente")
    return {k: man.get(k) for k in keys}


def setup_routes(app, library):
    """Rutas del enlace; también se montan en el servidor Live 3D (5002) para la página /objetos."""
    def _oid(request):
        oid = request.match_info["oid"]
        if not library.exists(oid):
            raise web.HTTPNotFound(text=json.dumps({"error": "objeto no encontrado"}), content_type="application/json")
        return oid

    async def estado(_r):
        return web.json_response({"nombre": "RealSenseScans · enlace Unity", "protocolo": PROTOCOL,
                                  "objetos": len(library.list()), "publicados": len(library.list(True)),
                                  "hora": time.time()})

    async def listar(r):
        todos = r.query.get("todos") in ("1", "true")
        desde = float(r.query["desde"]) if r.query.get("desde") else None
        objs = [_public(m) for m in library.list(published_only=not todos, since=desde)]
        return web.json_response({"objetos": objs, "hora": time.time()})

    async def manifiesto(r):
        return web.json_response(library.get(_oid(r)))

    async def modelo(r):
        oid = _oid(r)
        return web.FileResponse(library.ensure_rsm(oid), headers={"Content-Type": "application/octet-stream"})

    async def miniatura(r):
        p = library.path(_oid(r), "miniatura.jpg")
        if not os.path.isfile(p):
            raise web.HTTPNotFound()
        return web.FileResponse(p)

    async def publicar(r):
        oid = _oid(r)
        data = await r.json() if r.can_read_body else {}
        if data.get("publicado", True):
            library.ensure_rsm(oid)
        return web.json_response(_public(library.publish(oid, bool(data.get("publicado", True)))))

    async def nombre(r):
        oid = _oid(r)
        data = await r.json()
        return web.json_response(_public(library.rename(oid, str(data.get("nombre", "")))))

    async def borrar(r):
        oid = _oid(r)
        library.delete(oid)
        return web.json_response({"borrado": oid})

    app.router.add_get("/api/estado", estado)
    app.router.add_get("/api/objetos", listar)
    app.router.add_get("/api/objetos/{oid}", manifiesto)
    app.router.add_get("/api/objetos/{oid}/modelo.rsm", modelo)
    app.router.add_get("/api/objetos/{oid}/miniatura.jpg", miniatura)
    app.router.add_post("/api/objetos/{oid}/publicar", publicar)
    app.router.add_post("/api/objetos/{oid}/nombre", nombre)
    app.router.add_delete("/api/objetos/{oid}", borrar)


def build_app(library=None, token=None):
    cfg = load_config() if token is None else {"token": token}
    app = web.Application(middlewares=[make_auth_middleware(cfg["token"])])
    setup_routes(app, library or Library())
    return app


def main(argv=None):
    ap = argparse.ArgumentParser(description="Enlace privado con Unity (biblioteca de objetos)")
    ap.add_argument("--red", action="store_true", help="escuchar también en la red local (exige token)")
    ap.add_argument("--puerto", type=int, default=None)
    a = ap.parse_args(argv)
    cfg = load_config()
    port = a.puerto or int(cfg.get("puerto", DEFAULT_PORT))
    host = "0.0.0.0" if a.red else "127.0.0.1"
    lib = Library()
    print(f"[Unity] Biblioteca: {lib.root} ({len(lib.list())} objetos, {len(lib.list(True))} publicados)", flush=True)
    print(f"[Unity] Enlace en http://{'<IP-de-este-PC>' if a.red else '127.0.0.1'}:{port}", flush=True)
    if a.red:
        print(f"[Unity] Token para Unity (cabecera X-ScanWorld-Token): {cfg['token']}", flush=True)
    web.run_app(build_app(lib, cfg["token"]), host=host, port=port, print=None)


if __name__ == "__main__":
    main()
