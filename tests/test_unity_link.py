"""Enlace privado con Unity: listado, descarga del modelo y token desde la red."""

import asyncio
from types import SimpleNamespace

import numpy as np
import open3d as o3d
from aiohttp import web
from aiohttp.test_utils import TestClient, TestServer

from live3d.objscan.library import Library
from live3d.objscan.rsm import HEADER, MAGIC
from live3d.unity_link import build_app, make_auth_middleware


def _lib(tmp_path):
    lib = Library(str(tmp_path / "lib"))
    mesh = o3d.geometry.TriangleMesh.create_box(0.1, 0.2, 0.1)
    mesh.compute_vertex_normals()
    mesh.paint_uniform_color([0.8, 0.2, 0.1])
    return lib, lib.create("Caja roja", mesh)


def test_flujo_listar_publicar_descargar(tmp_path):
    lib, man = _lib(tmp_path)

    async def go():
        client = TestClient(TestServer(build_app(lib, token="secreto")))
        await client.start_server()
        try:
            r = await client.get("/api/objetos")
            assert (await r.json())["objetos"] == []                      # nada publicado aún
            r = await client.get("/api/objetos?todos=1")
            assert [o["id"] for o in (await r.json())["objetos"]] == [man["id"]]
            r = await client.post(f"/api/objetos/{man['id']}/publicar", json={"publicado": True})
            assert (await r.json())["version"] == 2
            r = await client.get("/api/objetos")
            objs = (await r.json())["objetos"]
            assert objs[0]["nombre"] == "Caja roja" and objs[0]["publicado"]
            r = await client.get(f"/api/objetos/{man['id']}/modelo.rsm")
            data = await r.read()
            assert data[:4] == MAGIC and len(data) > HEADER.size
            r = await client.get("/api/objetos/no_existe/modelo.rsm")
            assert r.status == 404
            t = (await (await client.get("/api/objetos")).json())["hora"]
            r = await client.get(f"/api/objetos?desde={t}")
            assert (await r.json())["objetos"] == []                      # solo cambios posteriores
        finally:
            await client.close()

    asyncio.run(go())


def test_desde_la_red_hace_falta_token():
    mw = make_auth_middleware("secreto")

    async def handler(_req):
        return web.json_response({"ok": True})

    async def call(remote, headers):
        return await mw(SimpleNamespace(remote=remote, headers=headers), handler)

    assert asyncio.run(call("127.0.0.1", {})).status == 200
    assert asyncio.run(call("192.168.1.40", {})).status == 401
    assert asyncio.run(call("192.168.1.40", {"X-ScanWorld-Token": "otro"})).status == 401
    assert asyncio.run(call("192.168.1.40", {"X-ScanWorld-Token": "secreto"})).status == 200
