"""Humo de punta a punta: servidor Live3D real en modo sintético dual (sin cámaras).

Lanza `python -m live3d --synthetic --dual` en un puerto libre, recibe cuadros de
ambas cámaras por /ws, consulta la API y hace una captura dual. La carpeta que
crea la captura en Saves/ se borra al terminar.
"""

import asyncio
import json
import os
import shutil

import aiohttp
import pytest

from live3d import protocol
from tools import bench_live3d

ROOT = bench_live3d.ROOT


@pytest.fixture(scope="module")
def server(tmp_path_factory):
    port = bench_live3d.free_port()
    log_path = tmp_path_factory.mktemp("live3d") / "server.log"
    with open(log_path, "w", encoding="utf-8") as log:
        proc = bench_live3d.spawn_server(port, ("--synthetic", "--dual"), log=log)
        base = f"http://127.0.0.1:{port}"
        try:
            asyncio.run(bench_live3d.wait_ready(base, timeout=60))
            yield base
        finally:
            proc.terminate()
            proc.wait(10)


def run(coro):
    return asyncio.run(coro)


async def _get(base, path):
    async with aiohttp.ClientSession() as s:
        async with s.get(base + path) as r:
            return r.status, await r.json()


async def _post(base, path, payload=None, timeout=120):
    async with aiohttp.ClientSession() as s:
        async with s.post(base + path, json=payload, timeout=aiohttp.ClientTimeout(total=timeout)) as r:
            return r.status, await r.json()


def test_status_reporta_fuente_sintetica_dual(server):
    status, body = run(_get(server, "/api/status"))
    assert status == 200
    assert body["camera"]["source"] == "synthetic_dual"
    assert body["camera"]["state"] == "streaming"


def test_stream_entrega_cuadros_de_ambas_camaras(server):
    cams, samples = run(bench_live3d.measure_stream(server, seconds=3.0))
    assert set(cams) == {0, 1}, "deben llegar cuadros de Cam 1 y Cam 2 (flags bit 0)"
    for c in cams.values():
        assert c.frames >= 10
        assert c.resolution is not None, "cada cuadro debe tener su meta"
    assert samples, "el servidor debe enviar mensajes de status"


def test_cuadro_se_decodifica_con_su_meta(server):
    async def one_frame():
        metas = {}
        async with aiohttp.ClientSession() as s:
            async with s.ws_connect(server.replace("http", "ws", 1) + "/ws", max_msg_size=0) as ws:
                await ws.send_str(json.dumps({"type": "subscribe", "depth": "zlib", "color": "raw", "ack": False}))
                async for msg in ws:
                    if msg.type == aiohttp.WSMsgType.TEXT:
                        d = json.loads(msg.data)
                        if d.get("type") == "meta":
                            metas[d["meta_id"]] = d
                    elif msg.type == aiohttp.WSMsgType.BINARY:
                        h = protocol.parse_header(msg.data)
                        if h["meta_id"] in metas:
                            return protocol.decode_frame(msg.data, metas[h["meta_id"]]), metas[h["meta_id"]]

    fr, meta = run(asyncio.wait_for(one_frame(), 15))
    assert fr["depth"].shape == (meta["depth"]["height"], meta["depth"]["width"])
    assert fr["color"].shape == (meta["color"]["height"], meta["color"]["width"], 3)
    assert (fr["depth"] > 0).mean() > 0.5, "la escena sintética debe tener profundidad válida en la mayoría de píxeles"


def test_dual_status_y_extrinsecas(server):
    status, body = run(_get(server, "/api/dual/status"))
    assert status == 200 and body["is_dual"] is True
    assert len(body["extrinsics"]) == 4
    status, body = run(_get(server, "/api/dual/extrinsics"))
    assert status == 200
    assert set(body["components"]) == {"translation_cm", "rotation_deg"}


def test_ajustes_se_aplican_por_websocket(server):
    async def change():
        async with aiohttp.ClientSession() as s:
            async with s.ws_connect(server.replace("http", "ws", 1) + "/ws") as ws:
                await ws.send_str(json.dumps({"type": "settings", "near": 0.4, "far": 2.5}))
                async for msg in ws:
                    if msg.type == aiohttp.WSMsgType.TEXT:
                        d = json.loads(msg.data)
                        if d.get("type") == "status" and d["settings"]["near"] == 0.4:
                            return d["settings"]

    settings = run(asyncio.wait_for(change(), 10))
    assert settings["far"] == 2.5


def test_captura_dual_genera_archivos(server):
    # Sin clientes suscritos: has_demand() también se activa con una petición de captura pendiente.
    status, body = run(_post(server, "/api/dual/capture"))
    assert status == 200, body
    assert body["status"] == "success"
    folder = os.path.join(ROOT, body["rel_folder"])
    try:
        files = set(os.listdir(folder))
        for f in ("nube_fusionada.ply", "nube_cam1.ply", "nube_cam2.ply",
                  "cam1_color.jpg", "cam2_color.jpg", "metadata_dual.json"):
            assert f in files, f"falta {f}"
        with open(os.path.join(folder, "metadata_dual.json"), encoding="utf-8") as fh:
            meta = json.load(fh)
        assert meta["points_fused"] > 1000
    finally:
        shutil.rmtree(folder, ignore_errors=True)
        parent = os.path.dirname(folder)
        if os.path.isdir(parent) and not os.listdir(parent):
            os.rmdir(parent)


def test_status_marca_la_simulacion_y_cada_camara(server):
    _, body = run(_get(server, "/api/status"))
    cam = body["camera"]
    assert cam["simulated"] is True
    assert [c["state"] for c in cam["cams"]] == ["streaming", "streaming"]


def test_auto_align_recupera_la_verdad_sintetica(server):
    import numpy as np
    from live3d.alignment import pose_delta
    status, body = run(_post(server, "/api/dual/auto_align", {}))
    assert status == 200 and body["status"] == "success", body
    gt = np.eye(4)
    gt[0, 3] = 0.25
    assert pose_delta(gt, body["matrix_4x4"])["translation_cm"] < 1.0
    _, dual = run(_get(server, "/api/dual/status"))
    assert dual["calibration"]["persisted"] is False   # la simulación no toca config/ en disco
    assert dual["calibration"]["quality"] == "buena"


def test_extrinsecas_invalidas_se_rechazan(server):
    status, body = run(_post(server, "/api/dual/extrinsics", {"matrix_4x4": [[2, 0, 0, 0], [0, 1, 0, 0],
                                                                            [0, 0, 1, 0], [0, 0, 0, 1]]}))
    assert status == 400 and body["status"] == "error"


def test_cors_permite_al_estudio(server):
    async def preflight():
        async with aiohttp.ClientSession() as s:
            async with s.options(server + "/api/dual/extrinsics",
                                 headers={"Origin": "http://localhost:5005",
                                          "Access-Control-Request-Method": "POST"}) as r:
                return r.status, r.headers.get("Access-Control-Allow-Origin")
    status, origin = run(preflight())
    assert status == 204 and origin == "http://localhost:5005"
