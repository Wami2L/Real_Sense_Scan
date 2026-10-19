"""API del estudio de calibración (5005) con 5002 simulado: toma, alineación con marcadores,
multi-toma, mapa de error y hoja de marcadores. No abre cámaras ni toca config/."""

import asyncio
import importlib.util
import os

import numpy as np
from aiohttp.test_utils import TestClient, TestServer

from live3d.alignment import pose_delta
from live3d.registration import euler_translation_to_matrix

from test_evidencias import T_MARK, marker_views

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))


def _load_studio(tmp_path, monkeypatch, prior):
    spec = importlib.util.spec_from_file_location("estudio_test", os.path.join(ROOT, "test_fotogrametria", "servidor_test.py"))
    st = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(st)
    views = marker_views()
    saved = []

    async def noop():
        return None

    async def fake_get(path, timeout=1.5):
        if path == "/api/status":
            return {"camera": {"state": "streaming", "simulated": True, "message": "sim"}}
        if path == "/api/dual/status":
            T = saved[-1][0] if saved else prior
            return {"is_dual": True, "extrinsics": np.asarray(T).tolist(), "calibration": {"is_calibrated": True}, "cams": []}
        return None

    async def fake_set(matrix, calibration):
        saved.append((np.asarray(matrix), calibration))
        return True, "ok"

    monkeypatch.setattr(st, "CAPTURES_DIR", str(tmp_path))
    monkeypatch.setattr(st.receiver, "run", noop)
    monkeypatch.setattr(st, "preflight_loop", noop)
    monkeypatch.setattr(st, "live3d_get", fake_get)
    monkeypatch.setattr(st, "live3d_set_extrinsics", fake_set)
    monkeypatch.setattr(st.receiver, "get_instant_pair",
                        lambda: {"views": views, "dt_ms": 0.0, "timestamp": "12:00:00"})
    return st, saved


def test_flujo_del_estudio_con_marcadores(tmp_path, monkeypatch):
    vieja = euler_translation_to_matrix(55, 0, 30, -60, 0, 0)   # calibración guardada del lado contrario
    st, saved = _load_studio(tmp_path, monkeypatch, vieja)

    async def run():
        async with TestClient(TestServer(st.create_app())) as c:
            r = await (await c.post("/api/capture")).json()
            assert r["status"] == "success" and r["capture"]["cam1_url"] and r["capture"]["cam_sizes"]["cam1"] == [848, 480]

            r = await (await c.post("/api/align", json={})).json()
            assert r["status"] == "success" and r["applied"], r.get("message")
            assert r["metrics"]["evidence"] == "marcadores"
            assert pose_delta(T_MARK, r["matrix_4x4"])["translation_cm"] < 1.5
            assert r["error_url"] and r["error_stats"]["pose"] == "nueva"
            assert any(c_["kind"] == "marcador" and c_["ok"] for c_ in r["correspondences"])
            assert saved and saved[-1][1]["evidence"] == "marcadores"
            for k in ("matches", "overlay", "error", "cam1", "cam2"):
                assert (tmp_path / f"capture_1_{k}.jpg").exists()

            await c.post("/api/capture")
            r = await (await c.post("/api/align_multi", json={"capture_ids": [1, 2]})).json()
            assert r["status"] == "success", r.get("message")
            assert r["multi"]["used"] == [1, 2] or sorted(r["multi"]["used"]) == [1, 2]
            assert r["multi"]["dispersion"]["max_translation_cm"] < 0.5
            assert "multi" in saved[-1][1]

            r = await (await c.post("/api/error_map", json={"capture_id": 1})).json()
            assert r["status"] == "success" and r["error_stats"]["median_mm"] < 10

            resp = await c.get("/api/markers.pdf?tam=10&ids=0-1")
            assert resp.status == 200 and resp.content_type == "application/pdf"
            assert (await resp.read())[:4] == b"%PDF"

            st_ = await (await c.get("/api/status")).json()
            cap = next(x for x in st_["captures"] if x["id"] == 1)
            assert cap["detail"]["metrics"]["evidence"] == "marcadores"

    asyncio.run(run())
