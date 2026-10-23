"""Encuadre con las dos cámaras en /objetos: imagen de cada una, de dónde sale su pose, cuánto se
aparta la calibración dual de lo que mide la hoja y calibración con la hoja en un clic."""

import asyncio

import numpy as np
from aiohttp import web
from aiohttp.test_utils import TestClient, TestServer

from live3d.objscan.api import setup_objscan_routes
from live3d.objscan.library import Library
from live3d.objscan.sintetico import ObjectScene, orbit_pose
from live3d.registration import euler_translation_to_matrix


class FakeDualSource:
    """Dos cámaras sintéticas que ven la hoja; la calibración guardada está desajustada."""
    simulated = True

    def __init__(self, error):
        self.sc1, self.sc2 = ObjectScene(seed=1), ObjectScene(seed=2)
        self.T1, self.T2 = orbit_pose(10, elevation_deg=35), orbit_pose(70, elevation_deg=40)
        self.true_T21 = np.linalg.inv(self.T1) @ self.T2
        self.T21 = self.true_T21 @ error
        self.config = {"extrinsics_cam2_to_cam1": {"is_calibrated": True, "calibration_date": "2026-10-23 10:00:00",
                                                   "method": "prueba", "matrix_4x4": self.T21.tolist()}}
        self.saved = None

    def get_snapshot(self, timeout=5.0):
        return {"cam1": self.sc1.render(self.T1, 0), "cam2": self.sc2.render(self.T2, 0), "extrinsics": self.T21}

    def set_extrinsics(self, T, **info):
        self.saved = (np.asarray(T), info)

    def cameras_status(self):
        return [{"idx": 0, "state": "streaming"}, {"idx": 1, "state": "streaming"}]


def _err(A, B):
    d = np.linalg.inv(A) @ B
    return np.linalg.norm(d[:3, 3]) * 1000, np.degrees(np.arccos(np.clip((np.trace(d[:3, :3]) - 1) / 2, -1, 1)))


def test_encuadre_dual_mide_y_corrige_la_calibracion(tmp_path):
    src = FakeDualSource(euler_translation_to_matrix(2, -1, 1, 3, -1, 0.5))  # 2,4 cm y ~3,2°
    app = web.Application()
    app["source"] = src
    setup_objscan_routes(app, Library(str(tmp_path / "lib")))

    async def go():
        client = TestClient(TestServer(app))
        await client.start_server()
        try:
            for _ in range(3):
                d = await (await client.get("/api/objscan/encuadre?hoja=A3")).json()
            cams = d["camaras"]
            assert [c["camara"] for c in cams] == ["cam1", "cam2"]
            assert all(c["imagen"].startswith("data:image/jpeg") and c["origen_pose"] == "base" for c in cams)
            cal = d["calibracion"]
            assert cal["medible"] and cal["fecha"].startswith("2026-10-23")
            assert 15 < cal["diferencia_mm"] < 40 and 2 < cal["diferencia_deg"] < 4.5   # mide el desajuste
            r = await client.post("/api/objscan/calibrar_con_hoja")
            assert r.status == 200, await r.text()
        finally:
            await client.close()

    asyncio.run(go())
    mm, deg = _err(src.saved[0], src.true_T21)
    assert mm < 8 and deg < 0.8                                   # la hoja da la calibración buena


def test_encuadre_cam2_sin_hoja_usa_la_calibracion(tmp_path):
    src = FakeDualSource(np.eye(4))
    src.sc2.sheet[:] = 1.0                                        # la Cam 2 no ve marcadores
    app = web.Application()
    app["source"] = src
    setup_objscan_routes(app, Library(str(tmp_path / "lib")))

    async def go():
        client = TestClient(TestServer(app))
        await client.start_server()
        try:
            d = await (await client.get("/api/objscan/encuadre?hoja=A3")).json()
            assert [c["origen_pose"] for c in d["camaras"]] == ["base", "calibración"]
            assert d["camaras"][1]["ok"]                          # el objeto también aislado en la Cam 2
            assert not d["calibracion"]["medible"]
        finally:
            await client.close()

    asyncio.run(go())
