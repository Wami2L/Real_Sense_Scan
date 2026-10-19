"""
Fuente simulada para el escáner de objetos: plato giratorio virtual con la figura sobre la
base de marcadores (python app.py --synthetic-object).

Emite el stream como una cámara real (profundidad alineada al color) y responde a los
snapshots con una imagen a resolución completa. El plato gira solo (GIRO_GRADOS_S) para que
cada toma vea el objeto desde otro ángulo, como si el usuario lo girara.
"""

import time

import numpy as np

from ..capture import BaseSource
from .sintetico import ObjectScene, d455_intrinsics, orbit_pose


class SyntheticObjectSource(BaseSource):
    name = "synthetic_object"
    simulated = True
    GIRO_GRADOS_S = 12.0
    STREAM_FPS = 10

    def __init__(self, store, on_frame, has_demand=lambda: True, log=print):
        super().__init__(store, on_frame, has_demand=has_demand, log=log)
        self.scene = ObjectScene()
        self.cam = orbit_pose(10, elevation_deg=32, distance=0.72)
        self._t0 = time.time()

    def angle(self):
        return ((time.time() - self._t0) * self.GIRO_GRADOS_S) % 360.0

    def _run(self):
        from ..protocol import build_meta
        self._set_status(state="streaming", message="Simulación: objeto en plato giratorio",
                         device="RealSense D455 (objeto simulado)", serial="SIM-OBJ", usb="-",
                         profile="SIM 424x240 · objeto")
        small = d455_intrinsics(424, 240)
        meta = build_meta(1, small, 0.001, small, [1, 0, 0, 0, 1, 0, 0, 0, 1], [0, 0, 0],
                          extra={"device": "D455 (objeto simulado)", "source": self.name, "cam_idx": 0})
        period = 1.0 / self.STREAM_FPS
        while not self._stop.is_set():
            tick = time.perf_counter()
            with self._capture_lock:
                req = self._capture_req
                self._capture_req = None
            if req is not None:
                try:
                    snap = self.scene.render(self.cam, self.angle())
                    snap.pop("gt_T_cam_to_world", None)
                    req["result"] = snap
                except Exception as ex:  # noqa: BLE001
                    req["error"] = ex
                finally:
                    req["event"].set()
            if self.has_demand():
                t0 = time.perf_counter()
                s = self.scene.render(self.cam, self.angle(), intr=small)
                self._emit(s["depth"], s["color"], meta, self.store.get(), t0)
            self._stop.wait(max(0.0, period - (time.perf_counter() - tick)))
