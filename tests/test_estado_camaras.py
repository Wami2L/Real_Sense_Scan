"""Estado por cámara y calibración de las fuentes duales (sin hardware)."""

import json
import time

import numpy as np
import pytest

from live3d.capture import SettingsStore, SyntheticDualSource


def test_sintetica_reporta_simulacion_y_dos_camaras_en_vivo():
    src = SyntheticDualSource(SettingsStore(), lambda f: None)
    src.start()
    try:
        time.sleep(0.5)
        st = src.status()
        assert st["simulated"] is True and st["state"] == "streaming"
        assert [c["state"] for c in st["cams"]] == ["streaming", "streaming"]
    finally:
        src.stop()


def test_sintetica_nunca_escribe_la_calibracion_en_disco():
    src = SyntheticDualSource(SettingsStore(), lambda f: None)
    assert src.config_path is None
    entry = src.set_extrinsics(np.eye(4), method="test")
    assert entry["method"] == "test" and src.calibration_info()["persisted"] is False


def test_snapshot_sintetico_esta_alineado_y_sin_desfase():
    src = SyntheticDualSource(SettingsStore(), lambda f: None)
    src.start()
    try:
        snap = src.get_snapshot(10)
    finally:
        src.stop()
    for k in ("cam1", "cam2"):
        s = snap[k]
        assert s["color"].shape[:2] == s["depth"].shape == (s["intrinsics"]["height"], s["intrinsics"]["width"])
    assert snap["dt_ms"] == 0.0


def test_set_extrinsics_rechaza_rotacion_invalida():
    src = SyntheticDualSource(SettingsStore(), lambda f: None)
    bad = np.eye(4)
    bad[0, 0] = 2.0
    with pytest.raises(ValueError):
        src.set_extrinsics(bad)


def test_dual_real_sin_camaras_reporta_no_conectadas(tmp_path):
    rs = pytest.importorskip("pyrealsense2")
    if len(rs.context().query_devices()) > 0:
        pytest.skip("hay RealSense conectadas")
    from live3d.capture import DualRealSenseSource
    cfg = tmp_path / "dual.json"
    cfg.write_text(json.dumps({"cameras": [{"serial": "000000000001"}, {"serial": "000000000002"}]}))
    src = DualRealSenseSource(SettingsStore(), lambda f: None, config_path=str(cfg))
    src.start()
    try:
        deadline = time.time() + 5
        while time.time() < deadline and src.status()["state"] != "no_device":
            time.sleep(0.1)
        st = src.status()
        assert st["state"] == "no_device", st
        assert all(c["state"] == "no_device" for c in st["cams"])
    finally:
        src.stop()


def test_guardado_atomico_conserva_el_resto_de_la_config(tmp_path):
    rs = pytest.importorskip("pyrealsense2")  # noqa: F841
    from live3d.capture import DualRealSenseSource
    cfg = tmp_path / "dual.json"
    cfg.write_text(json.dumps({"cameras": [{"serial": "A1"}, {"serial": "B2", "emitter_enabled": False}],
                               "extrinsics_cam2_to_cam1": {"matrix_4x4": np.eye(4).tolist(), "fitness": 0.3}}))
    src = DualRealSenseSource(SettingsStore(), lambda f: None, config_path=str(cfg))
    T = np.eye(4)
    T[:3, 3] = (-0.5, 0.02, 0.2)
    src.set_extrinsics(T, method="test", quality="buena")
    saved = json.loads(cfg.read_text(encoding="utf-8"))
    assert saved["cameras"][1]["emitter_enabled"] is False          # no se pierde la config de cámaras
    assert saved["extrinsics_cam2_to_cam1"]["quality"] == "buena"
    assert "fitness" not in saved["extrinsics_cam2_to_cam1"]         # no quedan métricas viejas mezcladas
    assert not (tmp_path / "dual.json.tmp").exists()


def test_historial_de_calibraciones_y_deshacer():
    src = SyntheticDualSource(SettingsStore(), lambda f: None)
    original = src.get_extrinsics().copy()
    A = np.eye(4)
    A[:3, 3] = (0.40, 0.05, 0.40)
    B = np.eye(4)
    B[:3, 3] = (0.45, 0.06, 0.42)
    src.set_extrinsics(A, method="primera", quality="aceptable")
    src.set_extrinsics(B, method="segunda", quality="buena")
    hist = src.calibration_history()
    assert [h.get("method") for h in hist[:2]] == ["primera", "verdad de referencia (simulación)"]
    src.restore_extrinsics(0)                       # deshacer: vuelve a A
    np.testing.assert_allclose(src.get_extrinsics(), A)
    assert src.calibration_info()["method"] == "primera"
    src.restore_extrinsics(0)                       # deshacer otra vez: vuelve a B, no se pierde nada
    np.testing.assert_allclose(src.get_extrinsics(), B)
    src.restore_extrinsics(len(src.calibration_history()) - 1)
    np.testing.assert_allclose(src.get_extrinsics(), original)
    with pytest.raises(IndexError):
        src.restore_extrinsics(99)


def test_historial_se_guarda_en_disco(tmp_path):
    rs = pytest.importorskip("pyrealsense2")  # noqa: F841
    from live3d.capture import DualRealSenseSource
    cfg = tmp_path / "dual.json"
    cfg.write_text(json.dumps({"cameras": [{"serial": "A1"}, {"serial": "B2"}],
                               "extrinsics_cam2_to_cam1": {"matrix_4x4": np.eye(4).tolist(), "is_calibrated": True,
                                                           "method": "vieja"}}))
    src = DualRealSenseSource(SettingsStore(), lambda f: None, config_path=str(cfg))
    T = np.eye(4)
    T[:3, 3] = (0.3, 0.0, 0.1)
    src.set_extrinsics(T, method="nueva")
    saved = json.loads(cfg.read_text(encoding="utf-8"))
    assert saved["extrinsics_history"][0]["method"] == "vieja"
    src.restore_extrinsics(0)
    saved = json.loads(cfg.read_text(encoding="utf-8"))
    assert saved["extrinsics_cam2_to_cam1"]["method"] == "vieja"
    assert saved["extrinsics_history"][0]["method"] == "nueva"


def test_snapshot_dual_con_una_camara_caida_usa_la_otra(tmp_path):
    pytest.importorskip("pyrealsense2")
    import threading
    from live3d.capture import DualRealSenseSource
    cfg = tmp_path / "dual.json"
    cfg.write_text(json.dumps({"cameras": [{"serial": "A1"}, {"serial": "B2"}]}))
    src = DualRealSenseSource(SettingsStore(), lambda f: None, config_path=str(cfg))

    def snap(idx):
        return {"color": np.zeros((4, 4, 3), np.uint8), "depth": np.zeros((4, 4), np.uint16), "heat": None,
                "intrinsics": {"width": 4, "height": 4, "fx": 2, "fy": 2, "ppx": 1.5, "ppy": 1.5},
                "depth_scale": 0.001, "timestamp": 1.0, "arrival_s": 1.0 + idx * 0.01,
                "frame_timestamp_ms": 0.0, "global_time": False, "cam_idx": idx}

    # las dos en vivo: con un solo cuadro todavía no responde
    src._set_cam(0, state="streaming"); src._cam_frame(0)
    src._set_cam(1, state="streaming"); src._cam_frame(1)
    req = {"event": threading.Event(), "result": None, "error": None, "dual_snaps": {0: snap(0)}}
    src._capture_req = req
    src._try_complete(req)
    assert not req["event"].is_set()
    req["dual_snaps"][1] = snap(1)
    src._try_complete(req)
    assert req["event"].is_set() and set(req["result"]) >= {"cam1", "cam2"} and req["result"]["dt_ms"] == 10.0

    # Cam 1 desconectada: responde solo con la Cam 2
    src._set_cam(0, state="no_device")
    req = {"event": threading.Event(), "result": None, "error": None, "dual_snaps": {1: snap(1)}}
    src._capture_req = req
    src._try_complete(req)
    assert req["event"].is_set() and req["result"].get("degraded")
    assert "cam2" in req["result"] and "cam1" not in req["result"]
