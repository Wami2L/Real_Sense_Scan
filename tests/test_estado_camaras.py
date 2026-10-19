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
