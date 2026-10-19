"""Protocolo binario RSF1: empaquetado, decodificación y deproyección."""

import numpy as np
import pytest

from live3d import protocol

INTR = {"width": 8, "height": 6, "fx": 4.0, "fy": 4.0, "ppx": 3.5, "ppy": 2.5}


def make_meta(meta_id=7):
    return protocol.build_meta(meta_id, INTR, 0.001, INTR)


def test_header_es_de_32_bytes():
    assert protocol.HEADER_SIZE == 32


@pytest.mark.parametrize("depth_codec", ["raw", "zlib"])
def test_ida_y_vuelta_depth_y_color_raw(depth_codec):
    rng = np.random.default_rng(0)
    depth = rng.integers(0, 5000, (6, 8), dtype=np.uint16)
    color = rng.integers(0, 255, (6, 8, 3), dtype=np.uint8)
    meta = make_meta()
    buf = protocol.pack_frame(42, 123.5, meta["meta_id"], depth_codec,
                              protocol.encode_depth(depth, depth_codec), "raw", color.tobytes(), flags=1)
    fr = protocol.decode_frame(buf, meta)
    assert fr["seq"] == 42 and fr["timestamp"] == 123.5 and fr["flags"] == 1
    np.testing.assert_array_equal(fr["depth"], depth)
    np.testing.assert_array_equal(fr["color"], color)


def test_meta_id_distinto_se_rechaza():
    depth = np.zeros((6, 8), np.uint16)
    buf = protocol.pack_frame(1, 0.0, 7, "raw", protocol.encode_depth(depth, "raw"), "none", b"")
    with pytest.raises(ValueError):
        protocol.decode_frame(buf, make_meta(meta_id=8))


def test_magic_invalido_se_rechaza():
    with pytest.raises(ValueError):
        protocol.parse_header(b"XXXX" + bytes(28))


def test_deproject_punto_central():
    meta = make_meta()
    depth = np.zeros((6, 8), np.uint16)
    depth[2, 3] = 1000  # 1 m en el píxel (u=3, v=2)
    pts, mask = protocol.deproject(depth, meta)
    assert mask.sum() == 1
    x, y, z = pts[0]
    assert z == pytest.approx(1.0)
    assert x == pytest.approx((3 - INTR["ppx"]) / INTR["fx"])
    assert y == pytest.approx((2 - INTR["ppy"]) / INTR["fy"])


def test_deproject_respeta_near_far():
    depth = np.zeros((6, 8), np.uint16)
    depth[0, :3] = (100, 1000, 5000)  # 0.1 m, 1 m, 5 m
    _, mask = protocol.deproject(depth, make_meta(), near=0.5, far=2.0)
    assert mask.sum() == 1 and mask[0, 1]
