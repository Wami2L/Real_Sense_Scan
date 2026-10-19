"""Convención de extrínsecas: Euler <-> matriz debe ser idéntica en todos los módulos."""

import numpy as np
import pytest

from live3d import registration
from test_fotogrametria import calibrador

CASOS = [
    (0, 0, 0, 0, 0, 0),
    (-95, 6.5, 26, 69, -6, -12),
    (12.3, -4.5, 7.8, -30, 15, 5),
]


@pytest.mark.parametrize("mod", [registration, calibrador], ids=["registration", "calibrador"])
@pytest.mark.parametrize("caso", CASOS)
def test_euler_matriz_ida_y_vuelta(mod, caso):
    T = mod.euler_translation_to_matrix(*caso)
    R = T[:3, :3]
    np.testing.assert_allclose(R @ R.T, np.eye(3), atol=1e-9)
    d = mod.matrix_to_euler_translation(T)
    t, r = d["translation_cm"], d["rotation_deg"]
    got = (t["x"], t["y"], t["z"], r["yaw"], r["pitch"], r["roll"])
    np.testing.assert_allclose(got, caso, atol=0.01)


@pytest.mark.parametrize("caso", CASOS)
def test_registration_y_calibrador_coinciden(caso):
    np.testing.assert_allclose(registration.euler_translation_to_matrix(*caso),
                               calibrador.euler_translation_to_matrix(*caso), atol=1e-12)


@pytest.mark.xfail(strict=True, reason="C12 (plan 10): ScalableTSDFVolume devuelve 0 puntos con Open3D 0.20; "
                                       "la captura dual cae siempre en Poisson")
def test_tsdf_dual_con_identidad_genera_malla():
    """Dos vistas idénticas de un plano a 1 m deben integrarse en una malla no vacía."""
    intr = {"width": 64, "height": 48, "fx": 50.0, "fy": 50.0, "ppx": 31.5, "ppy": 23.5}
    snap = {"color": np.full((48, 64, 3), 128, np.uint8),
            "depth": np.full((48, 64), 1000, np.uint16),
            "intrinsics": intr, "depth_scale": 0.001}
    mesh, _ = registration.integrate_dual_tsdf(snap, snap, np.eye(4), voxel_length=0.01, sdf_trunc=0.04)
    assert len(mesh.triangles) > 0
