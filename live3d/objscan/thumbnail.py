"""Miniatura de una malla sin depender de una GPU: proyección ortográfica con z-buffer de puntos
muestreados sobre la superficie y sombreado simple por normal."""

import math

import numpy as np


def render_thumbnail(mesh, size=320, azimuth_deg=35.0, elevation_deg=25.0, bg=(28, 24, 22)):
    import open3d as o3d
    img = np.zeros((size, size, 3), np.uint8)
    img[:] = bg
    if len(mesh.triangles) == 0:
        return img
    m = o3d.geometry.TriangleMesh(mesh)
    if not m.has_vertex_normals():
        m.compute_vertex_normals()
    n_pts = 250000  # densidad fija: sin huecos entre puntos sea cual sea la malla
    pcd = m.sample_points_uniformly(n_pts)
    P, N = np.asarray(pcd.points), np.asarray(pcd.normals)
    C = np.asarray(pcd.colors) if pcd.has_colors() else np.full((len(P), 3), 0.75)
    a, e = math.radians(azimuth_deg), math.radians(elevation_deg)
    # cámara mirando al objeto desde (sin a, sin e, cos a) del marco base (Y arriba)
    fwd = -np.array([math.sin(a) * math.cos(e), math.sin(e), math.cos(a) * math.cos(e)])
    right = np.cross(fwd, [0, 1, 0])
    right /= np.linalg.norm(right)
    up = np.cross(right, fwd)
    c = (P.max(0) + P.min(0)) / 2
    Q = P - c
    x, y, z = Q @ right, Q @ up, Q @ fwd
    span = max(np.ptp(x), np.ptp(y)) * 1.15 + 1e-9
    u = np.round((x / span + 0.5) * (size - 1)).astype(int)
    v = np.round((0.5 - y / span) * (size - 1)).astype(int)
    light = -fwd * 0.6 + up * 0.6
    light /= np.linalg.norm(light)
    shade = 0.45 + 0.55 * np.clip(N @ light, 0, 1)
    col = np.clip(C * shade[:, None], 0, 1)
    order = np.argsort(-z)  # lejos primero: los cercanos sobrescriben
    for du in (-1, 0, 1):
        for dv in (-1, 0, 1):
            uu, vv = np.clip(u[order] + du, 0, size - 1), np.clip(v[order] + dv, 0, size - 1)
            img[vv, uu] = (col[order][:, ::-1] * 255).astype(np.uint8)  # RGB -> BGR
    return img
