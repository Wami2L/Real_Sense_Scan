"""
Escena sintética para el escáner de objetos (sin cámara): verdad de referencia para tests y
fuente simulada del servidor.

Mesa + hoja de marcadores (la misma que se imprime) + objeto procedural con textura + un
"estorbo" sobre la mesa + pared de fondo. Se renderiza con raycasting de Open3D desde una
pose de cámara conocida, con intrínsecas tipo D455 y ruido de profundidad sigma_z(z).

Dos modos, como en la realidad:
  * plato giratorio: la hoja y el objeto giran (turntable_deg); cámara, mesa y pared quietas.
  * órbita: todo quieto y la cámara se mueve alrededor.
"""

import math
import time

import numpy as np
import open3d as o3d

from ..alignment import D455
from .base import LAYOUTS, render_sheet

BASE_LAYOUT = LAYOUTS["A3"]


def d455_intrinsics(width=848, height=480, hfov_deg=90.0):
    f = (width / 2) / math.tan(math.radians(hfov_deg) / 2)
    return {"width": width, "height": height, "fx": f, "fy": f, "ppx": (width - 1) / 2, "ppy": (height - 1) / 2}


def look_at(eye, target, up=(0.0, 1.0, 0.0)):
    """T cámara -> mundo con la convención RealSense (x derecha, y abajo, z adelante)."""
    eye, target, up = (np.asarray(v, np.float64) for v in (eye, target, up))
    z = target - eye
    z /= np.linalg.norm(z)
    x = np.cross(z, up)
    x /= np.linalg.norm(x)
    y = np.cross(z, x)
    T = np.eye(4)
    T[:3, 0], T[:3, 1], T[:3, 2], T[:3, 3] = x, y, z, eye
    return T


def orbit_pose(azimuth_deg, elevation_deg=30.0, distance=0.75, target=(0.0, 0.07, 0.0)):
    a, e = math.radians(azimuth_deg), math.radians(elevation_deg)
    t = np.asarray(target, np.float64)
    eye = t + distance * np.array([math.sin(a) * math.cos(e), math.sin(e), math.cos(a) * math.cos(e)])
    return look_at(eye, t)


def rot_y(deg):
    a = math.radians(deg)
    R = np.eye(4)
    R[0, 0], R[0, 2], R[2, 0], R[2, 2] = math.cos(a), math.sin(a), -math.sin(a), math.cos(a)
    return R


def _noise3(p, seed=0.0):
    """Textura procedural suave pero no periódica a la escala del objeto (0..1)."""
    x, y, z = p[:, 0] * 40 + seed, p[:, 1] * 40, p[:, 2] * 40
    v = (np.sin(x * 1.13 + np.sin(y * 0.71)) + np.sin(y * 1.37 + np.sin(z * 0.93) * 2)
         + np.sin(z * 0.79 + np.sin(x * 1.71) * 1.5) + 0.5 * np.sin((x + y + z) * 2.3))
    return np.clip(0.5 + v / 7.0, 0, 1)


def make_object():
    """Figura asimétrica (cuerpo, cabeza y asa) con la base en y=0. Devuelve (malla, dimensiones)."""
    body = o3d.geometry.TriangleMesh.create_cylinder(radius=0.05, height=0.10, resolution=96, split=12)
    body.rotate(o3d.geometry.get_rotation_matrix_from_xyz((math.pi / 2, 0, 0)), center=(0, 0, 0))
    body.translate((0, 0.05, 0))
    head = o3d.geometry.TriangleMesh.create_sphere(radius=0.045, resolution=48)
    head.translate((0.0, 0.135, 0.0))
    handle = o3d.geometry.TriangleMesh.create_box(0.05, 0.03, 0.025)
    handle.translate((0.04, 0.045, -0.0125))
    nose = o3d.geometry.TriangleMesh.create_cone(radius=0.015, height=0.04, resolution=32)
    nose.rotate(o3d.geometry.get_rotation_matrix_from_xyz((0, -math.pi / 2, 0)), center=(0, 0, 0))
    nose.translate((0.0, 0.14, -0.04))
    mesh = body + head + handle + nose
    mesh.compute_vertex_normals()
    return mesh


OBJECT_PARTS_COLOR = np.array([0.85, 0.45, 0.20])  # base cálida; la textura modula


class ObjectScene:
    """Escena sintética. `render()` devuelve un snapshot con la forma de los de capture.py."""

    def __init__(self, layout=BASE_LAYOUT, with_clutter=True, seed=0):
        self.layout = layout
        self.obj_mesh = make_object()
        self.sheet = render_sheet(layout, px_per_mm=3.0, guides=True).astype(np.float32) / 255.0
        self.with_clutter = with_clutter
        self.rng = np.random.default_rng(seed)

    # ---- geometría de la escena -----------------------------------------
    def _meshes(self, turntable_deg):
        L = self.layout
        R = rot_y(turntable_deg)
        sheet = o3d.geometry.TriangleMesh.create_box(L.width_mm / 1000, 0.0004, L.height_mm / 1000)
        sheet.translate((-L.width_mm / 2000, -0.0004, -L.height_mm / 2000))
        sheet.transform(R)
        obj = o3d.geometry.TriangleMesh(self.obj_mesh)
        obj.transform(R)
        table = o3d.geometry.TriangleMesh.create_box(1.6, 0.03, 1.2)
        table.translate((-0.8, -0.0304, -0.7))
        wall = o3d.geometry.TriangleMesh.create_box(3.0, 2.0, 0.05)
        wall.translate((-1.5, -0.8, -1.4))
        meshes = {"sheet": sheet, "object": obj, "table": table, "wall": wall}
        if self.with_clutter:
            clutter = o3d.geometry.TriangleMesh.create_box(0.08, 0.12, 0.08)
            clutter.translate((0.30, 0.0, -0.32))
            meshes["clutter"] = clutter
        return meshes

    def _shade(self, kind, P, N, turntable_deg):
        n = len(P)
        if kind == "sheet" or kind == "table":
            Pb = (np.linalg.inv(rot_y(turntable_deg))[:3, :3] @ P.T).T if kind == "sheet" else P
            if kind == "sheet":
                L = self.layout
                u = ((Pb[:, 0] * 1000 + L.width_mm / 2) / L.width_mm * (self.sheet.shape[1] - 1))
                v = ((Pb[:, 2] * 1000 + L.height_mm / 2) / L.height_mm * (self.sheet.shape[0] - 1))
                u = np.clip(np.round(u).astype(int), 0, self.sheet.shape[1] - 1)
                v = np.clip(np.round(v).astype(int), 0, self.sheet.shape[0] - 1)
                g = self.sheet[v, u] * 0.92 + 0.04
                return np.repeat(g[:, None], 3, 1)
            t = _noise3(P * 0.6, seed=3.0)
            return np.column_stack((0.45 + 0.2 * t, 0.30 + 0.12 * t, 0.18 + 0.06 * t))
        if kind == "object":
            Pl = (np.linalg.inv(rot_y(turntable_deg))[:3, :3] @ P.T).T
            t = _noise3(Pl, seed=11.0)
            stripes = 0.5 + 0.5 * np.sin(Pl[:, 1] * 160.0)
            base = np.where((Pl[:, 1] > 0.09)[:, None], [0.25, 0.55, 0.85], OBJECT_PARTS_COLOR)
            return np.clip(base * (0.55 + 0.6 * t)[:, None] * (0.85 + 0.15 * stripes)[:, None], 0, 1)
        if kind == "clutter":
            return np.tile([0.2, 0.7, 0.3], (n, 1)) * (0.6 + 0.4 * _noise3(P, 5.0))[:, None]
        t = _noise3(P * 0.3, seed=7.0)
        return np.column_stack((0.75 * (0.7 + 0.3 * t), 0.75 * (0.7 + 0.3 * t), 0.72 * (0.7 + 0.3 * t)))

    def render(self, T_cam_to_world, turntable_deg=0.0, intr=None, noise=True, sensor=D455):
        intr = intr or d455_intrinsics()
        meshes = self._meshes(turntable_deg)
        scene = o3d.t.geometry.RaycastingScene()
        ids = {}
        for name, m in meshes.items():
            ids[scene.add_triangles(o3d.t.geometry.TriangleMesh.from_legacy(m))] = name
        w, h = intr["width"], intr["height"]
        uu, vv = np.meshgrid(np.arange(w), np.arange(h))
        d_cam = np.stack(((uu - intr["ppx"]) / intr["fx"], (vv - intr["ppy"]) / intr["fy"], np.ones_like(uu, float)), -1)
        R, t = T_cam_to_world[:3, :3], T_cam_to_world[:3, 3]
        d_w = d_cam.reshape(-1, 3) @ R.T
        rays = np.hstack((np.repeat(t[None], len(d_w), 0), d_w)).astype(np.float32)
        ans = scene.cast_rays(o3d.core.Tensor(rays))
        z = ans["t_hit"].numpy().astype(np.float64)       # dirección con z_cam = 1 -> t = profundidad
        gid = ans["geometry_ids"].numpy()
        nrm = ans["primitive_normals"].numpy().astype(np.float64)
        hit = np.isfinite(z)
        P = t + d_w * np.where(hit, z, 0)[:, None]

        color = np.zeros((len(z), 3))
        light = np.array([0.3, 0.9, 0.4])
        light /= np.linalg.norm(light)
        for g, name in ids.items():
            sel = hit & (gid == g)
            if sel.any():
                c = self._shade(name, P[sel], nrm[sel], turntable_deg)
                lam = np.clip(np.abs(nrm[sel] @ light), 0, 1)
                color[sel] = c * (0.55 + 0.45 * lam)[:, None]
        color = (np.clip(color, 0, 1) * 255).astype(np.uint8).reshape(h, w, 3)

        zz = np.where(hit, z, 0.0)
        if noise:
            zz = zz + self.rng.normal(0, 1, zz.shape) * sensor.sigma_z(zz)
        zz[(zz < sensor.z_min_m) | (zz > 6.0)] = 0
        depth = np.clip(np.round(zz * 1000), 0, 65535).astype(np.uint16).reshape(h, w)
        return {"color": color, "depth": depth, "heat": color, "intrinsics": dict(intr), "depth_scale": 0.001,
                "timestamp": time.time(), "gt_T_cam_to_world": T_cam_to_world,
                "gt_turntable_deg": float(turntable_deg)}

    def gt_T_cam_to_base(self, T_cam_to_world, turntable_deg):
        """Pose verdadera de la cámara en el marco de la base (que gira con el plato)."""
        return np.linalg.inv(rot_y(turntable_deg)) @ T_cam_to_world


def object_dimensions():
    """Caja envolvente verdadera del objeto en el marco base (m): (min, max)."""
    m = make_object()
    b = m.get_axis_aligned_bounding_box()
    return np.asarray(b.min_bound), np.asarray(b.max_bound)
