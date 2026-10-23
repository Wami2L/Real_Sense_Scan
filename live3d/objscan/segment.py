"""
Aislar el objeto en cada vista y preparar la vista previa del encuadre.

Con base de marcadores: el objeto es lo que queda dentro del VOLUMEN DE TRABAJO, un cilindro
vertical sobre el centro de la hoja (radio y altura ajustables), por encima del papel.

Sin base: plano dominante (mesa) por RANSAC y, sobre él, el grupo de puntos más cercano al
centro de la imagen. Ese grupo define un marco provisional (Y = normal de la mesa).
"""

from dataclasses import dataclass

import cv2
import numpy as np
import open3d as o3d

from ..alignment import D455

import dataclasses

# A 848x480 la D455 mide desde ~0.34 m (Min-Z = fx·B/126 px de disparidad = 447·0.095/126); el
# 0.52 m del modelo general es el de 1280x720. Para objetos conviene acercarse (0.4-0.6 m: el
# ruido baja con z² y la foto gana detalle). La cámara ya devuelve 0 donde no puede medir.
D455_OBJ = dataclasses.replace(D455, z_min_m=0.30)
from .base import estimate_pose


@dataclass
class WorkVolume:
    radius_m: float = 0.15
    height_m: float = 0.35
    floor_m: float = 0.004       # por encima del papel (descarta la hoja y su ruido)

    def public(self):
        return {"radio_cm": round(self.radius_m * 100, 1), "altura_cm": round(self.height_m * 100, 1)}


def pixel_points(depth_u16, depth_scale, intr, z_max=3.0, sensor=D455_OBJ):
    """Puntos 3D (marco de color, profundidad alineada) para cada píxel válido + índices."""
    z = depth_u16.astype(np.float64) * depth_scale
    valid = (z >= sensor.z_min_m) & (z <= z_max)
    vv, uu = np.nonzero(valid)
    zz = z[vv, uu]
    P = np.column_stack(((uu - intr["ppx"]) / intr["fx"] * zz, (vv - intr["ppy"]) / intr["fy"] * zz, zz))
    return P, vv, uu


def transform(T, P):
    return P @ T[:3, :3].T + T[:3, 3]


def in_volume(Pb, vol):
    r = np.hypot(Pb[:, 0], Pb[:, 2])
    return (Pb[:, 1] > vol.floor_m) & (Pb[:, 1] < vol.height_m) & (r < vol.radius_m)


def _largest_cluster(P, eps=0.012, min_points=20, prefer=None):
    """Índices del grupo principal (DBSCAN sobre una versión submuestreada)."""
    if len(P) < min_points:
        return np.arange(len(P))
    pcd = o3d.geometry.PointCloud(o3d.utility.Vector3dVector(P))
    down, _, idx_map = pcd.voxel_down_sample_and_trace(eps / 2.5, pcd.get_min_bound(), pcd.get_max_bound())
    labels = np.asarray(down.cluster_dbscan(eps=eps, min_points=8))
    if labels.max() < 0:
        return np.arange(len(P))
    dp = np.asarray(down.points)
    best, best_score = None, None
    for lab in range(labels.max() + 1):
        sel = labels == lab
        n = int(sel.sum())
        if n < 8:
            continue
        score = n if prefer is None else -np.linalg.norm(dp[sel].mean(0)[[0, 2]] - prefer) + 1e-4 * n
        if best_score is None or score > best_score:
            best, best_score = lab, score
    keep_down = np.nonzero(labels == best)[0]
    keep = np.concatenate([np.asarray(idx_map[i]) for i in keep_down])
    return np.unique(keep[keep >= 0])


def segment_with_base(snap, T_cam_to_base, vol, sensor=D455_OBJ):
    """Objeto dentro del volumen de trabajo. Devuelve dict con máscara, puntos (marco base) y colores."""
    intr = snap["intrinsics"]
    P, vv, uu = pixel_points(snap["depth"], snap["depth_scale"], intr, sensor=sensor)
    Pb = transform(T_cam_to_base, P)
    sel = in_volume(Pb, vol)
    idx = np.nonzero(sel)[0]
    if len(idx) > 50:  # dentro del volumen, quedarse con el grupo principal (fuera polvo y bordes)
        idx = idx[_largest_cluster(Pb[idx], prefer=np.zeros(2))]
    mask = np.zeros(snap["depth"].shape, bool)
    mask[vv[idx], uu[idx]] = True
    return {"mask": mask, "points_base": Pb[idx], "points_cam": P[idx],
            "colors": snap["color"][vv[idx], uu[idx]], "edge_frac": _edge_fraction(Pb[idx], vol)}


def _edge_fraction(Pb, vol):
    """Fracción de puntos pegados al borde del volumen: si es alta, el objeto se sale del volumen."""
    if len(Pb) == 0:
        return 0.0
    r = np.hypot(Pb[:, 0], Pb[:, 2])
    return float(np.mean((r > vol.radius_m * 0.93) | (Pb[:, 1] > vol.height_m * 0.95)))


def frame_without_base(snap, sensor=D455_OBJ, z_max=1.6):
    """Marco provisional sin hoja: plano de la mesa + objeto más cercano al centro de la imagen.

    Devuelve (T_cam_to_base, volumen ajustado al objeto) o (None, motivo).
    """
    intr = snap["intrinsics"]
    P, vv, uu = pixel_points(snap["depth"], snap["depth_scale"], intr, z_max=z_max, sensor=sensor)
    if len(P) < 3000:
        return None, "Muy poca profundidad válida en la imagen"
    pcd = o3d.geometry.PointCloud(o3d.utility.Vector3dVector(P)).voxel_down_sample(0.006)
    o3d.utility.random.seed(0)
    # En una habitación el plano más grande suele ser una pared: extraer planos uno tras otro
    # hasta encontrar uno HORIZONTAL (normal hacia arriba de la imagen) con tamaño suficiente.
    up_cam = np.array([0.0, -1.0, 0.0])
    n = d = None
    rest = pcd
    for _ in range(8):
        if len(rest.points) < 500:
            break
        # umbral acorde al ruido del sensor en esa zona (a 1,5 m ~5 mm de sigma)
        plane, inl = rest.segment_plane(distance_threshold=0.01, ransac_n=3, num_iterations=1000)
        cand = np.asarray(plane[:3], np.float64)
        cd = float(plane[3])
        if cand @ up_cam < 0:                 # que la normal apunte "hacia arriba"
            cand, cd = -cand, -cd
        if cand @ up_cam > 0.5 and len(inl) > 0.04 * len(pcd.points):
            # afinar por mínimos cuadrados (RANSAC deja el plano algo inclinado, y con un volumen
            # amplio una inclinación de 1-2° mete trozos de mesa como si fueran objeto)
            Pd = np.asarray(pcd.points)
            sel = np.abs(Pd @ cand + cd) < 0.01
            for _ in range(3):
                Q = Pd[sel]
                c = Q.mean(0)
                cand = np.linalg.svd(Q - c, full_matrices=False)[2][2]
                if cand @ up_cam < 0:
                    cand = -cand
                cd = -float(cand @ c)
                r = Pd @ cand + cd
                sel = np.abs(r) < max(0.004, 2.5 * float(np.std(r[sel])))
            n, d = cand, cd
            break
        rest = rest.select_by_index(inl, invert=True)
    if n is None:
        return None, "No se encuentra una mesa bajo el objeto (¿la cámara mira de frente a una pared?)"
    h = P @ n + d                              # altura sobre la mesa
    # por encima del ruido de la mesa a esa distancia (3 sigma): si no, el ruido "une" objetos
    table = np.abs(h) < 0.01
    z_table = float(np.median(P[table, 2])) if table.any() else 1.0
    h_min = max(0.006, 3.0 * float(sensor.sigma_z(z_table)))
    above = (h > h_min) & (h < 0.5)
    if above.sum() < 300:
        return None, "No hay nada encima de la mesa en la imagen"
    Pa = P[above]
    # de los grupos sobre la mesa, el que se proyecta más cerca del centro de la imagen
    proj = np.column_stack((Pa[:, 0] / Pa[:, 2], Pa[:, 1] / Pa[:, 2]))
    keep = np.arange(len(Pa))
    pcd_a = o3d.geometry.PointCloud(o3d.utility.Vector3dVector(Pa))
    labels = np.asarray(pcd_a.cluster_dbscan(eps=0.015, min_points=15))
    if labels.max() >= 0:
        best, best_d = None, None
        for lab in range(labels.max() + 1):
            sel = labels == lab
            if sel.sum() < 150:
                continue
            dist = float(np.linalg.norm(np.median(proj[sel], 0)))
            if best_d is None or dist < best_d:
                best, best_d = lab, dist
        if best is not None:
            keep = np.nonzero(labels == best)[0]
    obj = Pa[keep]
    c = obj.mean(0)
    origin = c - (c @ n + d) * n               # proyección del centro sobre la mesa
    x = np.array([1.0, 0.0, 0.0]) - n * n[0]
    x /= np.linalg.norm(x)
    z = np.cross(x, n)
    T_base_to_cam = np.eye(4)
    T_base_to_cam[:3, 0], T_base_to_cam[:3, 1], T_base_to_cam[:3, 2], T_base_to_cam[:3, 3] = x, n, z, origin
    T = np.linalg.inv(T_base_to_cam)
    Pb = transform(T, obj)
    # volumen generoso: desde una sola vista solo se ve la mitad del objeto y el centro estimado
    # está desplazado hacia la cámara
    vol = WorkVolume(radius_m=float(np.clip(np.percentile(np.hypot(Pb[:, 0], Pb[:, 2]), 98) * 1.25, 0.04, 0.6)),
                     height_m=float(np.clip(np.percentile(Pb[:, 1], 99) * 1.2, 0.03, 1.0)), floor_m=h_min)
    return T, vol


def object_extent(Pb):
    if len(Pb) < 10:
        return None
    lo, hi = np.percentile(Pb, 1, axis=0), np.percentile(Pb, 99, axis=0)
    return {"ancho_cm": round(float(hi[0] - lo[0]) * 100, 1), "fondo_cm": round(float(hi[2] - lo[2]) * 100, 1),
            "alto_cm": round(float(hi[1] - max(lo[1], 0)) * 100, 1)}


def analyze(snap, layout, vol, use_base=True, scale=1.0, sensor=D455_OBJ, T_fallback=None):
    """Diagnóstico del encuadre de UNA vista: base, objeto, tamaño, distancia y avisos.
    Rectifica la lente primero; devuelve el snapshot rectificado en out["snap"].
    T_fallback: pose a usar si esta cámara no ve la hoja (la Cam 2 con la calibración dual).
    out["origen_pose"]: "base" | "calibración" | "mesa"."""
    from .optica import rectify_snapshot
    snap = rectify_snapshot(snap)
    bgr = cv2.cvtColor(snap["color"], cv2.COLOR_RGB2BGR)
    zmap = snap["depth"].astype(np.float64) * snap["depth_scale"]
    out = {"base": None, "warnings": [], "ok": False, "snap": snap}
    T = None
    if use_base:
        bp = estimate_pose(bgr, snap["intrinsics"], layout, zmap=zmap, scale=scale)
        if bp.ok and bp.scale_ratio and abs(bp.scale_ratio - 1) > 0.015:
            # hoja impresa a otro tamaño (p. ej. "ajustar a página"): la profundidad manda
            fixed = estimate_pose(bgr, snap["intrinsics"], layout, zmap=zmap, scale=scale * bp.scale_ratio)
            if fixed.ok:
                fixed.scale_ratio = bp.scale_ratio * scale
                bp = fixed
        out["base"] = bp.public()
        if bp.sheets > 1:
            out["warnings"].append("Se ven otras copias de la hoja de marcadores (¿en la pared?): se usa la de la "
                                   "mesa, pero es mejor retirarlas o taparlas.")
        if bp.ok and bp.scale_ratio and abs(bp.scale_ratio - 1) > 0.015:
            out["warnings"].append(f"La hoja está impresa al {bp.scale_ratio * 100:.0f} %: se corrige sola, pero "
                                   "lo ideal es imprimirla al 100 % (sin «ajustar a página»).")
        if bp.ok:
            T = bp.T_cam_to_base
            out["origen_pose"] = "base"
    if T is None and T_fallback is not None:
        T = np.asarray(T_fallback, np.float64)
        out["origen_pose"] = "calibración"
    if T is None:
        out["origen_pose"] = "mesa"
        T, v2 = frame_without_base(snap, sensor=sensor)
        if T is None:
            out["warnings"].append(v2)
            out["mask"] = np.zeros(snap["depth"].shape, bool)
            return out
        vol = v2
        out["warnings"].append("Sin base de marcadores a la vista: el objeto se separa por la mesa y la "
                               "alineación será por geometría (menos precisa).")
    seg = segment_with_base(snap, T, vol, sensor=sensor)
    out.update(mask=seg["mask"], T_cam_to_base=T, volume=vol.public(), points=int(len(seg["points_base"])))
    out["extent"] = object_extent(seg["points_base"])
    if len(seg["points_base"]):
        out["distance_m"] = round(float(np.median(seg["points_cam"][:, 2])), 2)
    n = len(seg["points_base"])
    if n < 800:
        out["warnings"].append("Casi no hay objeto dentro del volumen de trabajo: colócalo en el centro de la base "
                               "o amplía el radio/altura.")
    else:
        out["ok"] = True
    if seg["edge_frac"] > 0.08:
        out["warnings"].append("El objeto toca el borde del volumen de trabajo: amplía el radio o la altura.")
    if out.get("distance_m") and out["distance_m"] < 0.36:
        out["warnings"].append("Demasiado cerca: a 848x480 la D455 no mide por debajo de ~0,34 m.")
    if out.get("distance_m") and out["distance_m"] > 0.8:
        out["warnings"].append("Lejos: acércate a 0,4-0,6 m para más detalle (el ruido crece con la distancia² "
                               "y la foto pierde resolución).")
    ext = out.get("extent")
    if ext and max(ext["ancho_cm"], ext["alto_cm"], ext["fondo_cm"]) < 6:
        out["warnings"].append("Objeto muy pequeño para la resolución de la D455 (< 6 cm): saldrá poco detalle.")
    # superficie sin profundidad (negra/brillante/transparente) dentro de la silueta
    if out["ok"]:
        m8 = seg["mask"].astype(np.uint8)
        hull = cv2.morphologyEx(m8, cv2.MORPH_CLOSE, np.ones((15, 15), np.uint8))
        holes = (hull > 0) & (snap["depth"] == 0)
        frac = holes.sum() / max(1, hull.sum())
        out["hole_frac"] = round(float(frac), 3)
        if frac > 0.15:
            out["warnings"].append(f"{frac * 100:.0f} % de la silueta sin profundidad: superficie oscura, brillante o "
                                   "transparente. Un spray mate (o polvo de talco) lo soluciona.")
    return out


def preview_image(snap, mask, info=None):
    """Color con el objeto resaltado (verde, contorno) y el resto atenuado. Devuelve BGR."""
    bgr = cv2.cvtColor(snap["color"], cv2.COLOR_RGB2BGR)
    dim = (bgr * 0.35).astype(np.uint8)
    out = np.where(mask[..., None], bgr, dim)
    if mask.any():
        m8 = cv2.morphologyEx(mask.astype(np.uint8), cv2.MORPH_CLOSE, np.ones((5, 5), np.uint8))
        tint = out.copy()
        tint[m8 > 0] = (0.75 * out[m8 > 0] + 0.25 * np.array([60, 255, 120])).astype(np.uint8)
        out = tint
        cnts, _ = cv2.findContours(m8, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        cv2.drawContours(out, cnts, -1, (80, 255, 140), 2, cv2.LINE_AA)
    if info and info.get("T_cam_to_base") is not None and info.get("volume"):
        _draw_volume(out, snap["intrinsics"], info["T_cam_to_base"], info["volume"])
    return out


def _draw_volume(img, intr, T_cam_to_base, vol_pub):
    """Dibuja el cilindro del volumen de trabajo (base y tapa) sobre la imagen."""
    r, h = vol_pub["radio_cm"] / 100, vol_pub["altura_cm"] / 100
    T_b2c = np.linalg.inv(T_cam_to_base)
    a = np.linspace(0, 2 * np.pi, 64)
    for y, col in ((0.0, (255, 200, 60)), (h, (255, 200, 60))):
        Pb = np.column_stack((r * np.cos(a), np.full_like(a, y), r * np.sin(a)))
        Pc = transform(T_b2c, Pb)
        if np.any(Pc[:, 2] <= 0.05):
            continue
        uv = np.column_stack((Pc[:, 0] / Pc[:, 2] * intr["fx"] + intr["ppx"], Pc[:, 1] / Pc[:, 2] * intr["fy"] + intr["ppy"]))
        cv2.polylines(img, [np.round(uv).astype(np.int32)], True, col, 1, cv2.LINE_AA)
