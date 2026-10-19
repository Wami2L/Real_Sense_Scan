"""
Alineación extrínseca Cam 2 -> Cam 1 por fotogrametría RGB-D, coherente con el sensor.

Un único motor para el visor (5002, /api/dual/auto_align) y el estudio (5005, /api/align).

Por qué así (Intel RealSense D455):
  * La profundidad viene de estéreo activo: el error crece con el cuadrado de la distancia,
    sigma_z ~= z^2 * sigma_disparidad / (f_px * baseline). Con baseline de 95 mm, a 1 m son
    unos pocos mm y a 3 m varios cm. Todos los umbrales del motor salen de este modelo, no de
    constantes fijas.
  * Por debajo del Min-Z (~0.52 m a 848x480) el sensor no mide. Más allá de unos 3 m el
    error es demasiado grande para calibrar con precisión milimétrica.
  * El RGB es global shutter y tiene filtro IR: el patrón del emisor no contamina las
    texturas que usa SIFT.
  * Cada píxel de color tiene profundidad métrica, así que no hay ambigüedad de escala:
    las correspondencias SIFT se elevan a 3D y la pose rígida se resuelve con
    Kabsch + RANSAC, y luego se refina con ICP denso (point-to-plane con kernel robusto).

Qué evita respecto al alineador anterior:
  * No hay poses del montaje físico cableadas en el código ni filtros de "pose admisible"
    que descartaban montajes distintos.
  * Nunca devuelve una pose de respaldo marcada como éxito: si la evidencia no basta,
    success=False y la calibración vigente no se toca.
  * Comprueba la sincronía (dt entre cámaras), que haya profundidad válida y que el
    solape sea suficiente antes de calcular.
  * Evalúa cada candidato con consistencia proyectiva (razonamiento de espacio libre),
    no con un "fitness" que penaliza el solape parcial natural de cámaras convergentes.
"""

import math
import time
from dataclasses import dataclass

import cv2
import numpy as np
import open3d as o3d

INTR_KEYS = ("width", "height", "fx", "fy", "ppx", "ppy")


# --------------------------------------------------------------------------- #
# Modelo del sensor
# --------------------------------------------------------------------------- #
@dataclass(frozen=True)
class SensorModel:
    name: str = "Intel RealSense D455"
    baseline_m: float = 0.095        # separación entre imagers IR
    depth_hfov_deg: float = 87.0
    native_depth_width: int = 848    # resolución de profundidad óptima en D4xx
    subpixel_rms: float = 0.10       # error de disparidad (px); Intel indica ~0.08 con buena textura
    z_min_m: float = 0.52            # Min-Z a 848x480
    z_max_m: float = 3.0             # tope útil para calibrar (error ~2 % a 4 m)

    @property
    def fx_native(self):
        return (self.native_depth_width / 2.0) / math.tan(math.radians(self.depth_hfov_deg) / 2.0)

    def sigma_z(self, z):
        """Desviación típica esperada de la profundidad (m) a distancia z (m)."""
        z = np.asarray(z, dtype=np.float64)
        return z * z * self.subpixel_rms / (self.fx_native * self.baseline_m)


D455 = SensorModel()


@dataclass(frozen=True)
class AlignOptions:
    max_dt_s: float = 0.25           # por encima: cuadros desincronizados -> no se calcula
    warn_dt_s: float = 0.05
    min_valid_depth_frac: float = 0.03
    sift_features: int = 3000
    ratio: float = 0.80
    ransac_iters: int = 3000
    min_feature_inliers: int = 8
    min_feature_inlier_ratio: float = 0.25
    cloud_voxel_m: float = 0.01
    icp_scales: tuple = (0.04, 0.02, 0.01)
    min_threshold_m: float = 0.01    # suelo del umbral de consistencia (bordes, cuantización)
    min_overlap_points: int = 600    # puntos mutuamente visibles y consistentes (vóxel 1 cm)
    # Hipótesis "a ciegas" (FPFH, búsqueda de montaje): al probar muchas poses, alguna encaja por
    # azar en poca superficie. Exigen más zona común (~0.15 m²) y que ninguna pose distinta la
    # explique casi igual de bien.
    min_overlap_points_blind: int = 1500
    ambiguity_margin: float = 0.10
    good_consistency: float = 0.85
    ok_consistency: float = 0.70
    min_overlap_frac: float = 0.03   # fracción mínima de ambas nubes que debe quedar en zona común
    # Un montaje rígido no se mueve solo: alejarse más que esto de la calibración vigente
    # exige evidencia fuerte (calidad buena + fotogrametría coherente) o confirmación manual.
    max_prior_shift_cm: float = 5.0
    max_prior_shift_deg: float = 5.0
    # Búsqueda de montaje: si ninguna hipótesis es buena (cámaras movidas, calibración vieja o
    # inexistente, SIFT débil por convergencia fuerte), se criba una rejilla de poses genéricas.
    mount_search: bool = True
    mount_search_top: int = 3
    mount_search_budget_s: float = 8.0  # tope de tiempo (escenas sin estructura hacen el ICP lento)


# --------------------------------------------------------------------------- #
# Vista RGB-D
# --------------------------------------------------------------------------- #
@dataclass
class RGBDView:
    """Una cámara en un instante.

    color: BGR uint8 (h, w, 3).  depth: uint16 (H, W) en el marco de profundidad.
    color_intr=None y depth_to_color=None significan profundidad ya alineada al color
    (mismo marco e intrínsecas), como en los snapshots de capture.py.
    """
    color: np.ndarray
    depth: np.ndarray
    depth_intr: dict
    depth_scale: float
    color_intr: dict = None
    depth_to_color: np.ndarray = None   # 4x4, p_color = E @ p_depth
    timestamp: float = None
    label: str = "cam"

    @classmethod
    def from_meta(cls, color_bgr, depth, meta, timestamp=None, label="cam"):
        """Desde un cuadro del stream /ws y su mensaje meta (protocol.build_meta)."""
        E = np.eye(4)
        d2c = meta.get("depth_to_color") or {}
        if d2c:
            E[:3, :3] = np.asarray(d2c["rotation"], np.float64).reshape(3, 3)
            E[:3, 3] = np.asarray(d2c["translation"], np.float64)
        ci = meta.get("color")
        return cls(color_bgr, depth, {k: meta["depth"][k] for k in INTR_KEYS}, float(meta["depth"]["scale"]),
                   {k: ci[k] for k in INTR_KEYS} if ci else None, E, timestamp, label)

    @classmethod
    def from_snapshot(cls, snap, label="cam"):
        """Desde un snapshot de capture.py (color RGB, profundidad alineada al color)."""
        return cls(cv2.cvtColor(snap["color"], cv2.COLOR_RGB2BGR), snap["depth"],
                   {k: snap["intrinsics"][k] for k in INTR_KEYS}, float(snap["depth_scale"]),
                   None, None, snap.get("timestamp"), label)

    @property
    def cintr(self):
        return self.color_intr or self.depth_intr

    @property
    def E(self):
        return np.eye(4) if self.depth_to_color is None else np.asarray(self.depth_to_color, np.float64)

    @property
    def is_aligned(self):
        if self.color_intr is None and self.depth_to_color is None:
            return True
        same_intr = self.color_intr is None or all(
            abs(float(self.color_intr[k]) - float(self.depth_intr[k])) < 1e-6 for k in INTR_KEYS)
        return same_intr and np.allclose(self.E, np.eye(4), atol=1e-9)

    def depth_m(self):
        return self.depth.astype(np.float32) * np.float32(self.depth_scale)


# --------------------------------------------------------------------------- #
# Geometría básica
# --------------------------------------------------------------------------- #
def deproject(depth_m, intr, z_min, z_max):
    """Profundidad (m) -> puntos (N, 3) en el marco de la cámara de profundidad."""
    h, w = depth_m.shape
    mask = (depth_m >= z_min) & (depth_m <= z_max)
    vv, uu = np.nonzero(mask)
    z = depth_m[vv, uu].astype(np.float64)
    x = (uu - intr["ppx"]) / intr["fx"] * z
    y = (vv - intr["ppy"]) / intr["fy"] * z
    return np.column_stack((x, y, z))


def transform(T, pts):
    T = np.asarray(T, np.float64)
    return pts @ T[:3, :3].T + T[:3, 3]


def depth_in_color_frame(view, z_min, z_max):
    """Mapa Z (m) en el marco e imagen de color, con z-buffer. 0 = sin dato."""
    dm = view.depth_m()
    ci = view.cintr
    if view.is_aligned and dm.shape == (ci["height"], ci["width"]):
        out = dm.copy()
        out[(out < z_min) | (out > z_max)] = 0
        return out
    pts = transform(view.E, deproject(dm, view.depth_intr, z_min, z_max))
    z = pts[:, 2]
    ok = z > 1e-6
    pts, z = pts[ok], z[ok]
    u = np.round(pts[:, 0] / z * ci["fx"] + ci["ppx"]).astype(np.int64)
    v = np.round(pts[:, 1] / z * ci["fy"] + ci["ppy"]).astype(np.int64)
    inside = (u >= 0) & (u < ci["width"]) & (v >= 0) & (v < ci["height"])
    u, v, z = u[inside], v[inside], z[inside]
    out = np.zeros((ci["height"], ci["width"]), np.float32)
    order = np.argsort(-z)  # el último escrito gana: el más cercano
    out[v[order], u[order]] = z[order]
    return out


def kabsch(A, B, w=None):
    """R, t que minimizan sum w_i ||R A_i + t - B_i||^2."""
    w = np.ones(len(A)) if w is None else np.asarray(w, np.float64)
    w = w / w.sum()
    ca, cb = w @ A, w @ B
    H = ((A - ca) * w[:, None]).T @ (B - cb)
    U, _, Vt = np.linalg.svd(H)
    D = np.diag([1.0, 1.0, np.sign(np.linalg.det(Vt.T @ U.T))])
    R = Vt.T @ D @ U.T
    T = np.eye(4)
    T[:3, :3] = R
    T[:3, 3] = cb - R @ ca
    return T


def rotation_angle_deg(R):
    c = (np.trace(np.asarray(R)[:3, :3]) - 1.0) / 2.0
    return math.degrees(math.acos(float(np.clip(c, -1.0, 1.0))))


def pose_delta(Ta, Tb):
    """Diferencia entre dos poses: traslación (cm) y ángulo de rotación (grados)."""
    Ta, Tb = np.asarray(Ta, np.float64), np.asarray(Tb, np.float64)
    return {"translation_cm": round(float(np.linalg.norm(Ta[:3, 3] - Tb[:3, 3])) * 100, 2),
            "rotation_deg": round(rotation_angle_deg(Ta[:3, :3].T @ Tb[:3, :3]), 2)}


# --------------------------------------------------------------------------- #
# Correspondencias fotogramétricas elevadas a 3D
# --------------------------------------------------------------------------- #
def _gray_clahe(bgr):
    gray = cv2.cvtColor(bgr, cv2.COLOR_BGR2GRAY)
    return cv2.createCLAHE(clipLimit=2.5, tileGridSize=(8, 8)).apply(gray)


def _lift_keypoints(kps, zmap, intr, radius, sensor, min_thr):
    """Eleva keypoints 2D a 3D (marco de color). Descarta los que caen en un borde de
    profundidad (parche no coherente con el ruido esperado) o sin dato."""
    h, w = zmap.shape
    out = np.full((len(kps), 3), np.nan)
    for i, kp in enumerate(kps):
        u, v = int(round(kp.pt[0])), int(round(kp.pt[1]))
        patch = zmap[max(0, v - radius):min(h, v + radius + 1), max(0, u - radius):min(w, u + radius + 1)]
        vals = patch[patch > 0]
        if vals.size == 0:
            continue
        z = float(np.median(vals))
        if float(vals.max() - vals.min()) > max(3.0 * float(sensor.sigma_z(z)), min_thr):
            continue  # discontinuidad: mezcla de primer plano y fondo
        out[i] = ((kp.pt[0] - intr["ppx"]) / intr["fx"] * z, (kp.pt[1] - intr["ppy"]) / intr["fy"] * z, z)
    return out


def match_features(view1, view2, sensor, opt):
    """SIFT + ratio de Lowe + verificación mutua; devuelve pares 3D en el marco de PROFUNDIDAD."""
    sift = cv2.SIFT_create(nfeatures=opt.sift_features, contrastThreshold=0.02)
    g1, g2 = _gray_clahe(view1.color), _gray_clahe(view2.color)
    kp1, d1 = sift.detectAndCompute(g1, None)
    kp2, d2 = sift.detectAndCompute(g2, None)
    info = {"keypoints_cam1": len(kp1), "keypoints_cam2": len(kp2), "matches": 0, "lifted": 0}
    empty = (np.empty((0, 3)), np.empty((0, 3)), [], kp1, kp2, info)
    if d1 is None or d2 is None or len(kp1) < 8 or len(kp2) < 8:
        return empty

    bf = cv2.BFMatcher(cv2.NORM_L2)
    fwd = bf.knnMatch(d1, d2, k=2)
    back = {m.queryIdx: m.trainIdx for m in bf.match(d2, d1)}
    matches = [p[0] for p in fwd if len(p) == 2 and p[0].distance < opt.ratio * p[1].distance
               and back.get(p[0].trainIdx) == p[0].queryIdx]
    info["matches"] = len(matches)
    if len(matches) < 3:
        return empty

    z1 = depth_in_color_frame(view1, sensor.z_min_m, sensor.z_max_m)
    z2 = depth_in_color_frame(view2, sensor.z_min_m, sensor.z_max_m)
    r1 = max(1, int(math.ceil(view1.cintr["width"] / view1.depth_intr["width"])))
    r2 = max(1, int(math.ceil(view2.cintr["width"] / view2.depth_intr["width"])))
    P1 = _lift_keypoints([kp1[m.queryIdx] for m in matches], z1, view1.cintr, r1, sensor, opt.min_threshold_m)
    P2 = _lift_keypoints([kp2[m.trainIdx] for m in matches], z2, view2.cintr, r2, sensor, opt.min_threshold_m)
    ok = ~(np.isnan(P1).any(1) | np.isnan(P2).any(1))
    matches = [m for m, k in zip(matches, ok) if k]
    # marco de color -> marco de profundidad (el que usan el visor y la captura)
    P1 = transform(np.linalg.inv(view1.E), P1[ok])
    P2 = transform(np.linalg.inv(view2.E), P2[ok])
    info["lifted"] = len(matches)
    return P1, P2, matches, kp1, kp2, info


def _match_thresholds(P1, P2, sensor, view1, view2, min_thr):
    """Umbral por correspondencia: ruido de profundidad (axial) + 1 px de localización (lateral)."""
    s1, s2 = sensor.sigma_z(P1[:, 2]), sensor.sigma_z(P2[:, 2])
    lat1 = P1[:, 2] / view1.cintr["fx"]
    lat2 = P2[:, 2] / view2.cintr["fx"]
    return np.maximum(min_thr, 3.0 * np.sqrt(s1 ** 2 + s2 ** 2 + lat1 ** 2 + lat2 ** 2))


def ransac_rigid(P2, P1, thr, iters, rng, sensor):
    """T(2->1) robusto con RANSAC de 3 puntos y reajuste ponderado (1/sigma^2)."""
    n = len(P1)
    if n < 3:
        return None, np.zeros(n, bool)
    best_inl = np.zeros(n, bool)
    for _ in range(iters):
        idx = rng.choice(n, 3, replace=False)
        a = P1[idx]
        if np.linalg.norm(np.cross(a[1] - a[0], a[2] - a[0])) < 1e-4:  # casi colineales
            continue
        T = kabsch(P2[idx], a)
        inl = np.linalg.norm(transform(T, P2) - P1, axis=1) < thr
        if inl.sum() > best_inl.sum():
            best_inl = inl
            if inl.sum() > 0.9 * n:
                break
    if best_inl.sum() < 3:
        return None, best_inl
    T = None
    for _ in range(3):  # reajuste ponderado sobre inliers
        w = 1.0 / (sensor.sigma_z(P1[best_inl, 2]) ** 2 + sensor.sigma_z(P2[best_inl, 2]) ** 2 + 1e-8)
        T = kabsch(P2[best_inl], P1[best_inl], w)
        inl = np.linalg.norm(transform(T, P2) - P1, axis=1) < thr
        if inl.sum() < 3 or np.array_equal(inl, best_inl):
            break
        best_inl = inl
    return T, best_inl


# --------------------------------------------------------------------------- #
# Nubes densas, registro y evaluación
# --------------------------------------------------------------------------- #
def _cloud(view, sensor, voxel):
    pts = deproject(view.depth_m(), view.depth_intr, sensor.z_min_m, sensor.z_max_m)
    pcd = o3d.geometry.PointCloud(o3d.utility.Vector3dVector(pts))
    pcd = pcd.voxel_down_sample(voxel)
    pcd.estimate_normals(o3d.geometry.KDTreeSearchParamHybrid(radius=voxel * 3, max_nn=30))
    pcd.orient_normals_towards_camera_location(np.zeros(3))
    return pcd


def _pyramid(pcd, scales):
    return {s: pcd.voxel_down_sample(s) for s in scales}


def refine_icp(pyr_src, pyr_tgt, T, scales):
    reg = o3d.pipelines.registration
    T = np.asarray(T, np.float64)
    for s in scales:
        src, tgt = pyr_src[s], pyr_tgt[s]
        if len(src.points) < 50 or len(tgt.points) < 50:
            continue
        est = reg.TransformationEstimationPointToPlane(reg.TukeyLoss(k=s * 1.5))
        r = reg.registration_icp(src, tgt, s * 2.5, T, est,
                                 reg.ICPConvergenceCriteria(relative_fitness=1e-6, relative_rmse=1e-6,
                                                            max_iteration=40))
        if r.fitness > 0:
            T = np.asarray(r.transformation)
    return T


def global_fpfh(pcd_src, pcd_tgt, voxel):
    """Registro global solo con geometría (FPFH + RANSAC): no depende de textura ni de una pose previa."""
    reg = o3d.pipelines.registration
    src, tgt = pcd_src.voxel_down_sample(voxel), pcd_tgt.voxel_down_sample(voxel)
    for p in (src, tgt):
        p.estimate_normals(o3d.geometry.KDTreeSearchParamHybrid(radius=voxel * 2.5, max_nn=30))
        p.orient_normals_towards_camera_location(np.zeros(3))
    feat = o3d.geometry.KDTreeSearchParamHybrid(radius=voxel * 5, max_nn=100)
    fs, ft = reg.compute_fpfh_feature(src, feat), reg.compute_fpfh_feature(tgt, feat)
    dist = voxel * 1.5
    res = reg.registration_ransac_based_on_feature_matching(
        src, tgt, fs, ft, True, dist, reg.TransformationEstimationPointToPoint(False), 3,
        [reg.CorrespondenceCheckerBasedOnEdgeLength(0.9), reg.CorrespondenceCheckerBasedOnDistance(dist)],
        reg.RANSACConvergenceCriteria(100000, 0.999))
    return np.asarray(res.transformation)


def projective_consistency(T_src_to_tgt, pts_src, normals_src, view_tgt, zmap_tgt, sensor, min_thr):
    """Proyecta los puntos de una cámara en la imagen de profundidad de la otra y compara.

    consistente: |z_pred - z_medida| <= umbral (3 sigma de ambas medidas) y la superficie
                 mira hacia la cámara destino (una superficie vista por ambas cámaras debe
                 estar de cara a las dos; descarta soluciones espejo "detrás de la pared").
    violación:   z_pred < z_medida - umbral (el punto flota delante de una superficie que la
                 otra cámara SÍ ve) o superficie coincidente pero de espaldas.
    oculto:      z_pred > z_medida + umbral (detrás de algo: oclusión legítima, no cuenta).
    """
    T = np.asarray(T_src_to_tgt, np.float64)
    P = transform(T, pts_src)
    N = normals_src @ T[:3, :3].T
    di = view_tgt.depth_intr
    z = P[:, 2]
    front = z > sensor.z_min_m * 0.5
    P, N, z = P[front], N[front], z[front]
    u = np.round(P[:, 0] / z * di["fx"] + di["ppx"]).astype(np.int64)
    v = np.round(P[:, 1] / z * di["fy"] + di["ppy"]).astype(np.int64)
    inside = (u >= 0) & (u < di["width"]) & (v >= 0) & (v < di["height"])
    u, v, z, P, N = u[inside], v[inside], z[inside], P[inside], N[inside]
    zm = zmap_tgt[v, u].astype(np.float64)
    seen = zm > 0
    z, zm, P, N = z[seen], zm[seen], P[seen], N[seen]
    # sigma acotada al rango del sensor: un punto predicho a decenas de metros no puede tener
    # una tolerancia de metros (quedaría "consistente" con cualquier cosa); cuenta como oculto
    zs = np.minimum(z, sensor.z_max_m)
    thr = np.maximum(min_thr, 3.0 * np.sqrt(sensor.sigma_z(zs) ** 2 + sensor.sigma_z(zm) ** 2))
    d = z - zm
    facing = np.einsum("ij,ij->i", N, -P) / np.maximum(np.linalg.norm(P, axis=1), 1e-9) > -0.2
    close = np.abs(d) <= thr
    cons = close & facing
    viol = (d < -thr) | (close & ~facing)
    hidden = d > thr
    return int(cons.sum()), int(viol.sum()), int(hidden.sum()), d[cons]


def evaluate(T, cloud1, cloud2, view1, view2, zd1, zd2, sensor, opt, P1=None, P2=None, thr=None):
    """cloud = (puntos, normales orientadas hacia su cámara), en el marco de profundidad."""
    c21, v21, h21, r21 = projective_consistency(T, *cloud2, view1, zd1, sensor, opt.min_threshold_m)
    c12, v12, h12, r12 = projective_consistency(np.linalg.inv(T), *cloud1, view2, zd2, sensor, opt.min_threshold_m)
    cons, viol, hid = c21 + c12, v21 + v12, h21 + h12
    res = np.concatenate([r21, r12])
    out = {
        "consistency": cons / max(1, cons + viol),
        "overlap_points": cons,
        "violation_points": viol,
        "hidden_points": hid,
        "overlap_frac": cons / max(1, len(cloud1[0]) + len(cloud2[0])),  # se cuenta en ambos sentidos
        "rmse_mm": float(np.sqrt(np.mean(res ** 2)) * 1000) if res.size else float("nan"),
        "feature_agreement": None,
    }
    if P1 is not None and len(P1):
        out["feature_agreement"] = float(np.mean(np.linalg.norm(transform(T, P2) - P1, axis=1) < thr))
    return out


def plausible_pose(T, sensor):
    """Pose rígida y con las cámaras a una distancia compatible con ver una zona común
    (dos cámaras a más de 2·z_max no pueden medir la misma superficie)."""
    T = np.asarray(T, np.float64)
    if T.shape != (4, 4) or not np.all(np.isfinite(T)):
        return False
    R = T[:3, :3]
    if not np.allclose(R @ R.T, np.eye(3), atol=1e-3) or np.linalg.det(R) < 0.99:
        return False
    return float(np.linalg.norm(T[:3, 3])) <= 2.0 * sensor.z_max_m


def mount_search_seeds():
    """Poses genéricas de dos cámaras que miran a una zona común: lado a lado en paralelo o
    convergentes (la cámara desplazada a un lado gira hacia el otro), a ambos lados. No es la
    pose de ningún montaje concreto: cubre el espacio de montajes razonables para el ICP."""
    from .registration import euler_translation_to_matrix
    poses = [(0.0, 0.0, 0.0)]
    # cada pose se prueba a ambos lados seguidos: si el tope de tiempo corta la criba, no queda un
    # lado entero sin explorar
    for tx, tz, yaw in [(tx, 0.0, 0.0) for tx in (15.0, 30.0)] +             [(tx, tz, yaw) for tx in (45.0, 25.0, 65.0, 90.0) for yaw in (60.0, 45.0, 75.0, 30.0, 90.0)
             for tz in (25.0, 0.0, 50.0)]:
        poses += [(tx, tz, -yaw), (-tx, tz, yaw)]
    return [euler_translation_to_matrix(tx, 0.0, tz, yaw, 0.0, 0.0) for tx, tz, yaw in poses]


def mount_search(pyr_src, pyr_tgt, view1, view2, zd1, zd2, sensor, opt):
    """Criba la rejilla de mount_search_seeds() con ICP grueso y consistencia proyectiva sobre
    nubes reducidas. Devuelve las mejores poses distintas (sin refinar), cuántas se cribaron y
    cuántas había (menos si se agotó el tope de tiempo)."""
    reg = o3d.pipelines.registration
    coarse, mid = opt.icp_scales[0], opt.icp_scales[min(1, len(opt.icp_scales) - 1)]
    src, tgt = pyr_src[coarse], pyr_tgt[coarse]
    if len(src.points) < 50 or len(tgt.points) < 50:
        return [], 0
    q1 = (np.asarray(pyr_tgt[mid].points), np.asarray(pyr_tgt[mid].normals))
    q2 = (np.asarray(pyr_src[mid].points), np.asarray(pyr_src[mid].normals))
    est = reg.TransformationEstimationPointToPlane(reg.TukeyLoss(k=coarse * 1.5))
    crit = reg.ICPConvergenceCriteria(max_iteration=25)
    min_ov = max(1, opt.min_overlap_points // 4)  # la nube cribada tiene ~4x menos puntos
    scored = []
    seeds = mount_search_seeds()
    t0, n = time.time(), 0
    for T0 in seeds:
        if time.time() - t0 > opt.mount_search_budget_s:
            break
        n += 1
        T = np.asarray(reg.registration_icp(src, tgt, coarse * 3.0, T0, est, crit).transformation)
        if not plausible_pose(T, sensor):
            continue
        ev = evaluate(T, q1, q2, view1, view2, zd1, zd2, sensor, opt)
        scored.append((ev["consistency"] * min(1.0, ev["overlap_points"] / min_ov), T))
    scored.sort(key=lambda x: -x[0])
    best = []
    for _, T in scored:
        if all(pose_delta(T, B)["translation_cm"] > 3.0 or pose_delta(T, B)["rotation_deg"] > 3.0 for B in best):
            best.append(T)
        if len(best) >= opt.mount_search_top:
            break
    return best, n, len(seeds)


# --------------------------------------------------------------------------- #
# Motor
# --------------------------------------------------------------------------- #
def _fail(message, warnings, t0, **extra):
    return {"success": False, "quality": "insuficiente", "message": message, "warnings": warnings,
            "total_time_ms": round((time.time() - t0) * 1000, 1), **extra}


def _depth_coverage(view, sensor):
    dm = view.depth_m()
    return float(((dm >= sensor.z_min_m) & (dm <= sensor.z_max_m)).mean())


def align_pair(view1, view2, prior_T=None, sensor=D455, opt=AlignOptions(), seed=0, extra_seeds=()):
    """Calcula T(2->1) en el marco de PROFUNDIDAD de cada cámara.

    prior_T: calibración vigente (semilla y referencia de estabilidad del montaje).
    extra_seeds: [(nombre, T)] hipótesis adicionales, p. ej. la pose de los sliders. No cuentan
    como calibración vigente: solo dan un punto de partida al ICP.

    Devuelve un dict con success, quality ('buena' | 'aceptable' | 'insuficiente'), message,
    matrix_4x4, metrics, candidates, warnings, delta_vs_prior y total_time_ms. Con
    success=False no se incluye matrix_4x4: el llamador debe conservar la calibración vigente.
    """
    from .registration import matrix_to_euler_translation  # convención única de Euler

    t0 = time.time()
    warnings = []

    # 1. Sincronía ---------------------------------------------------------
    dt_ms = None
    if view1.timestamp is not None and view2.timestamp is not None:
        dt = abs(float(view1.timestamp) - float(view2.timestamp))
        dt_ms = round(dt * 1000, 1)
        if dt > opt.max_dt_s:
            return _fail(f"Cuadros desincronizados (Δt = {dt_ms} ms entre cámaras). "
                         "Una cámara está congelada o se perdió señal: revisa el estado de ambas.",
                         warnings, t0, metrics={"dt_ms": dt_ms})
        if dt > opt.warn_dt_s:
            warnings.append(f"Δt entre cámaras de {dt_ms} ms: mantén la escena quieta durante la toma.")

    # 2. Profundidad útil en el rango del sensor ----------------------------
    cov = {}
    for name, v in (("Cam 1", view1), ("Cam 2", view2)):
        cov[name] = _depth_coverage(v, sensor)
        if cov[name] < opt.min_valid_depth_frac:
            return _fail(f"{name}: solo {cov[name] * 100:.1f} % de píxeles con profundidad entre "
                         f"{sensor.z_min_m:.2f} y {sensor.z_max_m:.1f} m. Acerca la escena (el {sensor.name} "
                         f"no mide por debajo de {sensor.z_min_m:.2f} m) o comprueba el emisor IR.",
                         warnings, t0, metrics={"dt_ms": dt_ms})
        if float(cv2.cvtColor(v.color, cv2.COLOR_BGR2GRAY).mean()) < 25:
            warnings.append(f"{name}: imagen muy oscura, habrá pocas texturas para SIFT.")

    # 3. Correspondencias SIFT elevadas a 3D + RANSAC -----------------------
    rng = np.random.default_rng(seed)
    o3d.utility.random.seed(seed)  # RANSAC de Open3D determinista: mismo par -> mismo resultado
    P1, P2, matches, kp1, kp2, finfo = match_features(view1, view2, sensor, opt)
    thr = _match_thresholds(P1, P2, sensor, view1, view2, opt.min_threshold_m) if len(P1) else np.empty(0)
    T_feat, inliers = ransac_rigid(P2, P1, thr, opt.ransac_iters, rng, sensor) if len(P1) >= 3 else (None, np.zeros(0, bool))
    n_inl = int(inliers.sum())
    finfo["inliers"] = n_inl
    inl_ratio = n_inl / max(1, len(P1))
    finfo["inlier_ratio_pct"] = round(inl_ratio * 100, 1)
    feature_strong = (T_feat is not None and n_inl >= opt.min_feature_inliers
                      and inl_ratio >= opt.min_feature_inlier_ratio)
    if not feature_strong:
        why = ("texturas repetitivas o poco distintivas" if len(P1) >= opt.min_feature_inliers
               else "poca textura común con profundidad válida")
        warnings.append(f"Fotogrametría débil ({n_inl} de {len(P1)} correspondencias 3D coherentes: {why}). "
                        "La pose depende sobre todo de la geometría; añade objetos con textura única en la zona común.")

    # 4. Nubes densas (marco de profundidad) y candidatos -------------------
    pcd1 = _cloud(view1, sensor, opt.cloud_voxel_m)
    pcd2 = _cloud(view2, sensor, opt.cloud_voxel_m)
    pts1, pts2 = np.asarray(pcd1.points), np.asarray(pcd2.points)
    cloud1, cloud2 = (pts1, np.asarray(pcd1.normals)), (pts2, np.asarray(pcd2.normals))
    pyr1, pyr2 = _pyramid(pcd1, opt.icp_scales), _pyramid(pcd2, opt.icp_scales)
    zd1 = view1.depth_m().copy()
    zd2 = view2.depth_m().copy()
    for zd in (zd1, zd2):
        zd[(zd < sensor.z_min_m) | (zd > sensor.z_max_m)] = 0

    seeds = []
    if T_feat is not None and n_inl >= 3:
        seeds.append(("SIFT 3D + RANSAC", T_feat))
    if prior_T is not None:
        seeds.append(("calibración vigente", np.asarray(prior_T, np.float64)))
    seeds += [(name, np.asarray(T, np.float64)) for name, T in extra_seeds]
    blind_seeds = []
    try:
        blind_seeds.append(("FPFH global (geometría)", global_fpfh(pcd2, pcd1, 0.02)))
    except RuntimeError as e:
        warnings.append(f"Registro global FPFH falló: {e}")

    candidates = []

    def add_candidate(name, T0, blind=False):
        T = refine_icp(pyr2, pyr1, T0, opt.icp_scales)
        if not plausible_pose(T, sensor):
            return
        ev = evaluate(T, cloud1, cloud2, view1, view2, zd1, zd2, sensor, opt, P1[inliers] if n_inl else None,
                      P2[inliers] if n_inl else None, thr[inliers] if n_inl else None)
        candidates.append({"name": name, "T": T, "blind": blind, **ev})

    def is_viable(c):
        min_pts = opt.min_overlap_points_blind if c["blind"] else opt.min_overlap_points
        return c["overlap_points"] >= min_pts and c["overlap_frac"] >= opt.min_overlap_frac

    for name, T0 in seeds:
        add_candidate(name, T0)
    for name, T0 in blind_seeds:
        add_candidate(name, T0, blind=True)
    if opt.mount_search and not any(is_viable(c) and c["consistency"] >= opt.good_consistency for c in candidates):
        t_ms = time.time()
        found, n_done, n_seeds = mount_search(pyr2, pyr1, view1, view2, zd1, zd2, sensor, opt)
        for i, T0 in enumerate(found):
            add_candidate(f"búsqueda de montaje #{i + 1}", T0, blind=True)
        finfo["mount_search"] = {"seeds": n_done, "of": n_seeds, "time_ms": round((time.time() - t_ms) * 1000, 1)}
        if n_done < n_seeds:
            warnings.append(f"Búsqueda de montaje cortada por tiempo ({n_done} de {n_seeds} hipótesis): la escena "
                            "tiene poca estructura o mucho ruido de profundidad.")

    viable = [c for c in candidates if is_viable(c)]
    summary = [{"name": c["name"], "consistency_pct": round(c["consistency"] * 100, 1),
                "overlap_points": c["overlap_points"], "hidden_points": c["hidden_points"],
                "overlap_frac_pct": round(c["overlap_frac"] * 100, 1), "rmse_mm": round(c["rmse_mm"], 2) if c["rmse_mm"] == c["rmse_mm"] else None,
                "feature_agreement_pct": None if c["feature_agreement"] is None else round(c["feature_agreement"] * 100, 1)}
               for c in candidates]
    base_metrics = {"dt_ms": dt_ms, "depth_coverage_pct": {k: round(v * 100, 1) for k, v in cov.items()}, **finfo}
    if not viable:
        return _fail("Solape insuficiente entre cámaras: ninguna hipótesis deja zona común suficiente. "
                     "Coloca el sujeto u objetos donde lo vean AMBAS cámaras, entre "
                     f"{sensor.z_min_m:.2f} y {sensor.z_max_m:.1f} m, sin taparlas de cerca.",
                     warnings, t0, candidates=summary, metrics=base_metrics)

    # 5. Selección: consistencia; con fotogrametría fuerte, que también la respete --
    def score(c):
        s = c["consistency"]
        if feature_strong and c["feature_agreement"] is not None:
            s = 0.7 * s + 0.3 * c["feature_agreement"]
        return s

    best = max(viable, key=score)
    # Estabilidad: si la calibración vigente refinada es igual de buena, no se mueve la pose
    prior_c = next((c for c in viable if c["name"] == "calibración vigente"), None)
    if prior_c is not None and prior_c is not best and score(prior_c) >= score(best) - 0.01:
        best = prior_c

    # 6. Veredicto -----------------------------------------------------------
    median_z = float(np.median(np.concatenate([pts1[:, 2], pts2[:, 2]])))
    expected_sigma_mm = float(sensor.sigma_z(median_z)) * 1000
    cons = best["consistency"]
    quality = "buena" if cons >= opt.good_consistency else ("aceptable" if cons >= opt.ok_consistency else "insuficiente")
    if feature_strong and best["feature_agreement"] is not None and best["feature_agreement"] < 0.5:
        warnings.append("La geometría y la fotogrametría no coinciden (posible simetría o superficie plana). "
                        "Revisa el solape visualmente.")
        if quality == "buena":
            quality = "aceptable"
    if best["blind"] and quality != "insuficiente":
        rivals = [c for c in candidates if c is not best
                  and c["overlap_points"] >= 0.5 * best["overlap_points"]
                  and c["consistency"] >= best["consistency"] - opt.ambiguity_margin
                  and (pose_delta(c["T"], best["T"])["translation_cm"] > opt.max_prior_shift_cm
                       or pose_delta(c["T"], best["T"])["rotation_deg"] > opt.max_prior_shift_deg)]
        if rivals:
            quality = "insuficiente"
            warnings.append(f"Ambigua: {len(rivals)} pose(s) muy distinta(s) explican la escena casi igual de bien. "
                            "Añade volumen y textura en la zona común (no solo una pared o una persona de cerca).")

    T = best["T"]
    needs_confirmation, quality_if_confirmed = False, None
    delta = pose_delta(prior_T, T) if prior_T is not None else None
    if delta and (delta["translation_cm"] > opt.max_prior_shift_cm or delta["rotation_deg"] > opt.max_prior_shift_deg):
        strong = quality == "buena" and feature_strong and (best["feature_agreement"] or 0) >= 0.6
        if not strong and quality != "insuficiente":
            needs_confirmation, quality_if_confirmed = True, quality
            quality = "insuficiente"
            warnings.append(f"La mejor hipótesis se aleja {delta['translation_cm']} cm / {delta['rotation_deg']}° de la "
                            "calibración vigente sin evidencia fotogramétrica suficiente. Si moviste las cámaras, "
                            "confírmalo (estudio: «Moví las cámaras») o recalibra con el tablero ChArUco.")
    metrics = {
        **base_metrics,
        "consistency_pct": round(cons * 100, 1),
        "overlap_points": best["overlap_points"],
        "overlap_frac_pct": round(best["overlap_frac"] * 100, 1),
        "free_space_violation_pct": round(best["violation_points"] / max(1, best["overlap_points"] + best["violation_points"]) * 100, 1),
        "rmse_mm": round(best["rmse_mm"], 2),
        "expected_sigma_mm": round(expected_sigma_mm, 2),
        "median_depth_m": round(median_z, 2),
        "feature_agreement_pct": None if best["feature_agreement"] is None else round(best["feature_agreement"] * 100, 1),
    }
    result = {
        "success": quality != "insuficiente",
        "quality": quality,
        "method": best["name"] + " + ICP point-to-plane robusto",
        "metrics": metrics,
        "candidates": summary,
        "warnings": warnings,
        "delta_vs_prior": delta,
        "needs_confirmation": needs_confirmation,
        "quality_if_confirmed": quality_if_confirmed,  # calidad geométrica si el montaje se movió de verdad
        "total_time_ms": round((time.time() - t0) * 1000, 1),
        "_debug": {"kp1": kp1, "kp2": kp2, "matches": matches, "inliers": inliers},
    }
    if result["success"]:
        result["matrix_4x4"] = np.asarray(T).tolist()
        result["components"] = matrix_to_euler_translation(T)
        result["message"] = (f"Alineación {quality}: consistencia {metrics['consistency_pct']} % sobre "
                             f"{metrics['overlap_points']} puntos comunes, RMSE {metrics['rmse_mm']} mm "
                             f"(ruido esperado del sensor ~{metrics['expected_sigma_mm']} mm a {median_z:.1f} m).")
    else:
        result["candidate_matrix_4x4"] = np.asarray(T).tolist()  # solo para inspección visual
        result["candidate_components"] = matrix_to_euler_translation(T)
        reason = ("contradice la calibración vigente" if needs_confirmation
                  else f"consistencia {metrics['consistency_pct']} %")
        result["message"] = f"Alineación no fiable ({reason}): no se aplica y se conserva la calibración vigente."
    return result



# --------------------------------------------------------------------------- #
# Previsualización de la toma: qué calidad cabe esperar ANTES de capturar
# --------------------------------------------------------------------------- #
# Etiquetas por píxel (rejilla de profundidad diezmada) para pintar guías sobre la vista en vivo
PF_NONE, PF_VALID, PF_COMMON, PF_NEAR, PF_CONFLICT, PF_FAR = 0, 1, 2, 3, 4, 5


def _decimate(view, target_width=212):
    """Profundidad (m) diezmada y sus intrínsecas: ~25 k píxeles bastan para una vista previa."""
    di = view.depth_intr
    step = max(1, int(round(float(di["width"]) / target_width)))
    dm = view.depth_m()[::step, ::step]
    intr = {"width": dm.shape[1], "height": dm.shape[0], "fx": di["fx"] / step, "fy": di["fy"] / step,
            "ppx": (di["ppx"] + 0.5) / step - 0.5, "ppy": (di["ppy"] + 0.5) / step - 0.5}
    return dm, intr


def _pixel_rays(intr):
    vv, uu = np.mgrid[0:intr["height"], 0:intr["width"]]
    return np.stack([(uu - intr["ppx"]) / intr["fx"], (vv - intr["ppy"]) / intr["fy"], np.ones(uu.shape)], -1)


def _zbuffer(P, intr):
    """Profundidad más cercana de unos puntos (marco de la cámara destino) en su rejilla; 0 = nada."""
    h, w = intr["height"], intr["width"]
    out = np.full((h, w), np.inf, np.float32)
    z = P[:, 2]
    ok = z > 1e-3
    P, z = P[ok], z[ok]
    u = np.round(P[:, 0] / z * intr["fx"] + intr["ppx"]).astype(np.int64)
    v = np.round(P[:, 1] / z * intr["fy"] + intr["ppy"]).astype(np.int64)
    inside = (u >= 0) & (u < w) & (v >= 0) & (v < h)
    u, v, z = u[inside], v[inside], z[inside]
    order = np.argsort(-z)
    out[v[order], u[order]] = z[order]
    holes = ~np.isfinite(out)  # cerrar solo los huecos del muestreo: rellenar también donde hay dato
    out[holes] = cv2.erode(out, np.ones((3, 3), np.uint8))[holes]  # desbordaría el primer plano en los bordes
    out[~np.isfinite(out)] = 0
    return out


def _cross_check(dm_a, intr_a, dm_b, intr_b, T_b_to_a, sensor, min_thr, slack=0.0):
    """Marca en la rejilla de A qué píxeles confirma B (común) y cuáles contradice (conflicto).
    slack (m): tolerancia extra, p. ej. el error de calibración que el ICP corrige solo."""
    valid_b = (dm_b >= sensor.z_min_m) & (dm_b <= sensor.z_max_m)
    Pb = (_pixel_rays(intr_b) * dm_b[..., None])[valid_b]
    pred = _zbuffer(transform(T_b_to_a, Pb), intr_a)
    valid_a = (dm_a >= sensor.z_min_m) & (dm_a <= sensor.z_max_m)
    seen = valid_a & (pred > 0)
    za = dm_a.astype(np.float64)
    thr = np.maximum(min_thr, 3.0 * np.sqrt(2.0) * sensor.sigma_z(za) + za / intr_a["fx"])
    thr = np.maximum(thr, slack)
    common = seen & (np.abs(pred - za) <= thr)
    conflict = seen & (pred < za - thr)   # B ve una superficie delante de lo que A mide: imposible
    return common, conflict


def _shared_frustum(intr_a, T_a_to_b, intr_b_full, sensor, n=10):
    """Por geometría pura: qué píxeles de A miran a una zona (entre z_min y z_max) que B también ve.
    Devuelve la máscara y la profundidad mínima y máxima (en A) de esa zona por píxel."""
    rays = _pixel_rays(intr_a).reshape(-1, 3)
    depths = np.linspace(sensor.z_min_m, sensor.z_max_m, n)
    T = np.asarray(T_a_to_b, np.float64)
    hit = np.zeros((rays.shape[0], n), bool)
    for k, d in enumerate(depths):
        Pb = rays * d @ T[:3, :3].T + T[:3, 3]
        z = Pb[:, 2]
        with np.errstate(divide="ignore", invalid="ignore"):
            u = Pb[:, 0] / z * intr_b_full["fx"] + intr_b_full["ppx"]
            v = Pb[:, 1] / z * intr_b_full["fy"] + intr_b_full["ppy"]
        hit[:, k] = ((z >= sensor.z_min_m) & (z <= sensor.z_max_m) & (u >= 0) & (u < intr_b_full["width"])
                     & (v >= 0) & (v < intr_b_full["height"]))
    shape = (intr_a["height"], intr_a["width"])
    mask = hit.any(1)
    dmin = np.where(mask, depths[np.argmax(hit, 1)], 0.0)
    dmax = np.where(mask, depths[n - 1 - np.argmax(hit[:, ::-1], 1)], 0.0)
    return mask.reshape(shape), dmin.reshape(shape), dmax.reshape(shape)


def preflight(view1, view2, T=None, sensor=D455, opt=AlignOptions(), prev=None, icp_slack_m=0.05):
    """Análisis rápido de un par en vivo para no capturar a ciegas.

    T: pose T(2->1) con la que juzgar la zona común (calibración vigente o la de los sliders).
    prev: el campo "_depth" de la llamada anterior, para saber si la escena se mueve.
    icp_slack_m: error de pose que la alineación corrige sola (ICP grueso); la superficie común se
    mide con esa tolerancia y la precisión de la pose actual, con la del ruido del sensor.
    Devuelve métricas, veredicto (level: lista | mejorable | bloqueada), calidad esperada,
    avisos accionables y, en claves "_", las rejillas de etiquetas PF_* para pintar guías.
    """
    t0 = time.time()
    views = (view1, view2)
    dec = [_decimate(v) for v in views]
    issues = []

    def issue(sev, text):
        issues.append({"severity": sev, "text": text})

    # 1. Sincronía --------------------------------------------------------------
    dt_ms = None
    if view1.timestamp is not None and view2.timestamp is not None:
        dt_ms = round(abs(float(view1.timestamp) - float(view2.timestamp)) * 1000, 1)
        if dt_ms > opt.max_dt_s * 1000:
            issue("block", f"Cámaras desincronizadas (Δt {dt_ms} ms): una está congelada o perdió señal.")
        elif dt_ms > opt.warn_dt_s * 1000:
            issue("warn", f"Δt entre cámaras de {dt_ms} ms: no te muevas durante la toma.")

    # 2. Profundidad útil por cámara ---------------------------------------------
    cams, labels = [], []
    for i, (dm, intr) in enumerate(dec):
        name = f"Cam {i + 1}"
        valid = (dm >= sensor.z_min_m) & (dm <= sensor.z_max_m)
        near = (dm > 0) & (dm < sensor.z_min_m)
        far = dm > sensor.z_max_m
        lab = np.full(dm.shape, PF_NONE, np.uint8)
        lab[valid], lab[near], lab[far] = PF_VALID, PF_NEAR, PF_FAR
        labels.append(lab)
        mz = float(np.median(dm[valid])) if valid.any() else None
        cam = {"valid_pct": round(float(valid.mean()) * 100, 1), "near_pct": round(float(near.mean()) * 100, 1),
               "far_pct": round(float(far.mean()) * 100, 1), "nodepth_pct": round(float((dm == 0).mean()) * 100, 1),
               "median_z_m": None if mz is None else round(mz, 2),
               "sigma_mm": None if mz is None else round(float(sensor.sigma_z(mz)) * 1000, 1), "moving_pct": None}
        if prev is not None and i < len(prev) and prev[i] is not None and prev[i].shape == dm.shape:
            both = valid & (prev[i] >= sensor.z_min_m) & (prev[i] <= sensor.z_max_m)
            if both.any():
                thr = np.maximum(0.01, 3.0 * np.sqrt(2.0) * sensor.sigma_z(dm[both]))
                cam["moving_pct"] = round(float((np.abs(dm[both] - prev[i][both]) > thr).mean()) * 100, 1)
        cams.append(cam)
        if valid.mean() < opt.min_valid_depth_frac:
            issue("block", f"{name}: casi sin profundidad útil ({cam['valid_pct']} %). Tapada, demasiado cerca "
                           f"(< {sensor.z_min_m:.2f} m) o mirando al vacío.")
        if near.mean() > 0.10:
            issue("warn", f"{name}: algo está a menos de {sensor.z_min_m:.2f} m (zona roja) y ahí no mide. Aléjalo.")
        if mz is not None and mz > 2.5:
            issue("warn", f"{name}: escena lejana ({mz:.1f} m); el ruido crece con z² (~{cam['sigma_mm']} mm). "
                          "Acerca el sujeto a 0,8–2 m.")
        if cam["moving_pct"] is not None and cam["moving_pct"] > 8:
            issue("warn", f"{name}: la escena se mueve ({cam['moving_pct']} % de la imagen cambia). "
                          "Quédate quieto durante la toma.")

    # 3. Zona común con la pose dada ---------------------------------------------
    overlap, frustum = None, [None, None]
    texture = None
    min_area = opt.min_overlap_points_blind * opt.cloud_voxel_m ** 2 / 2.0  # lo que exige el motor (~0,075 m²)
    rec_area = 2.0 * min_area
    if T is not None:
        T = np.asarray(T, np.float64)
        Ti = np.linalg.inv(T)
        (dm1, i1), (dm2, i2) = dec
        s1, _ = _cross_check(dm1, i1, dm2, i2, T, sensor, opt.min_threshold_m)          # precisión de la pose
        s2, _ = _cross_check(dm2, i2, dm1, i1, Ti, sensor, opt.min_threshold_m)
        c1, k1 = _cross_check(dm1, i1, dm2, i2, T, sensor, opt.min_threshold_m, icp_slack_m)  # superficie común
        c2, k2 = _cross_check(dm2, i2, dm1, i1, Ti, sensor, opt.min_threshold_m, icp_slack_m)
        labels[0][k1], labels[1][k2] = PF_CONFLICT, PF_CONFLICT
        labels[0][c1], labels[1][c2] = PF_COMMON, PF_COMMON
        f1, a1, b1 = _shared_frustum(i1, Ti, view2.depth_intr, sensor)
        f2, _, _ = _shared_frustum(i2, T, view1.depth_intr, sensor)
        frustum = [f1, f2]
        area = [float(np.sum((dm[c] / it["fx"]) * (dm[c] / it["fy"]))) for dm, it, c in ((dm1, i1, c1), (dm2, i2, c2))]
        n_c, n_k = int(c1.sum() + c2.sum()), int(k1.sum() + k2.sum())
        cons = n_c / max(1, n_c + n_k)                                  # ¿la pose está en la cuenca del ICP?
        precise = int(s1.sum() + s2.sum()) / max(1, n_c)                # ¿y además ya es precisa?
        overlap = {"area_m2": round(sum(area) / 2.0, 3), "min_area_m2": round(min_area, 3),
                   "recommended_area_m2": round(rec_area, 3), "consistency_pct": round(cons * 100, 1),
                   "pose_precision_pct": round(precise * 100, 1),
                   "common_pct": [round(float(c1.mean()) * 100, 1), round(float(c2.mean()) * 100, 1)],
                   "conflict_pct": [round(float(k1.mean()) * 100, 1), round(float(k2.mean()) * 100, 1)],
                   "frustum_pct": [round(float(f1.mean()) * 100, 1), round(float(f2.mean()) * 100, 1)],
                   "depth_range_m": None, "recommended_depth_m": None}
        if f1.any():
            mid = (a1[f1] + b1[f1]) / 2.0   # centro de la zona compartida a lo largo de cada rayo de Cam 1
            overlap["depth_range_m"] = [round(float(np.percentile(mid, 10)), 2), round(float(np.percentile(mid, 90)), 2)]
            overlap["recommended_depth_m"] = round(float(np.median(mid)), 2)
        # textura en la zona común (puntos de interés en Cam 1): lo que tendrá la fotogrametría
        zone = c1 if c1.sum() >= 50 else f1
        if zone.any():
            g = _gray_clahe(view1.color)
            mask = cv2.resize(zone.astype(np.uint8) * 255, (g.shape[1], g.shape[0]), interpolation=cv2.INTER_NEAREST)
            pts = cv2.goodFeaturesToTrack(g, maxCorners=800, qualityLevel=0.02, minDistance=6, mask=mask)
            texture = 0 if pts is None else int(len(pts))

        if not f1.any():
            issue("block", "Con esta pose las cámaras no comparten campo de visión entre "
                           f"{sensor.z_min_m:.2f} y {sensor.z_max_m:.1f} m.")
        elif overlap["area_m2"] < min_area:
            if n_k > n_c and (n_c + n_k) > 0:
                issue("warn", f"La pose usada no encaja con lo que ven las cámaras (consistencia "
                              f"{overlap['consistency_pct']} %). Si moviste las cámaras, captura igualmente y alinea; "
                              "luego confirma la pose nueva.")
            else:
                rng_txt = (f" a {overlap['depth_range_m'][0]}–{overlap['depth_range_m'][1]} m de la Cam 1"
                           if overlap["depth_range_m"] else "")
                issue("warn", f"Poca zona común ({overlap['area_m2']} m² de {min_area:.3f} necesarios). Pon objetos "
                              f"con volumen dentro del contorno cian{rng_txt}, idealmente a "
                              f"~{overlap['recommended_depth_m']} m.")
        elif cons < opt.ok_consistency:
            issue("warn", f"Hay zona común pero la pose usada no encaja (consistencia {overlap['consistency_pct']} %): "
                          "¿se movió alguna cámara? Captura y alinea para obtener la pose nueva.")
        elif precise < opt.good_consistency:
            issue("info", f"La pose usada está algo desviada (solo el {overlap['pose_precision_pct']} % de la zona común "
                          "coincide al milímetro); la alineación la refinará.")
        if texture is not None and texture < 40:
            issue("info", f"Poca textura en la zona común ({texture} puntos de interés): la alineación dependerá "
                          "solo de la geometría. Ayudan objetos con dibujos o letras.")
    else:
        issue("info", "Sin calibración: no se puede estimar la zona común. Asegúrate de que ambas cámaras ven el "
                      "mismo objeto con volumen a 0,8–2 m; tras la primera alineación aparecerán las guías.")

    # 4. Veredicto ------------------------------------------------------------------
    blocked = any(i["severity"] == "block" for i in issues)
    warned = any(i["severity"] == "warn" for i in issues)
    if blocked:
        expected = "insuficiente"
    elif overlap is None:
        expected = "desconocida"
    elif overlap["area_m2"] < min_area:
        expected = "desconocida" if overlap["consistency_pct"] < opt.ok_consistency * 100 else "insuficiente"
    elif overlap["consistency_pct"] >= opt.good_consistency * 100:
        expected = "buena"
    elif overlap["consistency_pct"] >= opt.ok_consistency * 100:
        expected = "aceptable"
    else:
        expected = "desconocida"
    level = "bloqueada" if blocked else ("lista" if expected == "buena" and not warned else "mejorable")
    zs = [c["median_z_m"] for c in cams if c["median_z_m"] is not None]
    noise = round(float(sensor.sigma_z(float(np.mean(zs)))) * 1000, 1) if zs else None
    if level == "lista":
        headline = (f"Lista para capturar: calidad esperada buena (consistencia {overlap['consistency_pct']} %, "
                    f"zona común {overlap['area_m2']} m²).")
    elif blocked:
        headline = next(i["text"] for i in issues if i["severity"] == "block")
    else:
        first = next((i["text"] for i in issues if i["severity"] == "warn"), None)
        headline = first or f"Calidad esperada: {expected}."
    if not issues:
        issue("ok", "Todo en orden.")
    return {"level": level, "expected_quality": expected, "headline": headline, "issues": issues,
            "cams": cams, "dt_ms": dt_ms, "overlap": overlap, "texture_points": texture,
            "expected_noise_mm": noise, "time_ms": round((time.time() - t0) * 1000, 1),
            "_labels": labels, "_frustum": frustum, "_depth": [d[0] for d in dec]}


def public_result(res):
    """Resultado serializable a JSON (sin objetos de OpenCV)."""
    return {k: v for k, v in res.items() if not k.startswith("_")}


def draw_matches(view1, view2, res, max_lines=60):
    """Imagen de diagnóstico: inliers 3D (verde) y descartados (rojo), con métricas."""
    img1, img2 = view1.color, view2.color
    h = max(img1.shape[0], img2.shape[0])
    w1 = img1.shape[1]
    vis = np.zeros((h + 64, w1 + img2.shape[1], 3), np.uint8)
    vis[:img1.shape[0], :w1] = img1
    vis[:img2.shape[0], w1:] = img2
    dbg = res.get("_debug") or {}
    kp1, kp2, matches = dbg.get("kp1") or [], dbg.get("kp2") or [], dbg.get("matches") or []
    inl = dbg.get("inliers")
    pairs = list(zip(matches, inl if inl is not None and len(inl) == len(matches) else [False] * len(matches)))
    pairs.sort(key=lambda p: not p[1])
    for m, ok in pairs[:max_lines]:
        p1 = tuple(int(round(c)) for c in kp1[m.queryIdx].pt)
        p2 = (int(round(kp2[m.trainIdx].pt[0])) + w1, int(round(kp2[m.trainIdx].pt[1])))
        col = (80, 220, 80) if ok else (60, 60, 230)
        cv2.line(vis, p1, p2, col, 1, cv2.LINE_AA)
        cv2.circle(vis, p1, 3, col, -1)
        cv2.circle(vis, p2, 3, col, -1)
    met = res.get("metrics") or {}
    cv2.rectangle(vis, (0, h), (vis.shape[1], h + 64), (24, 20, 18), -1)
    line1 = (f"{res.get('quality', '?').upper()} | consistencia {met.get('consistency_pct', '-')}% | "
             f"RMSE {met.get('rmse_mm', '-')} mm (ruido esperado {met.get('expected_sigma_mm', '-')} mm)")
    line2 = (f"SIFT: {met.get('matches', 0)} pares, {met.get('lifted', 0)} con profundidad, "
             f"{met.get('inliers', 0)} inliers 3D | dt {met.get('dt_ms', '-')} ms | {res.get('total_time_ms', 0):.0f} ms")
    cv2.putText(vis, line1, (12, h + 24), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 230, 180), 1, cv2.LINE_AA)
    cv2.putText(vis, line2, (12, h + 50), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (220, 220, 220), 1, cv2.LINE_AA)
    cv2.putText(vis, "CAM 1", (12, 26), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2, cv2.LINE_AA)
    cv2.putText(vis, "CAM 2", (w1 + 12, 26), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 200, 255), 2, cv2.LINE_AA)
    return vis
