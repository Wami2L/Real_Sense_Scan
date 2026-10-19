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
    min_manual_pairs: int = 3            # parejas fijadas a mano para calcular una pose (mejor 5-6)


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
# Correspondencias inequívocas: marcadores ArUco y puntos fijados a mano
# --------------------------------------------------------------------------- #
# Diccionario distinto del tablero ChArUco (5X5_100) para no confundirlos. Celdas grandes:
# se detectan bien a distancia y con mucho ángulo (hasta ~60-70°).
ARUCO_DICT = cv2.aruco.DICT_4X4_50 if hasattr(cv2, "aruco") else None


def detect_markers(bgr):
    """Marcadores ArUco (DICT_4X4_50) de una imagen BGR: {id: esquinas (4, 2) en píxeles}."""
    if ARUCO_DICT is None:
        return {}
    params = cv2.aruco.DetectorParameters()
    params.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX
    det = cv2.aruco.ArucoDetector(cv2.aruco.getPredefinedDictionary(ARUCO_DICT), params)
    corners, ids, _ = det.detectMarkers(cv2.cvtColor(bgr, cv2.COLOR_BGR2GRAY))
    if ids is None:
        return {}
    return {int(i): c.reshape(4, 2).astype(np.float64) for i, c in zip(ids.ravel(), corners)}


def _marker_corners_3d(quad, zmap, intr, sensor):
    """Esquinas 3D (marco de color) de un marcador impreso.

    La profundidad justo en el borde de un marcador es poco fiable, así que se ajusta un plano a
    la profundidad de su interior y del margen blanco que lo rodea (el mismo papel) y se cortan
    los rayos de las 4 esquinas con ese plano.
    """
    c2d = quad.mean(0)
    grown = c2d + (quad - c2d) * 1.3
    mask = np.zeros(zmap.shape, np.uint8)
    cv2.fillConvexPoly(mask, np.round(grown).astype(np.int32), 1)
    vv, uu = np.nonzero((mask > 0) & (zmap > 0))
    if len(vv) < 25:
        return None
    z = zmap[vv, uu].astype(np.float64)
    X = np.column_stack(((uu - intr["ppx"]) / intr["fx"] * z, (vv - intr["ppy"]) / intr["fy"] * z, z))
    for _ in range(3):  # plano robusto: fuera lo que no es el papel
        c = X.mean(0)
        n = np.linalg.svd(X - c, full_matrices=False)[2][2]
        r = (X - c) @ n
        keep = np.abs(r) <= max(3.0 * float(np.std(r)), 0.002)
        if keep.all() or keep.sum() < 25:
            break
        X = X[keep]
    c = X.mean(0)
    n = np.linalg.svd(X - c, full_matrices=False)[2][2]
    rays = np.column_stack(((quad[:, 0] - intr["ppx"]) / intr["fx"], (quad[:, 1] - intr["ppy"]) / intr["fy"],
                            np.ones(4)))
    den = rays @ n
    if np.any(np.abs(den) < 0.05):
        return None  # marcador visto casi de canto
    P = rays * ((c @ n) / den)[:, None]
    if np.any(P[:, 2] < sensor.z_min_m * 0.8) or np.any(P[:, 2] > sensor.z_max_m * 1.2):
        return None
    return P


def marker_correspondences(view1, view2, sensor=D455, z1=None, z2=None):
    """Esquinas de los marcadores que ven ambas cámaras, elevadas a 3D (marco de PROFUNDIDAD)."""
    m1, m2 = detect_markers(view1.color), detect_markers(view2.color)
    common = sorted(set(m1) & set(m2))
    info = {"cam1": sorted(m1), "cam2": sorted(m2), "common": common, "used": [], "no_depth": []}
    P1, P2, px1, px2, labels, owner = [], [], [], [], [], []
    if common:
        z1 = depth_in_color_frame(view1, sensor.z_min_m, sensor.z_max_m) if z1 is None else z1
        z2 = depth_in_color_frame(view2, sensor.z_min_m, sensor.z_max_m) if z2 is None else z2
    for mid in common:
        a = _marker_corners_3d(m1[mid], z1, view1.cintr, sensor)
        b = _marker_corners_3d(m2[mid], z2, view2.cintr, sensor)
        if a is None or b is None:
            info["no_depth"].append(mid)
            continue
        P1.append(transform(np.linalg.inv(view1.E), a))
        P2.append(transform(np.linalg.inv(view2.E), b))
        px1.append(m1[mid])
        px2.append(m2[mid])
        labels += [f"M{mid}.{k + 1}" for k in range(4)]
        owner += [mid] * 4
        info["used"].append(mid)
    stack = (lambda L, k: np.vstack(L) if L else np.empty((0, k)))
    return {"P1": stack(P1, 3), "P2": stack(P2, 3), "px1": stack(px1, 2), "px2": stack(px2, 2),
            "labels": labels, "owner": np.asarray(owner, np.int64), "info": info, "quads": (m1, m2)}


def pose_from_markers(mk, sensor=D455):
    """T(2->1) desde las esquinas de los marcadores: Kabsch por marcador y consenso entre ellos
    (un marcador mal medido no arrastra a los demás). Devuelve (T, inliers, umbral) o (None, ...)."""
    P1, P2, owner = mk["P1"], mk["P2"], mk["owner"]
    if len(P1) < 4:
        return None, np.zeros(len(P1), bool), np.empty(0)
    thr = np.maximum(0.015, 3.0 * np.sqrt(sensor.sigma_z(P1[:, 2]) ** 2 + sensor.sigma_z(P2[:, 2]) ** 2))
    best_T, best_inl = None, None
    for mid in np.unique(owner):
        sel = owner == mid
        T = kabsch(P2[sel], P1[sel])
        inl = np.linalg.norm(transform(T, P2) - P1, axis=1) < thr
        if best_inl is None or inl.sum() > best_inl.sum():
            best_T, best_inl = T, inl
    T = kabsch(P2[best_inl], P1[best_inl]) if best_inl.sum() >= 4 else best_T
    inl = np.linalg.norm(transform(T, P2) - P1, axis=1) < thr
    return T, inl, thr


def _lift_pixel(uv, zmap, intr, radius):
    """Punto 3D (marco de color) bajo un píxel elegido a mano. Devuelve (xyz | None, en_borde)."""
    h, w = zmap.shape
    u, v = int(round(uv[0])), int(round(uv[1]))
    if not (0 <= u < w and 0 <= v < h):
        return None, False
    patch = zmap[max(0, v - radius):min(h, v + radius + 1), max(0, u - radius):min(w, u + radius + 1)]
    vals = patch[patch > 0]
    if vals.size < 3:
        return None, False
    z = float(np.median(vals))
    edge = float(vals.max() - vals.min()) > max(0.02, 3.0 * float(D455.sigma_z(z)))
    if edge:  # entre primer plano y fondo: quedarse con la superficie más cercana (la que se ve)
        near = vals[vals <= np.percentile(vals, 40)]
        z = float(np.median(near))
    return np.array([(uv[0] - intr["ppx"]) / intr["fx"] * z, (uv[1] - intr["ppy"]) / intr["fy"] * z, z]), edge


def robust_kabsch(P2, P1, thr, max_trials=400, seed=0):
    """T(2->1) desde pocas parejas, tolerando alguna errónea (p. ej. un clic en un punto que la
    otra cámara no ve): prueba tríos, se queda con la pose que más parejas explica y la reajusta.
    Devuelve (T, inliers)."""
    import itertools
    n = len(P1)
    combos = list(itertools.combinations(range(n), 3))
    if len(combos) > max_trials:
        rng = np.random.default_rng(seed)
        combos = [combos[i] for i in rng.choice(len(combos), max_trials, replace=False)]
    best, best_key = None, None
    for c in combos:
        a = P1[list(c)]
        if np.linalg.norm(np.cross(a[1] - a[0], a[2] - a[0])) < 1e-4:  # casi alineados
            continue
        T = kabsch(P2[list(c)], a)
        r = np.linalg.norm(transform(T, P2) - P1, axis=1)
        inl = r < thr
        key = (int(inl.sum()), -float(r[inl].sum()))
        if best_key is None or key > best_key:
            best, best_key = inl, key
    if best is None or best.sum() < 3:
        T = kabsch(P2, P1)
        return T, np.linalg.norm(transform(T, P2) - P1, axis=1) < thr
    T = kabsch(P2[best], P1[best])
    return T, np.linalg.norm(transform(T, P2) - P1, axis=1) < thr


def lift_manual_pairs(view1, view2, pairs, sensor=D455, z1=None, z2=None):
    """Parejas elegidas a mano [((u1, v1), (u2, v2)), ...] en píxeles de la imagen de COLOR de cada
    cámara -> puntos 3D (marco de PROFUNDIDAD). Devuelve P1, P2 (solo las válidas) y el estado de cada una."""
    z1 = depth_in_color_frame(view1, sensor.z_min_m, sensor.z_max_m) if z1 is None else z1
    z2 = depth_in_color_frame(view2, sensor.z_min_m, sensor.z_max_m) if z2 is None else z2
    r1 = max(2, int(math.ceil(view1.cintr["width"] / view1.depth_intr["width"])) + 1)
    r2 = max(2, int(math.ceil(view2.cintr["width"] / view2.depth_intr["width"])) + 1)
    P1, P2, px1, px2, status = [], [], [], [], []
    for i, (a, b) in enumerate(pairs):
        pa, ea = _lift_pixel(a, z1, view1.cintr, r1)
        pb, eb = _lift_pixel(b, z2, view2.cintr, r2)
        st = {"i": i + 1, "ok": pa is not None and pb is not None, "edge": bool(ea or eb), "reason": None}
        if pa is None or pb is None:
            st["reason"] = "sin profundidad en " + " y ".join(n for n, p in (("Cam 1", pa), ("Cam 2", pb)) if p is None)
        else:
            P1.append(transform(np.linalg.inv(view1.E), pa[None])[0])
            P2.append(transform(np.linalg.inv(view2.E), pb[None])[0])
            px1.append(a)
            px2.append(b)
            if st["edge"]:
                st["reason"] = "en un borde de profundidad (menos preciso)"
        status.append(st)
    arr = (lambda L, k: np.asarray(L, np.float64).reshape(-1, k))
    return arr(P1, 3), arr(P2, 3), arr(px1, 2), arr(px2, 2), status


# --------------------------------------------------------------------------- #
# Motor
# --------------------------------------------------------------------------- #
def _fail(message, warnings, t0, **extra):
    return {"success": False, "quality": "insuficiente", "message": message, "warnings": warnings,
            "total_time_ms": round((time.time() - t0) * 1000, 1), **extra}


def _depth_coverage(view, sensor):
    dm = view.depth_m()
    return float(((dm >= sensor.z_min_m) & (dm <= sensor.z_max_m)).mean())


def align_pair(view1, view2, prior_T=None, sensor=D455, opt=AlignOptions(), seed=0, extra_seeds=(),
               manual_pairs=()):
    """Calcula T(2->1) en el marco de PROFUNDIDAD de cada cámara.

    prior_T: calibración vigente (semilla y referencia de estabilidad del montaje).
    extra_seeds: [(nombre, T)] hipótesis adicionales, p. ej. la pose de los sliders. No cuentan
    como calibración vigente: solo dan un punto de partida al ICP.
    manual_pairs: [((u1, v1), (u2, v2)), ...] puntos fijados a mano en las imágenes de color.

    Evidencia (de más a menos fiable): puntos a mano, marcadores ArUco (DICT_4X4_50) y SIFT.
    La geometría (consistencia proyectiva) decide entre hipótesis y la evidencia la desempata.

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

    # 3. Correspondencias: SIFT, marcadores ArUco y puntos fijados a mano ----------
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

    zc1 = depth_in_color_frame(view1, sensor.z_min_m, sensor.z_max_m)
    zc2 = depth_in_color_frame(view2, sensor.z_min_m, sensor.z_max_m)
    groups = {}  # evidencia: correspondencias 3D (marco de profundidad) con su umbral
    if n_inl:
        groups["sift"] = {"P1": P1[inliers], "P2": P2[inliers], "thr": thr[inliers], "strong": feature_strong}

    mk = marker_correspondences(view1, view2, sensor, zc1, zc2)
    finfo["markers"] = mk["info"]
    T_mk, mk_inl, mk_thr = pose_from_markers(mk, sensor)
    if T_mk is not None:
        groups["marcadores"] = {"P1": mk["P1"], "P2": mk["P2"], "thr": mk_thr,
                                "strong": int(mk_inl.sum()) >= 4 and mk_inl.mean() >= 0.75}
    elif mk["info"]["common"]:
        warnings.append(f"Marcadores {mk['info']['common']} vistos por ambas cámaras pero sin profundidad útil "
                        "alrededor: acércalos a 0,6–2 m y que no queden de canto.")
    for name, ids in (("Cam 1", set(mk["info"]["cam1"]) - set(mk["info"]["cam2"])),
                      ("Cam 2", set(mk["info"]["cam2"]) - set(mk["info"]["cam1"]))):
        if ids:
            warnings.append(f"Marcador(es) {sorted(ids)} solo los ve la {name}: oriéntalos hacia el punto medio entre cámaras.")

    T_man, man = None, None
    if manual_pairs:
        M1, M2, mpx1, mpx2, mstatus = lift_manual_pairs(view1, view2, manual_pairs, sensor, zc1, zc2)
        man = {"P1": M1, "P2": M2, "px1": mpx1, "px2": mpx2, "status": mstatus}
        finfo["manual_pairs"] = {"given": len(manual_pairs), "valid": len(M1)}
        if len(M1) >= opt.min_manual_pairs:
            lat = np.maximum(M1[:, 2] / view1.cintr["fx"], M2[:, 2] / view2.cintr["fx"]) * 4.0  # ~4 px de clic
            mthr = np.maximum(0.03, 3.0 * np.sqrt(sensor.sigma_z(M1[:, 2]) ** 2 + sensor.sigma_z(M2[:, 2]) ** 2) + lat)
            T_man, man_inl = robust_kabsch(M2, M1, mthr)
            res_m = np.linalg.norm(transform(T_man, M2) - M1, axis=1)
            strong = int(man_inl.sum()) >= 3 and float(man_inl.mean()) >= 0.6
            groups["manual"] = {"P1": M1, "P2": M2, "thr": mthr, "strong": strong}
            valid_ids = [st["i"] for st in mstatus if st["ok"]]
            bad = [valid_ids[k] for k in np.nonzero(~man_inl)[0]]
            if not strong:
                worst = int(np.argmax(res_m))
                warnings.append(f"Los puntos a mano no son coherentes entre sí (la pareja {valid_ids[worst]} se desvía "
                                f"{res_m[worst] * 100:.1f} cm). Revisa que cada pareja marque el mismo punto físico.")
            elif bad:
                warnings.append(f"Pareja(s) a mano {bad} descartada(s): no coinciden con las demás (¿un punto que la "
                                "otra cámara no ve, o un clic desplazado?).")
            if len(M1) == 3:
                warnings.append("Con solo 3 parejas no se puede comprobar si alguna está mal: añade 2–3 más.")
        else:
            warnings.append(f"Puntos a mano: {len(M1)} parejas con profundidad de {len(manual_pairs)}; hacen falta "
                            f"al menos {opt.min_manual_pairs} (mejor 5–6, repartidas y no alineadas).")

    top = next((k for k in ("manual", "marcadores", "sift") if k in groups and groups[k]["strong"]), None)
    if top != "sift" and not feature_strong:
        why = ("texturas repetitivas o poco distintivas" if len(P1) >= opt.min_feature_inliers
               else "poca textura común con profundidad válida")
        msg = f"Fotogrametría débil ({n_inl} de {len(P1)} correspondencias 3D coherentes: {why})."
        if top is None:
            msg += (" La pose depende sobre todo de la geometría; pon marcadores ArUco en la zona común "
                    "o marca puntos a mano.")
        warnings.append(msg)

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
    if T_man is not None:
        seeds.append(("puntos fijados a mano", T_man))
    if T_mk is not None:
        seeds.append(("marcadores ArUco", T_mk))
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

    def agreement(T, kind):
        g = groups.get(kind)
        if g is None or not len(g["P1"]):
            return None
        return float(np.mean(np.linalg.norm(transform(T, g["P2"]) - g["P1"], axis=1) < g["thr"]))

    def add_candidate(name, T0, blind=False, refine=True):
        T = refine_icp(pyr2, pyr1, T0, opt.icp_scales) if refine else np.asarray(T0, np.float64)
        if not plausible_pose(T, sensor):
            return
        ev = evaluate(T, cloud1, cloud2, view1, view2, zd1, zd2, sensor, opt)
        ag = {k: agreement(T, k) for k in groups}
        ev["feature_agreement"] = ag.get(top) if top else ag.get("sift")
        candidates.append({"name": name, "T": T, "blind": blind, "agreements": ag, **ev})

    def geo_viable(c):
        min_pts = opt.min_overlap_points_blind if c["blind"] else opt.min_overlap_points
        return c["overlap_points"] >= min_pts and c["overlap_frac"] >= opt.min_overlap_frac

    def is_viable(c):
        # Con marcadores o puntos a mano coherentes, la pose vale aunque la zona común sea pequeña
        # (cámaras muy abiertas): la evidencia es inequívoca. Se avisa para revisarla a ojo.
        return geo_viable(c) or (top in ("manual", "marcadores") and not c["blind"]
                                 and (c["feature_agreement"] or 0) >= 0.75)

    for name, T0 in seeds:
        add_candidate(name, T0)
        if name in ("puntos fijados a mano", "marcadores ArUco"):
            add_candidate(name + " (sin ICP)", T0, refine=False)
    for name, T0 in blind_seeds:
        add_candidate(name, T0, blind=True)
    if opt.mount_search and top is None and not any(
            is_viable(c) and c["consistency"] >= opt.good_consistency for c in candidates):
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
                "overlap_frac_pct": round(c["overlap_frac"] * 100, 1),
                "rmse_mm": round(c["rmse_mm"], 2) if c["rmse_mm"] == c["rmse_mm"] else None,
                "feature_agreement_pct": None if c["feature_agreement"] is None else round(c["feature_agreement"] * 100, 1),
                "agreements_pct": {k: None if v is None else round(v * 100, 1) for k, v in c["agreements"].items()},
                "blind": c["blind"], "viable": c in viable}
               for c in candidates]
    base_metrics = {"dt_ms": dt_ms, "depth_coverage_pct": {k: round(v * 100, 1) for k, v in cov.items()},
                    "evidence": top, **finfo}
    if not viable:
        out = _fail("Solape insuficiente entre cámaras: ninguna hipótesis deja zona común suficiente. "
                    "Coloca el sujeto u objetos donde lo vean AMBAS cámaras, entre "
                    f"{sensor.z_min_m:.2f} y {sensor.z_max_m:.1f} m, sin taparlas de cerca, "
                    "o usa marcadores ArUco / puntos a mano.",
                    warnings, t0, candidates=summary, metrics=base_metrics)
        out["correspondences"] = _correspondence_list(None, P1, P2, matches, kp1, kp2, inliers, thr, mk, groups, man)
        return out

    # 5. Selección: consistencia geométrica y, si hay evidencia fuerte, que también la respete --
    def score(c):
        s = c["consistency"]
        if top is not None and c["feature_agreement"] is not None:
            w = 0.3 if top == "sift" else 0.5
            s = (1 - w) * s + w * c["feature_agreement"]
        return s

    best = max(viable, key=score)
    # Estabilidad: si la calibración vigente refinada es igual de buena, no se mueve la pose
    prior_c = next((c for c in viable if c["name"] == "calibración vigente"), None)
    if prior_c is not None and prior_c is not best and score(prior_c) >= score(best) - 0.01:
        best = prior_c
    for s_ in summary:
        s_["chosen"] = s_["name"] == best["name"]

    # 6. Veredicto -----------------------------------------------------------
    median_z = float(np.median(np.concatenate([pts1[:, 2], pts2[:, 2]])))
    expected_sigma_mm = float(sensor.sigma_z(median_z)) * 1000
    cons = best["consistency"]
    quality = "buena" if cons >= opt.good_consistency else ("aceptable" if cons >= opt.ok_consistency else "insuficiente")
    evidence_only = not geo_viable(best)
    if evidence_only:
        quality = "aceptable" if (best["feature_agreement"] or 0) >= 0.9 else "insuficiente"
        warnings.append("Poca superficie común: la pose sale de los marcadores / puntos a mano y la geometría no "
                        "puede confirmarla. Revísala en la fusión 2D y en el mapa de error.")
    if top is not None and best["feature_agreement"] is not None and best["feature_agreement"] < 0.5:
        src = {"sift": "la fotogrametría", "marcadores": "los marcadores", "manual": "los puntos a mano"}[top]
        warnings.append(f"La geometría y {src} no coinciden (posible simetría, superficie plana o un error en "
                        "las correspondencias). Revisa el solape visualmente.")
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
                            "Añade volumen y textura en la zona común, marcadores ArUco o puntos a mano.")

    T = best["T"]
    needs_confirmation, quality_if_confirmed = False, None
    delta = pose_delta(prior_T, T) if prior_T is not None else None
    if delta and (delta["translation_cm"] > opt.max_prior_shift_cm or delta["rotation_deg"] > opt.max_prior_shift_deg):
        ag = best["feature_agreement"] or 0
        strong = ((top == "sift" and quality == "buena" and ag >= 0.6)
                  or (top in ("marcadores", "manual") and quality != "insuficiente" and ag >= 0.75))
        if not strong and quality != "insuficiente":
            needs_confirmation, quality_if_confirmed = True, quality
            quality = "insuficiente"
            warnings.append(f"La mejor hipótesis se aleja {delta['translation_cm']} cm / {delta['rotation_deg']}° de la "
                            "calibración vigente sin evidencia suficiente (marcadores, puntos a mano o fotogrametría). "
                            "Si moviste las cámaras, confírmalo (estudio: «Moví las cámaras»).")
    metrics = {
        **base_metrics,
        "consistency_pct": round(cons * 100, 1),
        "overlap_points": best["overlap_points"],
        "overlap_frac_pct": round(best["overlap_frac"] * 100, 1),
        "free_space_violation_pct": round(best["violation_points"] / max(1, best["overlap_points"] + best["violation_points"]) * 100, 1),
        "rmse_mm": round(best["rmse_mm"], 2) if best["rmse_mm"] == best["rmse_mm"] else None,
        "expected_sigma_mm": round(expected_sigma_mm, 2),
        "median_depth_m": round(median_z, 2),
        "feature_agreement_pct": None if best["feature_agreement"] is None else round(best["feature_agreement"] * 100, 1),
    }
    result = {
        "success": quality != "insuficiente",
        "quality": quality,
        "method": best["name"] + ("" if "sin ICP" in best["name"] else " + ICP point-to-plane robusto"),
        "metrics": metrics,
        "candidates": summary,
        "warnings": warnings,
        "delta_vs_prior": delta,
        "needs_confirmation": needs_confirmation,
        "quality_if_confirmed": quality_if_confirmed,  # calidad geométrica si el montaje se movió de verdad
        "correspondences": _correspondence_list(T, P1, P2, matches, kp1, kp2, inliers, thr, mk, groups, man),
        "total_time_ms": round((time.time() - t0) * 1000, 1),
        "_debug": {"kp1": kp1, "kp2": kp2, "matches": matches, "inliers": inliers, "markers": mk["quads"]},
    }
    if man is not None:
        result["manual_pairs"] = man["status"]
    if result["success"]:
        result["matrix_4x4"] = np.asarray(T).tolist()
        result["components"] = matrix_to_euler_translation(T)
        rm = f", RMSE {metrics['rmse_mm']} mm" if metrics["rmse_mm"] is not None else ""
        result["message"] = (f"Alineación {quality}: consistencia {metrics['consistency_pct']} % sobre "
                             f"{metrics['overlap_points']} puntos comunes{rm} "
                             f"(ruido esperado del sensor ~{metrics['expected_sigma_mm']} mm a {median_z:.1f} m).")
    else:
        result["candidate_matrix_4x4"] = np.asarray(T).tolist()  # solo para inspección visual
        result["candidate_components"] = matrix_to_euler_translation(T)
        reason = ("contradice la calibración vigente" if needs_confirmation
                  else f"consistencia {metrics['consistency_pct']} %")
        result["message"] = f"Alineación no fiable ({reason}): no se aplica y se conserva la calibración vigente."
    return result


def _correspondence_list(T, P1, P2, matches, kp1, kp2, inliers, thr, mk, groups, man, max_sift=120):
    """Parejas para el explorador del estudio: píxeles en ambas imágenes de color y residuo 3D (mm)
    con la pose final. kind: manual | marcador | sift."""
    out = []

    def residual(Q1, Q2, k):
        if T is None or not len(Q1):
            return [None] * len(Q1)
        return list(np.linalg.norm(transform(T, Q2) - Q1, axis=1) * 1000)

    if man is not None:
        r = residual(man["P1"], man["P2"], "manual")
        mthr = groups["manual"]["thr"] if "manual" in groups else np.full(len(man["P1"]), 0.03)
        j = 0
        for st in man["status"]:
            item = {"kind": "manual", "label": f"P{st['i']}", "residual_mm": None, "ok": False,
                    "note": st["reason"], "p1": None, "p2": None}
            if st["ok"]:
                item.update(p1=[round(float(x), 1) for x in man["px1"][j]], p2=[round(float(x), 1) for x in man["px2"][j]])
                if r[j] is not None:
                    item["residual_mm"] = round(float(r[j]), 1)
                    item["ok"] = bool(r[j] / 1000 < mthr[j])
                j += 1
            out.append(item)
    if len(mk["P1"]):
        r = residual(mk["P1"], mk["P2"], "marcadores")
        mthr = groups["marcadores"]["thr"] if "marcadores" in groups else np.full(len(mk["P1"]), 0.015)
        for k in range(len(mk["P1"])):
            out.append({"kind": "marcador", "label": mk["labels"][k],
                        "p1": [round(float(x), 1) for x in mk["px1"][k]], "p2": [round(float(x), 1) for x in mk["px2"][k]],
                        "residual_mm": None if r[k] is None else round(float(r[k]), 1),
                        "ok": r[k] is not None and bool(r[k] / 1000 < mthr[k]), "note": None})
    if len(P1):
        r = residual(P1, P2, "sift")
        order = sorted(range(len(P1)), key=lambda k: (not bool(inliers[k]) if len(inliers) else True, r[k] or 0))
        for n, k in enumerate(order[:max_sift]):
            m = matches[k]
            out.append({"kind": "sift", "label": f"S{n + 1}",
                        "p1": [round(float(x), 1) for x in kp1[m.queryIdx].pt],
                        "p2": [round(float(x), 1) for x in kp2[m.trainIdx].pt],
                        "residual_mm": None if r[k] is None else round(float(r[k]), 1),
                        "ok": r[k] is not None and bool(r[k] / 1000 < thr[k]), "note": None})
    return out



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

    # 2b. Marcadores ArUco: si ambas cámaras ven alguno, la alineación será directa -----
    mq = (detect_markers(view1.color), detect_markers(view2.color))
    common_mk = sorted(set(mq[0]) & set(mq[1]))
    markers = {"cam1": sorted(mq[0]), "cam2": sorted(mq[1]), "common": common_mk}
    if common_mk:
        issue("ok", f"Marcador(es) {common_mk} visibles por ambas cámaras: la alineación será directa.")
    for name, ids in (("Cam 1", set(mq[0]) - set(mq[1])), ("Cam 2", set(mq[1]) - set(mq[0]))):
        if ids:
            issue("info", f"Marcador(es) {sorted(ids)} solo en la {name}: gíralos hacia el punto medio entre cámaras.")

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
    if not blocked and common_mk and expected in ("desconocida", "insuficiente"):
        expected = "aceptable"  # los marcadores dan la pose aunque la calibración vieja no encaje
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
    if level != "lista" and not blocked and common_mk and not warned:
        headline = f"Marcadores {common_mk} vistos por ambas cámaras: captura y alinea."
    return {"level": level, "expected_quality": expected, "headline": headline, "issues": issues,
            "cams": cams, "dt_ms": dt_ms, "overlap": overlap, "texture_points": texture, "markers": markers,
            "_markers": [{k: v.tolist() for k, v in m.items()} for m in mq],
            "_color_size": [list(v.color.shape[1::-1]) for v in views],
            "expected_noise_mm": noise, "time_ms": round((time.time() - t0) * 1000, 1),
            "_labels": labels, "_frustum": frustum, "_depth": [d[0] for d in dec]}



# --------------------------------------------------------------------------- #
# Mapa de error de la alineación y combinación de varias tomas
# --------------------------------------------------------------------------- #
ERROR_BINS_MM = (5.0, 10.0, 20.0, 50.0)
ERROR_COLORS = ((80, 200, 80), (60, 220, 230), (40, 150, 250), (60, 60, 235), (60, 20, 150))  # BGR por tramo
ERROR_LABELS = ("< 5 mm", "5-10", "10-20", "20-50", "> 50 mm")


def alignment_error_map(view1, view2, T, sensor=D455, same_surface_m=0.10):
    """Distancia entre lo que mide la Cam 1 y lo que mide la Cam 2 llevada con T, píxel a píxel.

    Pinta sobre la imagen de la Cam 1: verde < 5 mm ... rojo > 20 mm. Solo cuenta la superficie que
    ven ambas (diferencias > same_surface_m son oclusiones u otra superficie, no error de pose).
    Devuelve (imagen BGR, estadísticas).
    """
    (dm1, i1), (dm2, i2) = _decimate(view1, 424), _decimate(view2, 424)
    valid2 = (dm2 >= sensor.z_min_m) & (dm2 <= sensor.z_max_m)
    pred = _zbuffer(transform(T, (_pixel_rays(i2) * dm2[..., None])[valid2]), i1)
    valid1 = (dm1 >= sensor.z_min_m) & (dm1 <= sensor.z_max_m)
    err = np.abs(pred.astype(np.float64) - dm1.astype(np.float64)) * 1000.0
    same = valid1 & (pred > 0) & (err <= same_surface_m * 1000.0)
    e = err[same]
    stats = {"pixels": int(same.sum()), "area_m2": round(float(np.sum((dm1[same] / i1["fx"]) * (dm1[same] / i1["fy"]))), 3)}
    if e.size:
        zs = float(np.median(dm1[same]))
        stats.update(median_mm=round(float(np.median(e)), 1), p90_mm=round(float(np.percentile(e, 90)), 1),
                     pct_lt_5=round(float(np.mean(e < 5)) * 100, 1), pct_lt_10=round(float(np.mean(e < 10)) * 100, 1),
                     pct_lt_20=round(float(np.mean(e < 20)) * 100, 1), median_depth_m=round(zs, 2),
                     expected_sigma_mm=round(float(sensor.sigma_z(zs)) * 1000 * math.sqrt(2.0), 1))
    # pintar sobre el color de Cam 1 (rejilla de profundidad -> imagen de color)
    base = view1.color.copy()
    h, w = base.shape[:2]
    bins = np.digitize(err, ERROR_BINS_MM)
    lab = np.where(same, bins + 1, 0).astype(np.uint8)
    lab = cv2.resize(lab, (w, h), interpolation=cv2.INTER_NEAREST)
    out = base.astype(np.float32) * 0.45
    for k, col in enumerate(ERROR_COLORS):
        m = lab == k + 1
        out[m] = base[m].astype(np.float32) * 0.35 + np.array(col, np.float32) * 0.65
    img = out.clip(0, 255).astype(np.uint8)
    band = np.full((46, w, 3), 24, np.uint8)
    x = 10
    for col, txt in zip(ERROR_COLORS, ERROR_LABELS):
        cv2.rectangle(band, (x, 8), (x + 14, 22), col, -1)
        cv2.putText(band, txt, (x + 18, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.42, (230, 230, 230), 1, cv2.LINE_AA)
        x += 22 + 9 * len(txt) + 14
    line = ("Sin superficie comun" if not e.size else
            f"Mediana {stats['median_mm']} mm | p90 {stats['p90_mm']} mm | <10 mm: {stats['pct_lt_10']}% | "
            f"comun {stats['area_m2']} m2 | ruido esperado ~{stats['expected_sigma_mm']} mm")
    cv2.putText(band, line, (10, 40), cv2.FONT_HERSHEY_SIMPLEX, 0.42, (0, 230, 180), 1, cv2.LINE_AA)
    return np.vstack([img, band]), stats


def combine_poses(entries, max_dev_cm=3.0, max_dev_deg=3.0):
    """Media ponderada robusta de varias T(2->1) del mismo montaje (una por toma).

    entries: [{"T": 4x4, "weight": w, ...}]. Descarta las que se alejan de la de más peso más de
    max_dev_cm / max_dev_deg. Devuelve (T_media, usadas, desviaciones de cada usada a la media).
    """
    if not entries:
        return None, [], []
    ref = max(entries, key=lambda e: e["weight"])["T"]
    used = [e for e in entries if pose_delta(e["T"], ref)["translation_cm"] <= max_dev_cm
            and pose_delta(e["T"], ref)["rotation_deg"] <= max_dev_deg]
    w = np.array([max(e["weight"], 1e-6) for e in used], np.float64)
    Rs = np.stack([np.asarray(e["T"], np.float64)[:3, :3] for e in used])
    ts = np.stack([np.asarray(e["T"], np.float64)[:3, 3] for e in used])
    U, _, Vt = np.linalg.svd(np.einsum("i,ijk->jk", w, Rs))
    R = U @ np.diag([1.0, 1.0, np.sign(np.linalg.det(U @ Vt))]) @ Vt
    Tm = np.eye(4)
    Tm[:3, :3], Tm[:3, 3] = R, (w[:, None] * ts).sum(0) / w.sum()
    return Tm, used, [pose_delta(e["T"], Tm) for e in used]


def public_result(res):
    """Resultado serializable a JSON (sin objetos de OpenCV)."""
    return {k: v for k, v in res.items() if not k.startswith("_")}


KIND_COLORS = {"sift": ((80, 220, 80), (60, 60, 230)), "marcador": ((0, 215, 255), (0, 120, 255)),
               "manual": ((255, 220, 0), (220, 60, 220))}  # BGR: (coherente, incoherente)


def draw_matches(view1, view2, res, max_lines=80):
    """Imagen de diagnóstico: parejas numeradas por tipo (SIFT, marcador, a mano), coherentes con la
    pose final o no, marcadores detectados y las métricas."""
    img1, img2 = view1.color, view2.color
    h = max(img1.shape[0], img2.shape[0])
    w1 = img1.shape[1]
    vis = np.zeros((h + 64, w1 + img2.shape[1], 3), np.uint8)
    vis[:img1.shape[0], :w1] = img1
    vis[:img2.shape[0], w1:] = img2
    quads = ((res.get("_debug") or {}).get("markers")) or ({}, {})
    for off, q in ((0, quads[0]), (w1, quads[1])):
        for mid, c in q.items():
            pts = np.round(c + [off, 0]).astype(np.int32)
            cv2.polylines(vis, [pts], True, (0, 215, 255), 2, cv2.LINE_AA)
            cv2.putText(vis, f"ID {mid}", tuple(pts[0] + [0, -6]), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 215, 255), 1,
                        cv2.LINE_AA)
    corr = [c for c in res.get("correspondences") or [] if c.get("p1") is not None]
    corr = [c for c in corr if c["kind"] != "sift"] + [c for c in corr if c["kind"] == "sift"][:max_lines]
    for c in corr:
        good, bad = KIND_COLORS[c["kind"]]
        col = good if c["ok"] else bad
        p1 = tuple(int(round(v)) for v in c["p1"])
        p2 = (int(round(c["p2"][0])) + w1, int(round(c["p2"][1])))
        cv2.line(vis, p1, p2, col, 1 if c["kind"] == "sift" else 2, cv2.LINE_AA)
        for p in (p1, p2):
            cv2.circle(vis, p, 3 if c["kind"] == "sift" else 5, col, -1)
        if c["kind"] != "sift":
            for p in (p1, p2):
                cv2.putText(vis, c["label"], (p[0] + 6, p[1] - 6), cv2.FONT_HERSHEY_SIMPLEX, 0.45, col, 1, cv2.LINE_AA)
    met = res.get("metrics") or {}
    cv2.rectangle(vis, (0, h), (vis.shape[1], h + 64), (24, 20, 18), -1)
    line1 = (f"{res.get('quality', '?').upper()} | consistencia {met.get('consistency_pct', '-')}% | "
             f"RMSE {met.get('rmse_mm', '-')} mm (ruido esperado {met.get('expected_sigma_mm', '-')} mm) | "
             f"evidencia: {met.get('evidence') or 'solo geometria'}")
    mk = met.get("markers") or {}
    line2 = (f"SIFT: {met.get('matches', 0)} pares, {met.get('lifted', 0)} con profundidad, "
             f"{met.get('inliers', 0)} coherentes | marcadores comunes {mk.get('used', [])} | "
             f"a mano {(met.get('manual_pairs') or {}).get('valid', 0)} | dt {met.get('dt_ms', '-')} ms")
    cv2.putText(vis, line1, (12, h + 24), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 230, 180), 1, cv2.LINE_AA)
    cv2.putText(vis, line2, (12, h + 50), cv2.FONT_HERSHEY_SIMPLEX, 0.47, (220, 220, 220), 1, cv2.LINE_AA)
    cv2.putText(vis, "CAM 1", (12, 26), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2, cv2.LINE_AA)
    cv2.putText(vis, "CAM 2", (w1 + 12, 26), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 200, 255), 2, cv2.LINE_AA)
    return vis
