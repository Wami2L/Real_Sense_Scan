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
    good_consistency: float = 0.85
    ok_consistency: float = 0.70
    min_overlap_frac: float = 0.03   # fracción mínima de ambas nubes que debe quedar en zona común
    # Un montaje rígido no se mueve solo: alejarse más que esto de la calibración vigente
    # exige evidencia fuerte (calidad buena + fotogrametría coherente) o confirmación manual.
    max_prior_shift_cm: float = 5.0
    max_prior_shift_deg: float = 5.0


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
    thr = np.maximum(min_thr, 3.0 * np.sqrt(sensor.sigma_z(z) ** 2 + sensor.sigma_z(zm) ** 2))
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


# --------------------------------------------------------------------------- #
# Motor
# --------------------------------------------------------------------------- #
def _fail(message, warnings, t0, **extra):
    return {"success": False, "quality": "insuficiente", "message": message, "warnings": warnings,
            "total_time_ms": round((time.time() - t0) * 1000, 1), **extra}


def _depth_coverage(view, sensor):
    dm = view.depth_m()
    return float(((dm >= sensor.z_min_m) & (dm <= sensor.z_max_m)).mean())


def align_pair(view1, view2, prior_T=None, sensor=D455, opt=AlignOptions(), seed=0):
    """Calcula T(2->1) en el marco de PROFUNDIDAD de cada cámara.

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
    try:
        seeds.append(("FPFH global (geometría)", global_fpfh(pcd2, pcd1, 0.02)))
    except RuntimeError as e:
        warnings.append(f"Registro global FPFH falló: {e}")

    candidates = []
    for name, T0 in seeds:
        T = refine_icp(pyr2, pyr1, T0, opt.icp_scales)
        ev = evaluate(T, cloud1, cloud2, view1, view2, zd1, zd2, sensor, opt, P1[inliers] if n_inl else None,
                      P2[inliers] if n_inl else None, thr[inliers] if n_inl else None)
        candidates.append({"name": name, "T": T, **ev})

    viable = [c for c in candidates if c["overlap_points"] >= opt.min_overlap_points
              and c["overlap_frac"] >= opt.min_overlap_frac]
    summary = [{"name": c["name"], "consistency_pct": round(c["consistency"] * 100, 1),
                "overlap_points": c["overlap_points"], "hidden_points": c["hidden_points"],
                "overlap_frac_pct": round(c["overlap_frac"] * 100, 1), "rmse_mm": round(c["rmse_mm"], 2) if c["rmse_mm"] == c["rmse_mm"] else None,
                "feature_agreement_pct": None if c["feature_agreement"] is None else round(c["feature_agreement"] * 100, 1)}
               for c in candidates]
    base_metrics = {"dt_ms": dt_ms, "depth_coverage_pct": {k: round(v * 100, 1) for k, v in cov.items()}, **finfo}
    if not viable:
        return _fail("Solape insuficiente entre cámaras: ninguna hipótesis deja zona común suficiente. "
                     "Coloca el sujeto u objetos donde lo vean AMBAS cámaras, entre "
                     f"{sensor.z_min_m:.2f} y {sensor.z_max_m:.1f} m.",
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

    T = best["T"]
    needs_confirmation = False
    delta = pose_delta(prior_T, T) if prior_T is not None else None
    if delta and (delta["translation_cm"] > opt.max_prior_shift_cm or delta["rotation_deg"] > opt.max_prior_shift_deg):
        strong = quality == "buena" and feature_strong and (best["feature_agreement"] or 0) >= 0.6
        if not strong and quality != "insuficiente":
            needs_confirmation = True
            quality = "insuficiente"
            warnings.append(f"La mejor hipótesis se aleja {delta['translation_cm']} cm / {delta['rotation_deg']}° de la "
                            "calibración vigente sin evidencia suficiente. Si moviste las cámaras, recalibra con el "
                            "tablero ChArUco o confirma la pose a mano en el modo Solape.")
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
