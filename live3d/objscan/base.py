"""
Base de escaneo: hoja impresa con marcadores ArUco alrededor del objeto.

Una sola definición (BaseLayout) para la hoja imprimible, el render sintético y la detección.
Cada vista que ve al menos un marcador da la pose EXACTA de la cámara respecto a la base
(PnP con la geometría conocida de la hoja). La escala se comprueba con la profundidad de la
D455: si la hoja no se imprimió al 100 % se detecta y se corrige.

Marcadores DICT_4X4_50, ids 10-17 (los 0-3 quedan para calibrar las dos cámaras).
"""

from dataclasses import dataclass, field

import cv2
import numpy as np

ARUCO_DICT = cv2.aruco.DICT_4X4_50
BASE_IDS = tuple(range(10, 18))


@dataclass(frozen=True)
class BaseLayout:
    """Geometría de la hoja (mm). Marcadores en las esquinas y en el centro de los lados."""
    name: str = "A3"
    width_mm: float = 420.0      # lado largo (eje X de la base)
    height_mm: float = 297.0     # lado corto (eje Z de la base)
    marker_mm: float = 60.0
    margin_mm: float = 12.0
    ids: tuple = BASE_IDS

    def marker_origins_mm(self):
        """Esquina superior izquierda de cada marcador en coordenadas de hoja (mm, desde el centro,
        u a la derecha, v hacia abajo)."""
        hw, hh, s, m = self.width_mm / 2, self.height_mm / 2, self.marker_mm, self.margin_mm
        left, right = -hw + m, hw - m - s
        top, bottom = -hh + m, hh - m - s
        cx, cz = -s / 2, -s / 2
        spots = [(left, top), (right, top), (right, bottom), (left, bottom),   # esquinas
                 (cx, top), (cx, bottom), (left, cz), (right, cz)]               # centros de lados
        return {mid: spots[k] for k, mid in enumerate(self.ids[:len(spots)])}

    def marker_corners_3d(self):
        """{id: (4, 3)} esquinas en el marco base (metros), en el orden de ArUco: TL, TR, BR, BL."""
        out = {}
        s = self.marker_mm
        for mid, (u, v) in self.marker_origins_mm().items():
            quad = np.array([[u, v], [u + s, v], [u + s, v + s], [u, v + s]], np.float64) / 1000.0
            out[mid] = np.column_stack((quad[:, 0], np.zeros(4), quad[:, 1]))
        return out

    def free_radius_m(self):
        """Radio del círculo libre central (sin marcadores) donde va el objeto."""
        inner_w = self.width_mm / 2 - self.margin_mm - self.marker_mm
        inner_h = self.height_mm / 2 - self.margin_mm - self.marker_mm
        return min(inner_w, inner_h) / 1000.0


LAYOUTS = {
    "A3": BaseLayout(),
    "A4": BaseLayout(name="A4", width_mm=297.0, height_mm=210.0, marker_mm=45.0, margin_mm=10.0,
                     ids=BASE_IDS[:8]),
}


def render_sheet(layout, px_per_mm=4.0, guides=True):
    """Imagen en escala de grises de la hoja (blanco = papel). La usan el PDF y el render sintético."""
    W, H = int(round(layout.width_mm * px_per_mm)), int(round(layout.height_mm * px_per_mm))
    img = np.full((H, W), 255, np.uint8)
    d = cv2.aruco.getPredefinedDictionary(ARUCO_DICT)
    s = int(round(layout.marker_mm * px_per_mm))
    for mid, (u, v) in layout.marker_origins_mm().items():
        x = int(round((u + layout.width_mm / 2) * px_per_mm))
        y = int(round((v + layout.height_mm / 2) * px_per_mm))
        img[y:y + s, x:x + s] = cv2.aruco.generateImageMarker(d, mid, s)
    if guides:
        c = (W // 2, H // 2)
        r = int(layout.free_radius_m() * 1000 * px_per_mm)
        g = 175
        cv2.circle(img, c, r, g, max(1, int(px_per_mm * 0.6)), cv2.LINE_AA)
        cv2.circle(img, c, int(r * 0.5), g, max(1, int(px_per_mm * 0.4)), cv2.LINE_AA)
        L = int(12 * px_per_mm)
        cv2.line(img, (c[0] - L, c[1]), (c[0] + L, c[1]), g, max(1, int(px_per_mm * 0.4)))
        cv2.line(img, (c[0], c[1] - L), (c[0], c[1] + L), g, max(1, int(px_per_mm * 0.4)))
    return img


def detect_all(bgr, layout):
    """Todas las detecciones de marcadores de la base: [(id, esquinas (4, 2) px)].
    Puede haber ids repetidos si se ven varias copias de la hoja (p. ej. pegadas en la pared)."""
    params = cv2.aruco.DetectorParameters()
    params.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX
    det = cv2.aruco.ArucoDetector(cv2.aruco.getPredefinedDictionary(ARUCO_DICT), params)
    corners, ids, _ = det.detectMarkers(cv2.cvtColor(bgr, cv2.COLOR_BGR2GRAY))
    if ids is None:
        return []
    known = set(layout.ids)
    return [(int(i), c.reshape(4, 2).astype(np.float64)) for i, c in zip(ids.ravel(), corners) if int(i) in known]


def detect(bgr, layout):
    """Marcadores de la base visibles: {id: esquinas (4, 2) px} (la hoja principal si hay varias)."""
    sheets = group_sheets(detect_all(bgr, layout), layout)
    return sheets[0] if sheets else {}


def group_sheets(found, layout, K=None, scale=1.0):
    """Separa las detecciones en hojas: cada marcador propone la pose de su hoja (PnP de un
    cuadrado) y se agrupan los marcadores que encajan con ella, un ejemplar por id.
    Devuelve [{id: esquinas}] de mayor a menor; con ids únicos, una sola hoja."""
    if not found:
        return []
    ids = [m for m, _ in found]
    if len(set(ids)) == len(ids):
        return [dict(found)]
    if K is None:   # sin intrínsecas: aproximación con un campo de visión típico
        h = max(q[:, 1].max() for _, q in found) * 2
        w = max(q[:, 0].max() for _, q in found) * 2
        K = np.array([[0.75 * w, 0, w / 2], [0, 0.75 * w, h / 2], [0, 0, 1]], np.float64)
    model = layout.marker_corners_3d()
    groups = []
    for m, q in found:
        n, rvecs, tvecs, _ = cv2.solvePnPGeneric(model[m] * scale, q, K, None, flags=cv2.SOLVEPNP_IPPE)
        for rv, tv in zip(rvecs[:n], tvecs[:n]):
            best = {}
            for j, (mj, qj) in enumerate(found):
                proj, _ = cv2.projectPoints(model[mj] * scale, rv, tv, K, None)
                err = float(np.sqrt(np.mean(np.sum((proj.reshape(4, 2) - qj) ** 2, axis=1))))
                side = float(np.mean(np.linalg.norm(qj - np.roll(qj, 1, axis=0), axis=1)))
                if err < max(4.0, 0.25 * side) and (mj not in best or err < best[mj][0]):
                    best[mj] = (err, j)
            groups.append(frozenset(j for _, j in best.values()))
    # hojas disjuntas, de la más completa a la menos
    sheets, used = [], set()
    for g in sorted(set(groups), key=len, reverse=True):
        g = g - used
        if len({ids[j] for j in g}) < len(g) or not g:
            continue
        used |= g
        sheets.append({ids[j]: found[j][1] for j in sorted(g)})
    return sheets


def K_from(intr):
    return np.array([[intr["fx"], 0, intr["ppx"]], [0, intr["fy"], intr["ppy"]], [0, 0, 1]], np.float64)


@dataclass
class BasePose:
    ok: bool
    T_cam_to_base: np.ndarray = None    # 4x4: X_base = T · X_cam (marco de COLOR de la cámara)
    markers: list = field(default_factory=list)
    reproj_px: float = None
    scale_ratio: float = None           # profundidad medida / profundidad PnP (1.0 = hoja al 100 %)
    message: str = ""
    sheets: int = 1                     # copias de la hoja a la vista

    def public(self):
        return {"ok": self.ok, "markers": self.markers, "hojas": self.sheets,
                "reproj_px": None if self.reproj_px is None else round(self.reproj_px, 2),
                "scale_ratio": None if self.scale_ratio is None else round(self.scale_ratio, 4),
                "message": self.message}


def estimate_pose(bgr, intr, layout, zmap=None, scale=1.0, quads=None):
    """Pose de la cámara respecto a la base por PnP sobre todas las esquinas visibles.

    zmap: profundidad (m) alineada al color, opcional. Con ella se mide la relación de escala
    real/impresa (para detectar hojas impresas al 90 %, etc.). `scale` aplica una escala ya
    conocida a la geometría de la hoja.
    Si se ven varias copias de la hoja (p. ej. otra pegada en la pared) se separan y se usa la
    más horizontal (la de la mesa) entre las que dan una pose fiable.
    """
    if quads is not None:
        return _pose_from_quads(quads, intr, layout, zmap, scale)
    sheets = group_sheets(detect_all(bgr, layout), layout, K_from(intr), scale)
    if not sheets:
        return BasePose(False, message="No se ve ningún marcador de la base")
    poses = [_pose_from_quads(q, intr, layout, zmap, scale) for q in sheets]
    for bp in poses:
        bp.sheets = len(sheets)

    def rank(bp):
        if not bp.ok:
            return (0, 0.0)
        up = -float(np.linalg.inv(bp.T_cam_to_base)[1, 1])   # normal de la hoja vs. "arriba" de la imagen
        return (1, up + 0.03 * len(bp.markers))
    best = max(poses, key=rank)
    if len(sheets) > 1:
        best.message += " · se ven otras copias de la hoja: se usa la de la mesa"
    return best


def _pose_from_quads(quads, intr, layout, zmap, scale):
    if not quads:
        return BasePose(False, message="No se ve ningún marcador de la base")
    model = layout.marker_corners_3d()
    obj = np.vstack([model[m] for m in sorted(quads)]) * scale
    img = np.vstack([quads[m] for m in sorted(quads)])
    K = K_from(intr)
    flags = cv2.SOLVEPNP_IPPE if len(obj) >= 4 else cv2.SOLVEPNP_ITERATIVE
    ok, rvec, tvec = cv2.solvePnP(obj, img, K, None, flags=flags)
    if not ok:
        return BasePose(False, markers=sorted(quads), message="PnP no convergió")
    rvec, tvec = cv2.solvePnPRefineLM(obj, img, K, None, rvec, tvec)
    proj, _ = cv2.projectPoints(obj, rvec, tvec, K, None)
    reproj = float(np.sqrt(np.mean(np.sum((proj.reshape(-1, 2) - img) ** 2, axis=1))))
    R, _ = cv2.Rodrigues(rvec)
    T_b2c = np.eye(4)
    T_b2c[:3, :3], T_b2c[:3, 3] = R, tvec.ravel()

    ratio = None
    if zmap is not None:
        ratios = []
        for m in sorted(quads):
            q = quads[m]
            c = q.mean(0)
            mask = np.zeros(zmap.shape, np.uint8)
            cv2.fillConvexPoly(mask, np.round(c + (q - c) * 0.8).astype(np.int32), 1)
            vals = zmap[(mask > 0) & (zmap > 0)]
            if vals.size < 15:
                continue
            centre_b = model[m].mean(0) * scale
            z_pnp = (R @ centre_b + tvec.ravel())[2]
            ratios.append(float(np.median(vals)) / z_pnp)
        if ratios:
            ratio = float(np.median(ratios))
    msg = f"{len(quads)} marcador(es), reproyección {reproj:.2f} px"
    if reproj > 3.0:
        return BasePose(False, markers=sorted(quads), reproj_px=reproj, scale_ratio=ratio,
                        message=msg + ": demasiado error (¿hoja doblada o marcador tapado?)")
    if zmap is not None:
        ref = refine_with_plane(quads, K, zmap, layout, T_b2c, scale)
        if ref is not None:
            T_b2c, s_abs, info = ref
            ratio = s_abs / scale          # relativa a la escala pedida, como la de la profundidad
            msg += f" · plano de la hoja por profundidad ({info['puntos']} pts, {info['rms_mm']:.1f} mm)"
    return BasePose(True, np.linalg.inv(T_b2c), sorted(quads), reproj, ratio, msg)


def _umeyama(src, dst):
    """Semejanza s, R, t que lleva src (N, 3) a dst (N, 3): dst ≈ s R src + t."""
    mu_s, mu_d = src.mean(0), dst.mean(0)
    a, b = src - mu_s, dst - mu_d
    U, S, Vt = np.linalg.svd(b.T @ a / len(src))
    D = np.eye(3)
    if np.linalg.det(U) * np.linalg.det(Vt) < 0:
        D[2, 2] = -1
    R = U @ D @ Vt
    s = float(np.trace(np.diag(S) @ D) / (a ** 2).sum(1).mean())
    return s, R, mu_d - s * R @ mu_s


def refine_with_plane(quads, K, zmap, layout, T_b2c, scale, step=2):
    """Pose de la base con la profundidad: el PnP con marcadores pequeños (lejos o pocos) fija bien
    la posición en la imagen pero mal la distancia y la inclinación. La D455 mide el plano de la
    hoja con miles de puntos: se ajusta ese plano, se cortan con él los rayos de las esquinas de
    los marcadores y una semejanza (Umeyama) entre la hoja nominal y esas esquinas 3D da la pose y
    la escala real de la impresión a la vez. None si no hay plano fiable (se queda el PnP).
    Devuelve (T_base->cámara, escala absoluta, info)."""
    h, w = zmap.shape
    vv, uu = np.mgrid[0:h:step, 0:w:step]
    z = zmap[vv, uu]
    ok = z > 0
    uu, vv, z = uu[ok].astype(np.float64), vv[ok].astype(np.float64), z[ok]
    P = np.column_stack(((uu - K[0, 2]) / K[0, 0] * z, (vv - K[1, 2]) / K[1, 1] * z, z))
    Tc2b = np.linalg.inv(T_b2c)
    Pb = P @ Tc2b[:3, :3].T + Tc2b[:3, 3]
    hw, hh = layout.width_mm / 2000 * scale, layout.height_mm / 2000 * scale
    r_free = layout.free_radius_m() * scale
    sel = (np.abs(Pb[:, 0]) < hw - 0.005) & (np.abs(Pb[:, 2]) < hh - 0.005) & \
          (np.hypot(Pb[:, 0], Pb[:, 2]) > r_free + 0.01) & (np.abs(Pb[:, 1]) < 0.05)
    Q = P[sel]
    if len(Q) < 1500:
        return None
    # plano robusto en el marco de la cámara (mínimos cuadrados con rechazo por MAD)
    keep = np.ones(len(Q), bool)
    for _ in range(4):
        c = Q[keep].mean(0)
        _, _, Vt = np.linalg.svd(Q[keep] - c, full_matrices=False)
        n = Vt[2]
        res = (Q - c) @ n
        mad = float(np.median(np.abs(res[keep] - np.median(res[keep])))) * 1.4826
        keep = np.abs(res) < max(3 * mad, 0.0015)
    if keep.sum() < 1000:
        return None
    rms = float(np.sqrt(np.mean(res[keep] ** 2)))
    if rms > 0.004:
        return None
    d = float(n @ c)
    # esquinas de los marcadores: rayo de cada píxel ∩ plano
    model = layout.marker_corners_3d()
    M = np.vstack([model[m] for m in sorted(quads)])
    px = np.vstack([quads[m] for m in sorted(quads)])
    rays = np.column_stack(((px[:, 0] - K[0, 2]) / K[0, 0], (px[:, 1] - K[1, 2]) / K[1, 1], np.ones(len(px))))
    den = rays @ n
    if np.any(np.abs(den) < 1e-6):
        return None
    X = rays * (d / den)[:, None]
    s, R, t = _umeyama(M, X)
    fit = float(np.sqrt(np.mean(np.sum((s * M @ R.T + t - X) ** 2, axis=1))))
    if fit > 0.004 or not (0.5 < s < 1.5):
        return None
    T = np.eye(4)
    T[:3, :3], T[:3, 3] = R, t
    # la semejanza lleva la hoja NOMINAL a la real: con la escala s, X = R (s·M) + t (metros reales)
    return T, s, {"puntos": int(keep.sum()), "rms_mm": rms * 1000, "esquinas_mm": fit * 1000}
