"""
Sesión de escaneo de un objeto: vistas -> poses -> malla limpia -> biblioteca.

Poses de cada vista (de más a menos fiable):
  base        PnP sobre los marcadores de la hoja (alineado concreto, independiente de la textura)
  calibración Cam 2 de una toma dual: pose de la Cam 1 de esa toma · T(2->1)
  icp         sin base: ICP con color del objeto contra el modelo acumulado
  referencia  primera vista sin base: define un marco provisional sobre la mesa

Todas se refinan con ICP sobre el OBJETO (nunca sobre el fondo) contra el resto de vistas.
La geometría sale del TSDF (profundidad, a escala) y el color de una textura UV horneada con
las fotos de cada toma (textura.py): la D455 da la forma rápido y las fotos, el detalle.
Una pose por base solo acepta correcciones pequeñas; la de calibración se corrige con más
libertad (así una calibración dual imperfecta no deja fantasmas).
"""

import base64
import json
import os
import threading
import time
import traceback

import cv2
import numpy as np
import open3d as o3d

from ..alignment import D455
from . import segment
from .base import LAYOUTS, estimate_pose
from .library import Library
from .optica import rectify_snapshot

reg = o3d.pipelines.registration

ROOT_DIR = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))


def _rot_deg(R):
    return float(np.degrees(np.arccos(np.clip((np.trace(R[:3, :3]) - 1) / 2, -1, 1))))


def _rel_to_root(path):
    """Ruta de la sesión para el manifiesto: relativa al proyecto si está dentro (Saves/...),
    absoluta si no (p. ej. los tests). La biblioteca la usa para re-texturizar."""
    path = os.path.abspath(path)
    root = os.path.abspath(ROOT_DIR)
    try:
        inside = os.path.commonpath([path.lower(), root.lower()]) == root.lower()
    except ValueError:  # otra unidad
        inside = False
    return os.path.relpath(path, root).replace("\\", "/") if inside else path


def _skirt_mask(V, band=(0.015, 0.05), low_h=0.014, cell=0.003, margin=0.012):
    """Vértices de la "falda" pegada al suelo: trozos de hoja o de mesa que el TSDF une a la base
    del objeto (píxeles mezclados en el borde). La huella del objeto se toma de su sección algo más
    arriba (band) y se agranda `margin`; lo que está a menos de `low_h` del suelo y fuera de esa
    huella, sobra. Objetos más anchos abajo que a 1,5-5 cm conservan su base hasta `margin`."""
    sel = (V[:, 1] > band[0]) & (V[:, 1] < band[1])
    if sel.sum() < 100:
        return np.zeros(len(V), bool)
    lo = V[:, [0, 2]].min(0) - 2 * margin
    ij = np.floor((V[:, [0, 2]] - lo) / cell).astype(int)
    shape = tuple(ij.max(0) + 1)
    occ = np.zeros(shape, np.uint8)
    occ[ij[sel, 0], ij[sel, 1]] = 1
    k = 2 * int(np.ceil(margin / cell)) + 1
    occ = cv2.morphologyEx(occ, cv2.MORPH_CLOSE, np.ones((5, 5), np.uint8))
    cnts, _ = cv2.findContours(occ, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    fill = np.zeros_like(occ)
    cv2.drawContours(fill, cnts, -1, 1, -1)                 # la sección es un anillo: rellenarlo
    fill = cv2.dilate(fill, cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (k, k)))
    return (V[:, 1] < low_h) & (fill[ij[:, 0], ij[:, 1]] == 0)


def view_dir(T):
    """Dirección unitaria del centro de la base hacia la cámara (marco base)."""
    c = np.asarray(T)[:3, 3]
    return c / max(np.linalg.norm(c), 1e-9)


def view_angles(T):
    """Azimut y elevación (°) de la cámara vista desde el centro de la base, y distancia (m)."""
    c = np.asarray(T)[:3, 3]
    return {"az": round(float(np.degrees(np.arctan2(c[0], c[2]))), 1),
            "el": round(float(np.degrees(np.arctan2(c[1], np.hypot(c[0], c[2])))), 1),
            "dist_m": round(float(np.linalg.norm(c)), 3)}


def _thumb(bgr, width=200):
    h, w = bgr.shape[:2]
    small = cv2.resize(bgr, (width, int(h * width / w)), interpolation=cv2.INTER_AREA)
    ok, buf = cv2.imencode(".jpg", small, [cv2.IMWRITE_JPEG_QUALITY, 75])
    return "data:image/jpeg;base64," + base64.b64encode(buf.tobytes()).decode() if ok else None


class ObjView:
    def __init__(self, idx, shot, cam, snap):
        self.idx, self.shot, self.cam = idx, shot, cam
        self.snap = snap
        self.T = None               # cámara (color) -> base
        self.source = None
        self.base_info = None
        self.mask = None
        self.pts_cam = None
        self.cols = None
        self.quality = {}
        self.thumb = None
        self.excluded = False       # fuera de la malla (p. ej. Cam 2 sin calibración creíble)
        self.T21 = None             # Cam 2: extrínseca usada como punto de partida
        self.sibling = None         # Cam 2: la vista de la Cam 1 de la misma toma

    @property
    def label(self):
        return f"Toma {self.shot + 1}" + (" · Cam 2" if self.cam == "cam2" else "")

    def object_cloud_base(self, voxel):
        p = o3d.geometry.PointCloud(o3d.utility.Vector3dVector(segment.transform(self.T, self.pts_cam)))
        p.colors = o3d.utility.Vector3dVector(self.cols.astype(np.float64) / 255.0)
        return p.voxel_down_sample(voxel)

    def object_cloud_cam(self, voxel):
        p = o3d.geometry.PointCloud(o3d.utility.Vector3dVector(self.pts_cam))
        p.colors = o3d.utility.Vector3dVector(self.cols.astype(np.float64) / 255.0)
        return p.voxel_down_sample(voxel)

    def summary(self):
        return {"index": self.idx, "toma": self.shot, "camara": self.cam, "etiqueta": self.label,
                "origen_pose": self.source, "base": self.base_info, "puntos": 0 if self.pts_cam is None else len(self.pts_cam),
                "calidad": self.quality, "thumb": self.thumb,
                "angulos": view_angles(self.T) if self.T is not None else None}


def _prep(pcd, voxel):
    pcd.estimate_normals(o3d.geometry.KDTreeSearchParamHybrid(radius=voxel * 3, max_nn=30))
    return pcd


def icp_object(src_cam, tgt_base, init_T, voxel, scales=(4, 2, 1), colored=False):
    """T (cámara -> base) que alinea la nube del objeto de una vista con el modelo.

    Por defecto point-to-plane con kernel robusto (solo geometría). El ICP con color está
    disponible pero NO se usa por defecto: con luz fija y objeto que gira el sombreado cambia
    entre tomas y el término de color sesga la pose (medido: 33 mm frente a 0,5 mm)."""
    T = np.asarray(init_T, np.float64)
    for s in scales:
        v = voxel * s
        src = _prep(src_cam.voxel_down_sample(v), v)
        tgt = _prep(tgt_base.voxel_down_sample(v), v)
        if len(src.points) < 30 or len(tgt.points) < 30:
            continue
        crit = reg.ICPConvergenceCriteria(relative_fitness=1e-6, relative_rmse=1e-6, max_iteration=40)
        try:
            if colored:
                r = reg.registration_colored_icp(src, tgt, v * 1.5, T, reg.TransformationEstimationForColoredICP(), crit)
            else:
                raise RuntimeError
        except RuntimeError:
            r = reg.registration_icp(src, tgt, v * 1.5, T, reg.TransformationEstimationPointToPlane(reg.TukeyLoss(k=v)), crit)
        if r.fitness > 0:
            T = np.asarray(r.transformation)
    src = src_cam.voxel_down_sample(voxel)
    ev = reg.evaluate_registration(src, tgt_base.voxel_down_sample(voxel), voxel * 1.5, T)
    return T, float(ev.fitness), float(ev.inlier_rmse)


def fpfh_global(src_cam, tgt_base, voxel):
    v = voxel * 2
    src = _prep(src_cam.voxel_down_sample(v), v)
    tgt = _prep(tgt_base.voxel_down_sample(v), v)
    feat = o3d.geometry.KDTreeSearchParamHybrid(radius=v * 5, max_nn=100)
    fs, ft = reg.compute_fpfh_feature(src, feat), reg.compute_fpfh_feature(tgt, feat)
    o3d.utility.random.seed(0)
    res = reg.registration_ransac_based_on_feature_matching(
        src, tgt, fs, ft, True, v * 1.5, reg.TransformationEstimationPointToPoint(False), 3,
        [reg.CorrespondenceCheckerBasedOnEdgeLength(0.9), reg.CorrespondenceCheckerBasedOnDistance(v * 1.5)],
        reg.RANSACConvergenceCriteria(100000, 0.999))
    return np.asarray(res.transformation)


class ObjectScanSession:
    MAX_SHOTS = 48

    def __init__(self, name="Objeto", layout="A3", volume=None, use_base=True, base_dir=ROOT_DIR,
                 library=None, log=print, sensor=segment.D455_OBJ, save_views=True):
        self.name = name or "Objeto"
        self.layout = LAYOUTS.get(layout, LAYOUTS["A3"])
        self.volume = volume or segment.WorkVolume()
        self.use_base = use_base
        self.sensor = sensor
        self.log = log
        self.library = library or Library()
        date = time.strftime("%Y-%m-%d")
        self.id = time.strftime("objeto_%H%M%S")
        self.dir = os.path.join(base_dir, "Saves", date, self.id)
        self.rel_dir = _rel_to_root(self.dir)
        self.save_views = save_views
        self.views = []
        self.shots = 0
        self.scale = 1.0              # hoja impresa a escala distinta del 100 %
        self._scales = []
        self.state = "capturing"      # capturing | processing | done
        self.progress, self.stage = 0.0, ""
        self.result, self.error = None, None
        self.voxel = 0.003
        self.cam2_corr = np.eye(4)    # corrección común de la calibración dual (marco de la Cam 2)
        self.calib_info = None
        self._lock = threading.Lock()

    @classmethod
    def from_folder(cls, folder, name=None, library=None, log=print, volume=None):
        """Sesión ya capturada (Saves/<fecha>/objeto_HHMMSS) lista para volver a procesar con el
        código actual: las vistas guardadas (color, profundidad, pose de la toma) sin la cámara."""
        views = []
        for d in sorted(os.listdir(folder)):
            vd = os.path.join(folder, d)
            if d.startswith("vista_") and os.path.isfile(os.path.join(vd, "vista.json")):
                with open(os.path.join(vd, "vista.json"), encoding="utf-8") as f:
                    views.append((vd, json.load(f)))
        if not views:
            raise ValueError(f"No hay vistas guardadas en {folder}")
        j0 = views[0][1]
        if volume is None and j0.get("volumen"):
            volume = segment.WorkVolume(j0["volumen"]["radio_cm"] / 100, j0["volumen"]["altura_cm"] / 100)
        s = cls(name or os.path.basename(folder), layout=j0.get("hoja", "A3"), volume=volume,
                use_base=j0.get("usar_base", True), library=library, log=log, save_views=False)
        s.dir = os.path.abspath(folder)
        s.rel_dir = _rel_to_root(s.dir)
        for vd, j in views:
            color = cv2.imread(os.path.join(vd, "color.jpg"))
            depth = cv2.imread(os.path.join(vd, "profundidad.png"), cv2.IMREAD_UNCHANGED)
            if color is None or depth is None or not j.get("T_cam_a_base"):
                continue
            snap = {"color": cv2.cvtColor(color, cv2.COLOR_BGR2RGB), "depth": depth, "intrinsics": j["intrinsics"],
                    "depth_scale": j["depth_scale"], "burst": j.get("rafaga", 1)}
            v = ObjView(len(s.views), j.get("toma", len(s.views)), j.get("camara", "cam1"), snap)
            v.T, v.source, v.base_info = np.array(j["T_cam_a_base"], np.float64), j.get("origen_pose") or "base", j.get("base")
            s._segment(v)
            if v.pts_cam is not None and len(v.pts_cam) >= 400:
                s.views.append(v)
        s.shots = len({v.shot for v in s.views})
        if s.views:
            s.voxel = s._auto_voxel(s.views[0])
        return s

    # ------------------------------------------------------------------ #
    # Captura
    # ------------------------------------------------------------------ #
    def add_snapshot(self, snap):
        """Añade una toma (simple o dual). Devuelve el resumen de la toma con su consejo."""
        with self._lock:
            if self.state != "capturing":
                raise RuntimeError("La sesión no está capturando.")
            if self.shots >= self.MAX_SHOTS:
                raise RuntimeError(f"Máximo {self.MAX_SHOTS} tomas por objeto.")
            shot = self.shots
            # simple (sin cam1/cam2), dual, o dual con una sola cámara en vivo
            parts = [(k, snap[k]) for k in ("cam1", "cam2") if snap.get(k) is not None] or [("cam1", snap)]
            new = []
            for cam, s in parts:
                v = ObjView(len(self.views) + len(new), shot, cam, rectify_snapshot(s))
                self._pose_and_segment(v, new, snap.get("extrinsics"))
                new.append(v)
            if not any(v.T is not None and v.pts_cam is not None and len(v.pts_cam) >= 400 for v in new):
                reasons = "; ".join(v.quality.get("motivo", "sin objeto") for v in new)
                raise ValueError(f"Toma descartada: {reasons}")
            usable = [v for v in new if v.T is not None and v.pts_cam is not None and len(v.pts_cam) >= 400]
            for v in usable:
                if self.save_views:
                    self._save_view(v)
            self.views.extend(usable)
            self.shots += 1
            return {"toma": shot, "vistas": [v.summary() for v in usable], "consejo": self._hint(usable),
                    "descartadas": [v.label + ": " + v.quality.get("motivo", "") for v in new if v not in usable]}

    def _model(self, exclude=(), voxel=None):
        voxel = voxel or self.voxel
        m = o3d.geometry.PointCloud()
        for w in self.views:
            if w not in exclude and w.T is not None:
                m += w.object_cloud_base(voxel)
        return m.voxel_down_sample(voxel)

    def _pose_and_segment(self, v, siblings, T_2to1):
        s = v.snap
        bgr = cv2.cvtColor(s["color"], cv2.COLOR_RGB2BGR)
        zmap = s["depth"].astype(np.float64) * s["depth_scale"]
        # 1) base de marcadores
        if self.use_base:
            bp = estimate_pose(bgr, s["intrinsics"], self.layout, zmap=zmap, scale=self.scale)
            v.base_info = bp.public()
            if bp.ok:
                if bp.scale_ratio and not self._scales:
                    self._update_scale(bp.scale_ratio)
                    if abs(self.scale - 1) > 1e-6:  # recalcular con la escala corregida
                        bp = estimate_pose(bgr, s["intrinsics"], self.layout, zmap=zmap, scale=self.scale)
                elif bp.scale_ratio:
                    self._update_scale(bp.scale_ratio * self.scale)
                if bp.ok:
                    v.T, v.source = bp.T_cam_to_base, "base"
        # 2) Cam 2 de una toma dual: calibración como estimación inicial
        if v.T is None and v.cam == "cam2" and T_2to1 is not None:
            sib = next((w for w in siblings if w.cam == "cam1" and w.T is not None), None)
            if sib is not None:
                v.T21, v.sibling = np.asarray(T_2to1, np.float64), sib
                v.T, v.source = sib.T @ v.T21 @ self.cam2_corr, "calibración"
        # 3) primera vista sin base: marco provisional
        has_model = any(w.T is not None for w in self.views + siblings)
        if v.T is None and not has_model:
            T, vol = segment.frame_without_base(s, sensor=self.sensor)
            if T is None:
                v.quality["motivo"] = vol
                return
            v.T, v.source = T, "referencia"
            if not self.use_base or not self._scales:
                self.volume = vol
        # 4) ICP contra el modelo
        if v.T is None:
            T_guess, _ = segment.frame_without_base(s, sensor=self.sensor)
            prev = next((w for w in reversed(self.views + siblings) if w.T is not None), None)
            init = prev.T if prev is not None else np.eye(4)
            seg = segment.segment_with_base(s, init, self._grown_volume(1.6), sensor=self.sensor)
            if len(seg["points_cam"]) < 300 and T_guess is not None:
                pts, vv, uu = segment.pixel_points(s["depth"], s["depth_scale"], s["intrinsics"])
                Pb = segment.transform(T_guess, pts)
                sel = segment.in_volume(Pb, segment.WorkVolume(0.6, 1.0, 0.006))
                seg = {"points_cam": pts[sel], "colors": s["color"][vv[sel], uu[sel]]}
            if len(seg["points_cam"]) < 300:
                v.quality["motivo"] = "sin base y sin objeto reconocible para alinear"
                return
            src = o3d.geometry.PointCloud(o3d.utility.Vector3dVector(seg["points_cam"]))
            src.colors = o3d.utility.Vector3dVector(seg["colors"].astype(np.float64) / 255)
            model = self._model(voxel=self.voxel)
            T, fit, rmse, how = self._pose_without_base(v, src, model, prev, T_guess)
            v.quality["semilla"] = how
            if fit < 0.3:
                v.quality.update(motivo=f"no encaja con las vistas anteriores ({fit * 100:.0f} % de solape): "
                                        "gira menos entre tomas", fitness=round(fit, 3))
                return
            v.T, v.source = T, "icp"
        # Segmentar con la pose y refinar contra el modelo
        self._segment(v)
        if v.pts_cam is None or len(v.pts_cam) < 400:
            v.quality["motivo"] = "casi no hay objeto dentro del volumen de trabajo"
            return
        same = self._same_view(v)
        if same is not None:
            w, overlap = same
            if overlap < 0.6:
                v.quality["motivo"] = (f"misma vista que la {w.label.lower()} pero el objeto se movió sobre la hoja: "
                                       "gira la HOJA con el objeto encima (no el objeto solo) o mueve la cámara")
            else:
                v.quality["motivo"] = (f"misma vista que la {w.label.lower()}: gira la hoja con el objeto ~30° "
                                       "o mueve la cámara antes de la siguiente toma")
            v.T = None
            return
        if self.voxel == 0.003:
            self.voxel = self._auto_voxel(v)
        self._refine(v, siblings)
        v.thumb = _thumb(segment.preview_image(s, v.mask))

    # ---- alineado sin base: hipótesis + puntuación con geometría y color ----
    def _pose_without_base(self, v, src, model, prev, T_guess):
        """Hipótesis: pose anterior, SIFT entre tomas (sobre el objeto), barrido de giro alrededor
        del eje vertical del objeto y FPFH. Cada una se puntúa con geometría + color (el color
        rompe la simetría de las piezas de revolución) y las mejores se refinan con ICP con color."""
        from scipy.spatial import cKDTree
        vox = self.voxel
        mdl = model.voxel_down_sample(vox)
        tree = cKDTree(np.asarray(mdl.points))
        def chroma(c):  # color sin intensidad: no cambia con el sombreado
            return c / np.maximum(c.sum(1, keepdims=True), 1e-3)

        mcol = chroma(np.asarray(mdl.colors))
        s_d = src.voxel_down_sample(vox)
        sp, sc = np.asarray(s_d.points), chroma(np.asarray(s_d.colors))

        def score(T):
            q = sp @ T[:3, :3].T + T[:3, 3]
            d, i = tree.query(q, distance_upper_bound=vox * 2)
            ok = np.isfinite(d)
            if ok.sum() < 20:
                return -1.0, 0.0
            fit = ok.mean()
            cerr = float(np.mean(np.abs(sc[ok] - mcol[i[ok]])))
            return fit - 3.0 * cerr, fit

        seeds = [("pose anterior", prev.T)] if prev is not None else []
        T_sift = self._sift_seed(v, prev) if prev is not None else None
        if T_sift is not None:
            seeds.append(("SIFT entre tomas", T_sift))
        c = np.asarray(mdl.points).mean(0)
        if prev is not None:
            for ang in range(-90, 91, 6):
                if ang == 0:
                    continue
                a = np.radians(ang)
                M = np.eye(4)
                M[[0, 0, 2, 2], [0, 2, 0, 2]] = np.cos(a), np.sin(a), -np.sin(a), np.cos(a)
                C, Ci = np.eye(4), np.eye(4)
                C[:3, 3], Ci[:3, 3] = c, -c
                seeds.append((f"giro {ang:+d}°", C @ M @ Ci @ prev.T))
        try:
            seeds.append(("FPFH", fpfh_global(src, model, vox)))
        except RuntimeError:
            pass
        ranked = sorted(seeds, key=lambda s: score(s[1])[0], reverse=True)
        best = None
        for name, T0 in ranked[:4]:
            T, fit, rmse = icp_object(src, model, T0, vox, scales=(2, 1))
            sc_val = score(T)[0]
            if best is None or sc_val > best[0]:
                best = (sc_val, T, fit, rmse, name)
        _, T, fit, rmse, name = best
        return T, fit, rmse, name

    def _sift_seed(self, v, prev):
        """T (cámara actual -> base) desde SIFT 3D entre esta vista y la anterior, solo con
        puntos del objeto en la vista anterior (en plato giratorio la mesa no se mueve y votaría
        por "no hubo giro")."""
        from ..alignment import AlignOptions, RGBDView, match_features, ransac_rigid, _match_thresholds
        try:
            va, vb = RGBDView.from_snapshot(prev.snap, "prev"), RGBDView.from_snapshot(v.snap, "cur")
            opt = AlignOptions()
            P1, P2, matches, kp1, kp2, _ = match_features(va, vb, self.sensor, opt)
            if len(P1) < 8:
                return None
            u1 = np.array([kp1[m.queryIdx].pt for m in matches]).round().astype(int)
            m_prev = cv2.dilate(prev.mask.astype(np.uint8), np.ones((9, 9), np.uint8)) > 0
            on_obj = m_prev[np.clip(u1[:, 1], 0, m_prev.shape[0] - 1), np.clip(u1[:, 0], 0, m_prev.shape[1] - 1)]
            if on_obj.sum() < 8:
                return None
            P1, P2 = P1[on_obj], P2[on_obj]
            thr = _match_thresholds(P1, P2, self.sensor, va, vb, opt.min_threshold_m)
            T21, inl = ransac_rigid(P2, P1, thr, 2000, np.random.default_rng(0), self.sensor)
            if T21 is None or inl.sum() < 8 or inl.mean() < 0.3:
                return None
            return prev.T @ T21   # cámara actual -> cámara anterior -> base
        except Exception:  # noqa: BLE001
            return None

    def _grown_volume(self, k):
        vol = self.volume
        return segment.WorkVolume(vol.radius_m * k, vol.height_m * k, vol.floor_m)

    def _segment(self, v):
        seg = segment.segment_with_base(v.snap, v.T, self.volume, sensor=self.sensor)
        v.mask, v.pts_cam, v.cols = seg["mask"], seg["points_cam"], seg["colors"]

    def novelty_deg(self, T, cam="cam1"):
        """Separación angular (°) entre la dirección de la cámara T (cámara -> base) y la de la
        vista ya tomada más parecida. Grande = vista nueva que merece una toma."""
        dirs = [view_dir(w.T) for w in self.views if w.cam == cam and w.T is not None and w.source == "base"]
        if not dirs:
            return 180.0
        d = view_dir(T)
        return float(np.degrees(np.arccos(np.clip(max(float(d @ e) for e in dirs), -1, 1))))

    def _same_view(self, v, max_mm=15.0, max_deg=4.0):
        """(vista anterior, solape del objeto) si v se tomó desde la misma pose respecto a la base
        que otra vista de la misma cámara; None si es una vista nueva. Con la base, una vista
        nueva exige mover la cámara o girar la hoja: si la pose no cambia, o la toma sobra o el
        objeto giró sobre la hoja (y fundirlas lo convertiría en un borrón)."""
        if v.source != "base":
            return None
        for w in self.views:
            if w.cam != v.cam or w.source != "base" or w.T is None:
                continue
            d = np.linalg.inv(w.T) @ v.T
            if np.linalg.norm(d[:3, 3]) * 1000 > max_mm or _rot_deg(d) > max_deg:
                continue
            a = segment.transform(v.T, v.pts_cam)
            b = o3d.geometry.PointCloud(o3d.utility.Vector3dVector(segment.transform(w.T, w.pts_cam)))
            tree = o3d.geometry.KDTreeFlann(b)
            step = max(1, len(a) // 3000)
            near = [tree.search_knn_vector_3d(p, 1)[2][0] for p in a[::step]]
            overlap = float(np.mean(np.sqrt(near) < max(0.004, 2.5 * self.voxel)))
            return w, overlap
        return None

    def _auto_voxel(self, v):
        """Vóxel según el ruido real de la vista: σz(z) de la D455 reducido por la ráfaga de cuadros
        (no del todo: parte del ruido es fijo por píxel). 1.5-4 mm."""
        z = float(np.median(v.pts_cam[:, 2]))
        n = max(1, int(v.snap.get("burst", 1)))
        sigma = self.sensor.sigma_z(z) * (1.0 if n == 1 else 1.4 / np.sqrt(n))
        return float(np.clip(1.5 * sigma + 0.001, 0.0015, 0.004))

    def _update_scale(self, ratio):
        self._scales.append(float(ratio))
        s = float(np.median(self._scales))
        self.scale = s if abs(s - 1) > 0.015 else 1.0

    def _refine(self, v, siblings=(), final=False):
        """Ajuste fino de la pose de la vista contra las demás (solo el objeto).

        Contra qué se ajusta: una vista por marcadores, contra las otras por marcadores (exactas);
        el resto, contra las que NO dependen de la calibración dual (para que una calibración
        mala no se refuerce a sí misma)."""
        pool = [w for w in self.views + list(siblings) if w is not v and w.T is not None and w.pts_cam is not None]
        if v.source == "base":
            others = [w for w in pool if w.source == "base"] or pool
        else:
            others = [w for w in pool if w.source != "calibración"] or pool
        if not others:
            v.quality.update(fitness=None, rmse_mm=None, correccion_mm=0.0, correccion_deg=0.0)
            return False
        model = o3d.geometry.PointCloud()
        for w in others:
            model += w.object_cloud_base(self.voxel)
        model = model.voxel_down_sample(self.voxel)
        src = v.object_cloud_cam(self.voxel)
        T0 = v.T
        _, fit0, rmse0 = icp_object(src, model, T0, self.voxel, scales=())
        # pose por marcadores: retoques finos; la de calibración puede estar a varios cm
        if v.source == "base" or final:
            scales = (2, 1)
            limits = (8.0, 2.0) if v.source == "base" else (12.0, 2.5)
        elif v.source == "calibración":
            scales, limits = (8, 4, 2, 1), (150.0, 25.0)
        else:
            scales, limits = (4, 2, 1), (1e9, 360.0)
        T, fit, rmse = icp_object(src, model, T0, self.voxel, scales=scales)
        dT = np.linalg.inv(T0) @ T
        dmm, ddeg = float(np.linalg.norm(dT[:3, 3]) * 1000), _rot_deg(dT)
        improves = (fit >= fit0 - 0.01 and rmse <= rmse0 * 0.97) or (fit0 < 0.2 and fit >= 0.35)
        accepted = fit >= 0.25 and dmm <= limits[0] and ddeg <= limits[1] and improves
        if accepted:
            v.T = T
        else:
            fit, rmse = fit0, rmse0
        v.quality.update(fitness=round(fit, 3), rmse_mm=round(rmse * 1000, 2),
                         correccion_mm=round(dmm if accepted else 0.0, 1),
                         correccion_deg=round(ddeg if accepted else 0.0, 2), refinada=accepted)
        if accepted and v.source == "calibración" and not final:
            self._update_cam2_corr(extra=[v])
        return accepted

    # ---- corrección común de la calibración dual ----
    def _cam2_samples(self, extra=()):
        out = []
        for v in list(self.views) + [e for e in extra if e not in self.views]:
            if v.source == "calibración" and v.T21 is not None and v.sibling is not None and v.quality.get("refinada"):
                out.append(np.linalg.inv(v.sibling.T @ v.T21) @ v.T)
        return out

    @staticmethod
    def _robust_mean(Ts):
        """Media robusta de transformaciones: mediana de traslación y media de rotaciones,
        descartando las que se alejan más de 2 cm / 3 grados de la mediana."""
        from scipy.spatial.transform import Rotation
        Ts = [np.asarray(T) for T in Ts]
        t = np.median([T[:3, 3] for T in Ts], axis=0)
        R = Rotation.from_matrix([T[:3, :3] for T in Ts])
        Rm = R.mean()
        keep = [k for k, T in enumerate(Ts)
                if np.linalg.norm(T[:3, 3] - t) < 0.02 and (Rm.inv() * R[k]).magnitude() < np.radians(3)]
        if keep:
            t = np.mean([Ts[k][:3, 3] for k in keep], axis=0)
            Rm = R[keep].mean()
        C = np.eye(4)
        C[:3, :3], C[:3, 3] = Rm.as_matrix(), t
        return C

    def _update_cam2_corr(self, extra=()):
        samples = self._cam2_samples(extra)
        if samples:
            self.cam2_corr = self._robust_mean(samples)

    MAX_CALIB_FIX = (0.15, 30.0)   # una corrección de calibración mayor que esto no es creíble (m, grados)

    def _joint_cam2_correction(self):
        """Una sola corrección de la calibración para TODAS las tomas.

        Cada vista de la Cam 2 se alinea (ICP desde la calibración y, si hace falta, desde un
        registro global FPFH) contra el modelo de las vistas que no dependen de la calibración.
        Solo se aplica si varias tomas coinciden; si no, las vistas de la Cam 2 que no encajan
        se dejan fuera de la malla (mejor menos datos que un objeto con fantasma)."""
        cam2 = [v for v in self.views if v.source == "calibración" and v.T21 is not None and v.sibling is not None]
        if not cam2:
            return None
        ref = [w for w in self.views if w.source != "calibración" and w.pts_cam is not None]
        model = o3d.geometry.PointCloud()
        for w in ref:
            model += w.object_cloud_base(self.voxel)
        model = model.voxel_down_sample(self.voxel)
        samples = []
        for v in cam2:
            T_cal = v.sibling.T @ v.T21 @ self.cam2_corr
            v.T = T_cal
            self._segment(v)
            src = v.object_cloud_cam(self.voxel)
            cands = [icp_object(src, model, T_cal, self.voxel, scales=(8, 4, 2, 1))]
            if cands[0][1] < 0.6:
                try:
                    cands.append(icp_object(src, model, fpfh_global(src, model, self.voxel), self.voxel, scales=(4, 2, 1)))
                except RuntimeError:
                    pass
            for T, fit, _ in sorted(cands, key=lambda c: -c[1]):
                C = np.linalg.inv(v.sibling.T @ v.T21) @ T
                if fit >= 0.35 and np.linalg.norm(C[:3, 3]) <= self.MAX_CALIB_FIX[0] and _rot_deg(C) <= self.MAX_CALIB_FIX[1]:
                    samples.append(C)
                    break
        info = {"tomas": len(cam2), "muestras": len(samples), "aplicada": False}
        need = max(2, int(np.ceil(0.25 * len(cam2))))
        if len(samples) >= need:
            C = self._robust_mean(samples)
            agree = sum(1 for S in samples if np.linalg.norm(S[:3, 3] - C[:3, 3]) < 0.02
                        and _rot_deg(np.linalg.inv(C) @ S) < 3.0)
            if agree >= need:
                self.cam2_corr = C
                info.update(aplicada=True, coinciden=agree)
        for v in cam2:
            v.T = v.sibling.T @ v.T21 @ self.cam2_corr
            self._segment(v)
            if not info["aplicada"]:
                # sin consenso: fuera las vistas de la Cam 2 que no encajan con la calibración vigente
                _, fit0, _ = icp_object(v.object_cloud_cam(self.voxel), model, v.T, self.voxel, scales=())
                if fit0 < 0.35:
                    v.quality["motivo"] = "Cam 2 descartada: no encaja con la calibración dual (recalíbrala)"
                    v.excluded = True
        C = self.cam2_corr
        info.update(mm=round(float(np.linalg.norm(C[:3, 3]) * 1000), 1), deg=round(_rot_deg(C), 2),
                    T_2a1_corregida=(cam2[0].T21 @ C).tolist())
        return info

    def _hint(self, new):
        v = new[0]
        q = v.quality
        if len(self.views) <= len(new):
            base = "con base de marcadores" if v.source == "base" else "sin base (alineado por geometría)"
            if v.source == "base":
                return (f"Primera toma guardada {base}. Gira la HOJA con el objeto encima ~30° (o muévete "
                        "alrededor) y toma la siguiente. No gires el objeto solo.")
            return f"Primera toma guardada {base}. Gira el objeto (o muévete) ~30° y toma la siguiente."
        fit = q.get("fitness")
        if fit is None:
            return "Toma guardada."
        pct = round(fit * 100)
        if v.source == "base":
            return (f"Encaje {pct} % con lo ya escaneado (pose por marcadores). Sigue girando la hoja ~30°; "
                    "al completar la vuelta, sube o baja la cámara y da otra vuelta para ver arriba y los lados.")
        if fit >= 0.6:
            return f"Encaje {pct} %: bien. Gira ~25-30° más."
        return f"Encaje {pct} %: justo. Gira menos (~15-20°) en la próxima toma."

    def undo(self):
        with self._lock:
            if self.state != "capturing" or not self.views:
                raise RuntimeError("No hay tomas que deshacer.")
            last = self.views[-1].shot
            self.views = [v for v in self.views if v.shot != last]
            self.shots = last
            return last

    def set_volume(self, radius_m=None, height_m=None):
        with self._lock:
            if radius_m is not None:
                self.volume.radius_m = float(np.clip(radius_m, 0.03, 0.8))
            if height_m is not None:
                self.volume.height_m = float(np.clip(height_m, 0.03, 1.2))
            for v in self.views:  # re-segmentar las vistas ya tomadas
                if v.T is not None:
                    self._segment(v)
            return self.volume.public()

    # ------------------------------------------------------------------ #
    # Procesado
    # ------------------------------------------------------------------ #
    def finish(self, background=True, target_triangles=150000, texture=True):
        with self._lock:
            if self.state != "capturing":
                raise RuntimeError("La sesión ya se está procesando o terminó.")
            if len({v.shot for v in self.views}) < 2:
                raise RuntimeError("Hacen falta al menos 2 tomas válidas.")
            self.state, self.error, self.progress, self.stage = "processing", None, 0.0, "Preparando..."
        if background:
            threading.Thread(target=self._process, args=(target_triangles, texture), name="ObjScan",
                             daemon=True).start()
        else:
            self._process(target_triangles, texture)

    def _stage(self, text, p):
        self.stage, self.progress = text, float(p)
        self.log(f"[Objeto] {int(p * 100):3d}% {text}")

    def _process(self, target_triangles, texture=True):
        t0 = time.time()
        try:
            # a) poses: primero las independientes de la calibración (marcadores o geometría),
            #    luego una corrección común para la Cam 2 y al final retoques finos de todas
            n = len(self.views)
            indep = [v for v in self.views if v.source != "calibración"]
            for k, v in enumerate(indep):
                self._stage(f"Ajustando poses ({k + 1}/{len(indep)})", 0.02 + 0.15 * k / max(1, len(indep)))
                self._refine(v)
            self._stage("Corrigiendo la calibración de la Cam 2", 0.18)
            self.calib_info = self._joint_cam2_correction()
            for k, v in enumerate(self.views):
                self._stage(f"Retoque final ({k + 1}/{n})", 0.22 + 0.13 * k / n)
                if not v.excluded:
                    self._refine(v, final=True)
            for v in self.views:
                self._segment(v)

            # b) volumen TSDF uniforme solo sobre el objeto
            self._stage("Integrando volumen (TSDF)", 0.4)
            mesh, pts = self._integrate()

            # c) limpieza, decimado y pivote
            self._stage("Limpiando malla", 0.7)
            mesh, offset = self._clean(mesh, target_triangles)
            pts.translate(offset)

            # d) textura fotográfica (atlas UV + fotos de las tomas)
            tex, tex_info = None, None
            if texture:
                self._stage("Texturizando con las fotos", 0.78)
                try:
                    mesh, tex, tex_info = self._texture(mesh, offset)
                except Exception as e:  # noqa: BLE001  (sin textura el objeto sigue valiendo)
                    traceback.print_exc()
                    tex_info = {"error": str(e)}

            # d) biblioteca
            self._stage("Guardando en la biblioteca", 0.9)
            fits = [v.quality.get("fitness") for v in self.views if v.quality.get("fitness") is not None]
            meta = {
                "sesion": self.rel_dir,
                "tomas": len({v.shot for v in self.views}),
                "vistas": len(self.views),
                "poses": {s: sum(1 for v in self.views if v.source == s) for s in ("base", "calibración", "icp", "referencia")},
                "encaje_medio": round(float(np.mean(fits)), 3) if fits else None,
                "escala_hoja": round(self.scale, 4),
                "voxel_mm": round(self.voxel * 1000, 2),
                "correccion_calibracion_dual": self.calib_info,
                "pivote_desplazado_m": [round(float(x), 4) for x in offset],
                "rafaga_cuadros": int(np.median([v.snap.get("burst", 1) for v in self.views])),
                "volumen": self.volume.public(),
                "textura": tex_info,
                "tiempo_proceso_s": round(time.time() - t0, 1),
            }
            man = self.library.create(self.name, mesh, pts, meta, texture=tex)
            if self.save_views and os.path.isdir(self.dir):
                with open(os.path.join(self.dir, "resumen_objeto.json"), "w", encoding="utf-8") as f:
                    json.dump({"objeto": man["id"], **meta,
                               "vistas_detalle": [{**v.summary(), "thumb": None, "T_cam_a_base": v.T.tolist()}
                                                  for v in self.views]}, f, indent=2, ensure_ascii=False)
            self.result = man
            self._stage("Completado", 1.0)
            self.state = "done"
        except Exception as e:  # noqa: BLE001
            traceback.print_exc()
            self.error = f"Error al generar el objeto: {e}"
            self.stage, self.progress, self.state = "", 0.0, "capturing"

    def _integrate(self):
        used = [v for v in self.views if not v.excluded]
        P = np.vstack([segment.transform(v.T, v.pts_cam) for v in used])
        lo, hi = np.percentile(P, 0.5, axis=0) - 0.02, np.percentile(P, 99.5, axis=0) + 0.02
        lo[1] = min(lo[1], -0.005)
        length = float(np.max(hi - lo))
        voxel = max(self.voxel * 0.6, length / 256)
        res = int(np.clip(np.ceil(length / voxel), 64, 256))
        voxel = length / res
        vol = o3d.pipelines.integration.UniformTSDFVolume(
            length=length, resolution=res, sdf_trunc=voxel * 4,
            color_type=o3d.pipelines.integration.TSDFVolumeColorType.RGB8, origin=lo.reshape(3, 1))
        pts = o3d.geometry.PointCloud()
        for k, v in enumerate(used):
            self._stage(f"Integrando vista {k + 1}/{len(used)}", 0.4 + 0.3 * k / len(used))
            s = v.snap
            m = cv2.dilate(v.mask.astype(np.uint8), np.ones((3, 3), np.uint8)) > 0
            depth = np.where(m, s["depth"], 0).astype(np.uint16)
            rgbd = o3d.geometry.RGBDImage.create_from_color_and_depth(
                o3d.geometry.Image(np.ascontiguousarray(s["color"])), o3d.geometry.Image(depth),
                depth_scale=1.0 / s["depth_scale"], depth_trunc=3.0, convert_rgb_to_intensity=False)
            intr = s["intrinsics"]
            pin = o3d.camera.PinholeCameraIntrinsic(int(intr["width"]), int(intr["height"]), intr["fx"], intr["fy"],
                                                    intr["ppx"], intr["ppy"])
            vol.integrate(rgbd, pin, np.linalg.inv(v.T))
            pts += v.object_cloud_base(self.voxel)
        mesh = vol.extract_triangle_mesh()
        return mesh, pts.voxel_down_sample(self.voxel)

    def _texture(self, mesh, offset):
        from .textura import TexView, bake
        To = np.eye(4)
        To[:3, 3] = offset
        views = [TexView(v.snap["color"], v.snap["intrinsics"], To @ v.T, v.mask)
                 for v in self.views if not v.excluded and v.T is not None]
        return bake(mesh, views, log=lambda m: self.log(f"[Objeto] textura: {m}"))

    def _clean(self, mesh, target_triangles):
        mesh.remove_degenerate_triangles()
        mesh.remove_duplicated_vertices()
        mesh.remove_unreferenced_vertices()
        V = np.asarray(mesh.vertices)
        if len(V) == 0:
            raise RuntimeError("El volumen no generó superficie: revisa que el objeto esté dentro del volumen de trabajo.")
        # fuera lo que queda por debajo del papel (restos de la mesa)
        low = V[:, 1] < self.volume.floor_m * 0.5
        mesh.remove_vertices_by_mask(low)
        mesh.remove_vertices_by_mask(_skirt_mask(np.asarray(mesh.vertices)))
        # componentes: quedarse con el objeto (la mayor y las que pesen > 10 % de ella)
        clusters, counts, _ = mesh.cluster_connected_triangles()
        clusters, counts = np.asarray(clusters), np.asarray(counts)
        if len(counts):
            mesh.remove_triangles_by_mask(counts[clusters] < 0.10 * counts.max())
            mesh.remove_unreferenced_vertices()
        mesh = mesh.filter_smooth_taubin(number_of_iterations=4)
        if len(mesh.triangles) > target_triangles:
            mesh = mesh.simplify_quadric_decimation(target_triangles)
        mesh.remove_degenerate_triangles()
        mesh.remove_unreferenced_vertices()
        mesh.compute_vertex_normals()
        # pivote: centro de la huella del objeto en X/Z y apoyado en y = 0
        V = np.asarray(mesh.vertices)
        lo, hi = V.min(0), V.max(0)
        offset = np.array([-(lo[0] + hi[0]) / 2, -lo[1], -(lo[2] + hi[2]) / 2])
        mesh.translate(offset)
        return mesh, offset

    # ------------------------------------------------------------------ #
    def _save_view(self, v):
        folder = os.path.join(self.dir, f"vista_{v.idx + 1:02d}_{v.cam}")
        os.makedirs(folder, exist_ok=True)
        s = v.snap
        cv2.imwrite(os.path.join(folder, "color.jpg"), cv2.cvtColor(s["color"], cv2.COLOR_RGB2BGR),
                    [cv2.IMWRITE_JPEG_QUALITY, 92])
        cv2.imwrite(os.path.join(folder, "profundidad.png"), s["depth"])
        if v.mask is not None:
            cv2.imwrite(os.path.join(folder, "mascara.png"), v.mask.astype(np.uint8) * 255)
        with open(os.path.join(folder, "vista.json"), "w", encoding="utf-8") as f:
            json.dump({"intrinsics": s["intrinsics"], "depth_scale": s["depth_scale"], "toma": v.shot,
                       "camara": v.cam, "origen_pose": v.source, "T_cam_a_base": v.T.tolist(),
                       "base": v.base_info, "rafaga": int(s.get("burst", 1)),
                       "rafaga_cambio": s.get("burst_cambio"), "volumen": self.volume.public(),
                       "hoja": self.layout.name, "usar_base": self.use_base}, f, indent=2, ensure_ascii=False)

    def summary(self):
        return {"id": self.id, "nombre": self.name, "dir": self.rel_dir, "state": self.state,
                "tomas": len({v.shot for v in self.views}), "vistas": [v.summary() for v in self.views],
                "volumen": self.volume.public(), "usar_base": self.use_base, "hoja": self.layout.name,
                "escala_hoja": round(self.scale, 4), "progress": round(self.progress, 3), "stage": self.stage,
                "result": self.result, "error": self.error}

    def discard(self):
        import shutil
        with self._lock:
            if self.state == "processing":
                raise RuntimeError("No se puede descartar mientras se procesa.")
            if os.path.isdir(self.dir):
                shutil.rmtree(self.dir, ignore_errors=True)
