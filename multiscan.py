"""
Multi-captura: fusiona varias capturas de la RealSense en un único mapeo 3D.

Flujo de una sesión:
  1. MultiScanSession(...)  -> crea  Saves/<fecha>/multi_<hora>/
  2. add_view(snapshot)     -> guarda la vista, crea su nube de puntos y la alinea
                               con la vista anterior (ICP coloreado multi-escala,
                               con RANSAC+FPFH como respaldo). Devuelve la calidad
                               del solapamiento para guiar al usuario.
  3. finish()               -> (en segundo plano)
        a. Grafo de poses con odometría + cierres de lazo entre vistas no consecutivas
        b. Optimización global (Levenberg-Marquardt) del grafo
        c. Fusión de nubes  -> nube_fusionada.ply / nube_fusionada_calor.ply
        d. Integración TSDF -> malla_fusionada.ply / .obj  (Poisson como respaldo)

Convención de transformaciones (igual que los tutoriales de Open3D):
  T(i->j) lleva puntos del sistema de la vista i al sistema de la vista j.
  pose[i] lleva puntos de la vista i al sistema global (= sistema de la vista 0).
"""

import base64
import json
import os
import shutil
import threading
import time
import traceback

import cv2
import numpy as np
import open3d as o3d

reg = o3d.pipelines.registration

MIN_DEPTH_M = 0.15
GOOD_FITNESS = 0.6       # si el ICP directo supera esto, no se prueba RANSAC
LOOP_MIN_FITNESS = 0.3   # mínimo para aceptar un cierre de lazo


# ---------------------------------------------------------------------- #
# Utilidades geométricas
# ---------------------------------------------------------------------- #
def make_intrinsic(intr):
    return o3d.camera.PinholeCameraIntrinsic(
        int(intr["width"]), int(intr["height"]),
        float(intr["fx"]), float(intr["fy"]), float(intr["ppx"]), float(intr["ppy"]),
    )


def build_rgbd(color, depth, depth_scale, max_depth):
    depth = depth.copy()
    depth[depth.astype(np.float32) * depth_scale < MIN_DEPTH_M] = 0
    return o3d.geometry.RGBDImage.create_from_color_and_depth(
        o3d.geometry.Image(np.ascontiguousarray(color)),
        o3d.geometry.Image(np.ascontiguousarray(depth)),
        depth_scale=1.0 / depth_scale,
        depth_trunc=float(max_depth),
        convert_rgb_to_intensity=False,
    )


def adaptive_voxel(pcd):
    """Tamaño de vóxel según el tamaño de la escena (robusto a puntos sueltos)."""
    pts = np.asarray(pcd.points)
    lo = np.percentile(pts, 2, axis=0)
    hi = np.percentile(pts, 98, axis=0)
    diag = float(np.linalg.norm(hi - lo))
    return float(np.clip(diag / 150.0, 0.004, 0.04))


def _hybrid(radius, nn):
    return o3d.geometry.KDTreeSearchParamHybrid(radius=radius, max_nn=nn)


def _quality(fitness):
    if fitness is None:
        return "referencia"
    if fitness >= 0.5:
        return "excelente"
    if fitness >= 0.3:
        return "buena"
    if fitness >= 0.15:
        return "baja"
    return "insuficiente"


def _thumb_b64(rgb, width=220):
    h, w = rgb.shape[:2]
    small = cv2.resize(rgb, (width, int(h * width / w)), interpolation=cv2.INTER_AREA)
    ok, buf = cv2.imencode(".jpg", cv2.cvtColor(small, cv2.COLOR_RGB2BGR), [cv2.IMWRITE_JPEG_QUALITY, 72])
    return "data:image/jpeg;base64," + base64.b64encode(buf.tobytes()).decode() if ok else None


# ---------------------------------------------------------------------- #
# Vista individual
# ---------------------------------------------------------------------- #
class ScanView:
    def __init__(self, index, snap, max_depth):
        self.index = index
        self.color = snap["color"]
        self.depth = snap["depth"]
        self.heat = snap["heat"]
        self.intr = dict(snap["intrinsics"])
        self.depth_scale = float(snap["depth_scale"])
        self.timestamp = snap["timestamp"]
        self.max_depth = max_depth
        self.link = None   # registro respecto a la vista anterior
        self.thumb = _thumb_b64(self.color)
        self.folder = None

        rgbd = build_rgbd(self.color, self.depth, self.depth_scale, max_depth)
        self.raw_pcd = o3d.geometry.PointCloud.create_from_rgbd_image(rgbd, make_intrinsic(self.intr))

        # En modo dual: fusionar Cámara 2 transformada para que cada toma actúe como un solo sensor virtual
        if "cam2" in snap and "extrinsics" in snap and snap["extrinsics"] is not None:
            s2 = snap["cam2"]
            rgbd2 = build_rgbd(s2["color"], s2["depth"], float(s2["depth_scale"]), max_depth)
            pcd2 = o3d.geometry.PointCloud.create_from_rgbd_image(rgbd2, make_intrinsic(dict(s2["intrinsics"])))
            pcd2.transform(np.asarray(snap["extrinsics"], dtype=np.float64))
            self.raw_pcd = (self.raw_pcd + pcd2).voxel_down_sample(0.0025)

        if len(self.raw_pcd.points) < 2000:
            raise ValueError(
                "La vista casi no tiene puntos de profundidad válidos. "
                "Acércate (mín. ~0.5 m) o aumenta la distancia máxima."
            )

    def preprocess(self, voxel):
        origin = np.zeros(3)
        self.pcd = self.raw_pcd.voxel_down_sample(voxel * 0.5)
        self.pcd.estimate_normals(_hybrid(voxel * 2, 30))
        self.pcd.orient_normals_towards_camera_location(origin)

        self.pyramid = {}
        for s in (4, 2, 1):
            p = self.raw_pcd.voxel_down_sample(voxel * s)
            p.estimate_normals(_hybrid(voxel * s * 2, 30))
            p.orient_normals_towards_camera_location(origin)
            self.pyramid[s] = p

        self.fpfh = reg.compute_fpfh_feature(self.pyramid[2], _hybrid(voxel * 2 * 5, 100))
        self.raw_pcd = None  # liberar memoria

    def summary(self, with_thumb=False):
        d = {
            "index": self.index,
            "points": len(self.pcd.points) if hasattr(self, "pcd") else 0,
            "fitness": None if self.link is None else round(self.link["fitness"], 3),
            "rmse": None if self.link is None else round(self.link["rmse"], 5),
            "quality": _quality(None if self.link is None else self.link["fitness"]),
        }
        if with_thumb:
            d["thumb"] = self.thumb
        return d


# ---------------------------------------------------------------------- #
# Registro por pares
# ---------------------------------------------------------------------- #
def _icp_refine(src, tgt, init, voxel):
    T = np.asarray(init, dtype=np.float64)
    for scale, iters in ((4, 60), (2, 40), (1, 30)):
        s, t = src.pyramid[scale], tgt.pyramid[scale]
        dist = voxel * scale * 1.5
        crit = reg.ICPConvergenceCriteria(relative_fitness=1e-6, relative_rmse=1e-6, max_iteration=iters)
        try:
            res = reg.registration_colored_icp(s, t, dist, T, reg.TransformationEstimationForColoredICP(), crit)
        except RuntimeError:
            res = reg.registration_icp(s, t, dist, T, reg.TransformationEstimationPointToPlane(), crit)
        if res.fitness > 0:
            T = res.transformation
    ev = reg.evaluate_registration(src.pyramid[1], tgt.pyramid[1], voxel * 1.5, T)
    return np.asarray(T), float(ev.fitness), float(ev.inlier_rmse)


def _global_ransac(src, tgt, voxel):
    dist = voxel * 2 * 1.5
    res = reg.registration_ransac_based_on_feature_matching(
        src.pyramid[2], tgt.pyramid[2], src.fpfh, tgt.fpfh, True, dist,
        reg.TransformationEstimationPointToPoint(False), 3,
        [
            reg.CorrespondenceCheckerBasedOnEdgeLength(0.9),
            reg.CorrespondenceCheckerBasedOnDistance(dist),
        ],
        reg.RANSACConvergenceCriteria(100000, 0.999),
    )
    return np.asarray(res.transformation)


def register_pair(src, tgt, voxel, init=None, log=print):
    """Devuelve (T src->tgt, matriz de información, fitness, rmse)."""
    candidates = []
    try:
        candidates.append(_icp_refine(src, tgt, np.eye(4) if init is None else init, voxel))
    except Exception as e:  # noqa: BLE001
        log(f"[Multi] ICP directo falló: {e}")

    if not candidates or candidates[0][1] < GOOD_FITNESS:
        try:
            T0 = _global_ransac(src, tgt, voxel)
            candidates.append(_icp_refine(src, tgt, T0, voxel))
        except Exception as e:  # noqa: BLE001
            log(f"[Multi] RANSAC falló: {e}")

    if not candidates:
        return np.eye(4), np.eye(6), 0.0, 0.0

    T, fit, rmse = max(candidates, key=lambda c: (round(c[1], 3), -c[2]))
    info = reg.get_information_matrix_from_point_clouds(src.pyramid[1], tgt.pyramid[1], voxel * 1.5, T)
    return T, np.asarray(info), fit, rmse


# ---------------------------------------------------------------------- #
# Sesión
# ---------------------------------------------------------------------- #
class MultiScanSession:
    MIN_VIEWS = 2
    MAX_VIEWS = 40

    def __init__(self, base_dir, max_depth=1.5, log=print):
        date = time.strftime("%Y-%m-%d")
        self.id = time.strftime("multi_%H%M%S")
        self.rel_dir = f"Saves/{date}/{self.id}"
        self.dir = os.path.join(base_dir, "Saves", date, self.id)
        os.makedirs(self.dir, exist_ok=True)

        self.max_depth = float(np.clip(max_depth, 0.3, 6.0))
        self.voxel = None
        self.views = []
        self.state = "capturing"   # capturing | processing | done
        self.progress = 0.0
        self.stage = ""
        self.result = None
        self.error = None
        self._op_lock = threading.Lock()
        self._log = log

    # ---------------- Captura ----------------
    def add_view(self, snap):
        with self._op_lock:
            if self.state != "capturing":
                raise RuntimeError("La sesión no está en modo captura.")
            if len(self.views) >= self.MAX_VIEWS:
                raise RuntimeError(f"Máximo de {self.MAX_VIEWS} vistas por sesión.")

            self.error = None
            view = ScanView(len(self.views), snap, self.max_depth)
            if self.voxel is None:
                self.voxel = adaptive_voxel(view.raw_pcd)
                self._log(f"[Multi] Vóxel de registro: {self.voxel * 1000:.1f} mm")
            view.preprocess(self.voxel)

            if self.views:
                T, info, fit, rmse = register_pair(self.views[-1], view, self.voxel, log=self._log)
                view.link = {"T": T, "info": info, "fitness": fit, "rmse": rmse}

            self._save_view(view)
            self.views.append(view)

            s = view.summary(with_thumb=True)
            s["hint"] = self._hint(view)
            return s

    def undo(self):
        with self._op_lock:
            if self.state != "capturing" or not self.views:
                raise RuntimeError("No hay vistas que deshacer.")
            view = self.views.pop()
            if view.folder and os.path.isdir(view.folder):
                shutil.rmtree(view.folder, ignore_errors=True)
            if not self.views:
                self.voxel = None
            return view.index

    def discard(self):
        with self._op_lock:
            if self.state == "processing":
                raise RuntimeError("No se puede descartar mientras se procesa.")
            if self.result is None and os.path.isdir(self.dir):
                shutil.rmtree(self.dir, ignore_errors=True)
                parent = os.path.dirname(self.dir)
                try:
                    if os.path.isdir(parent) and not os.listdir(parent):
                        os.rmdir(parent)
                except OSError:
                    pass

    def finish(self):
        with self._op_lock:
            if self.state != "capturing":
                raise RuntimeError("La sesión ya se está procesando o terminó.")
            if len(self.views) < self.MIN_VIEWS:
                raise RuntimeError(f"Necesitas al menos {self.MIN_VIEWS} vistas para fusionar.")
            self.state = "processing"
            self.error = None
            self.progress = 0.0
            self.stage = "Preparando..."
        threading.Thread(target=self._process, name="MultiScanFusion", daemon=True).start()

    def summary(self, with_thumbs=False):
        return {
            "id": self.id,
            "dir": self.rel_dir,
            "state": self.state,
            "max_depth": self.max_depth,
            "voxel_mm": None if self.voxel is None else round(self.voxel * 1000, 1),
            "views": [v.summary(with_thumb=with_thumbs) for v in self.views],
            "count": len(self.views),
            "progress": round(self.progress, 3),
            "stage": self.stage,
            "result": self.result,
            "error": self.error,
        }

    # ---------------- Internos ----------------
    @staticmethod
    def _hint(view):
        if view.link is None:
            return "Vista de referencia guardada. Gira la cámara (o el objeto) ~20-30° y añade la siguiente."
        q = _quality(view.link["fitness"])
        pct = round(view.link["fitness"] * 100)
        if q == "excelente":
            return f"Alineación excelente ({pct}% de solapamiento). Puedes moverte un poco más."
        if q == "buena":
            return f"Alineación buena ({pct}%). Sigue con giros similares."
        if q == "baja":
            return f"Solapamiento bajo ({pct}%). Te moviste demasiado: reduce el giro entre capturas."
        return f"No se pudo alinear ({pct}%). Pulsa Deshacer y captura más cerca de la vista anterior."

    def _save_view(self, view):
        folder = os.path.join(self.dir, f"vista_{view.index + 1:02d}")
        os.makedirs(folder, exist_ok=True)
        view.folder = folder
        cv2.imwrite(os.path.join(folder, "captura_color.jpg"), cv2.cvtColor(view.color, cv2.COLOR_RGB2BGR))
        cv2.imwrite(os.path.join(folder, "mapa_profundidad.jpg"), cv2.cvtColor(view.heat, cv2.COLOR_RGB2BGR))
        cv2.imwrite(os.path.join(folder, "profundidad_raw.png"), view.depth)  # uint16, reprocesable
        o3d.io.write_point_cloud(os.path.join(folder, "nube_puntos.ply"), view.pcd)
        meta = {
            "intrinsics": view.intr,
            "depth_scale": view.depth_scale,
            "max_depth": view.max_depth,
            "timestamp": view.timestamp,
            "registro_con_anterior": None if view.link is None else {
                "fitness": view.link["fitness"],
                "rmse": view.link["rmse"],
                "transformacion": np.asarray(view.link["T"]).tolist(),
            },
        }
        with open(os.path.join(folder, "metadata_vista.json"), "w", encoding="utf-8") as f:
            json.dump(meta, f, indent=4)

    def _set_stage(self, text, progress):
        self.stage = text
        self.progress = float(progress)
        self._log(f"[Multi] {int(progress * 100):3d}% {text}")

    @staticmethod
    def _loop_pairs(n):
        if n <= 6:
            return [(i, j) for i in range(n) for j in range(i + 2, n)]
        pairs = {(i, j) for i in range(n) for j in (i + 2, i + 3) if j < n}
        pairs.add((0, n - 1))  # cierre de vuelta completa alrededor del objeto
        pairs.add((0, n - 2))
        pairs.add((1, n - 1))
        return sorted(pairs)

    def _process(self):
        t0 = time.time()
        try:
            views, voxel, n = self.views, self.voxel, len(self.views)

            # a) Grafo de poses ---------------------------------------------
            self._set_stage("Construyendo grafo de poses", 0.03)
            graph = reg.PoseGraph()
            odometry = np.eye(4)
            graph.nodes.append(reg.PoseGraphNode(np.eye(4)))
            for i in range(1, n):
                link = views[i].link
                odometry = link["T"] @ odometry
                graph.nodes.append(reg.PoseGraphNode(np.linalg.inv(odometry)))
                graph.edges.append(reg.PoseGraphEdge(i - 1, i, link["T"], link["info"], uncertain=False))

            pairs = self._loop_pairs(n)
            loops = 0
            for k, (i, j) in enumerate(pairs):
                self._set_stage(f"Buscando cierres de lazo ({k + 1}/{len(pairs)})", 0.05 + 0.40 * k / max(1, len(pairs)))
                init = np.linalg.inv(np.asarray(graph.nodes[j].pose)) @ np.asarray(graph.nodes[i].pose)
                T, info, fit, _ = register_pair(views[i], views[j], voxel, init=init, log=self._log)
                if fit >= LOOP_MIN_FITNESS:
                    graph.edges.append(reg.PoseGraphEdge(i, j, T, info, uncertain=True))
                    loops += 1

            # b) Optimización global ---------------------------------------
            self._set_stage("Optimizando poses globalmente", 0.47)
            option = reg.GlobalOptimizationOption(
                max_correspondence_distance=voxel * 1.5,
                edge_prune_threshold=0.25,
                preference_loop_closure=1.0,
                reference_node=0,
            )
            with o3d.utility.VerbosityContextManager(o3d.utility.VerbosityLevel.Error):
                reg.global_optimization(
                    graph,
                    reg.GlobalOptimizationLevenbergMarquardt(),
                    reg.GlobalOptimizationConvergenceCriteria(),
                    option,
                )
            poses = [np.asarray(node.pose) for node in graph.nodes]

            # c) Fusión de nubes -------------------------------------------
            self._set_stage("Fusionando nubes de puntos", 0.55)
            merged = o3d.geometry.PointCloud()
            for v, pose in zip(views, poses):
                p = o3d.geometry.PointCloud(v.pcd)
                p.transform(pose)
                merged += p
            merged = merged.voxel_down_sample(voxel * 0.5)
            if len(merged.points) > 100:
                merged, _ = merged.remove_statistical_outlier(nb_neighbors=20, std_ratio=2.0)
            o3d.io.write_point_cloud(os.path.join(self.dir, "nube_fusionada.ply"), merged)

            # Versión "mapa de calor": color según distancia a la primera vista
            heat = o3d.geometry.PointCloud(merged)
            pts = np.asarray(heat.points)
            if len(pts):
                dist = np.linalg.norm(pts - poses[0][:3, 3], axis=1)
                lo, hi = np.percentile(dist, 2), np.percentile(dist, 98)
                norm = np.clip((dist - lo) / max(hi - lo, 1e-6), 0, 1)
                jet = cv2.applyColorMap((norm * 255).astype(np.uint8).reshape(-1, 1), cv2.COLORMAP_JET)
                heat.colors = o3d.utility.Vector3dVector(jet.reshape(-1, 3)[:, ::-1].astype(np.float64) / 255.0)
            o3d.io.write_point_cloud(os.path.join(self.dir, "nube_fusionada_calor.ply"), heat)

            # d) Integración TSDF ------------------------------------------
            tsdf_voxel = float(np.clip(voxel / 2.5, 0.002, 0.02))
            volume = o3d.pipelines.integration.ScalableTSDFVolume(
                voxel_length=tsdf_voxel,
                sdf_trunc=tsdf_voxel * 5,
                color_type=o3d.pipelines.integration.TSDFVolumeColorType.RGB8,
            )
            for k, (v, pose) in enumerate(zip(views, poses)):
                self._set_stage(f"Integrando volumen TSDF ({k + 1}/{n})", 0.62 + 0.23 * k / n)
                rgbd = build_rgbd(v.color, v.depth, v.depth_scale, self.max_depth)
                volume.integrate(rgbd, make_intrinsic(v.intr), np.linalg.inv(pose))

            self._set_stage("Extrayendo y limpiando malla", 0.87)
            mesh = volume.extract_triangle_mesh()
            method = "TSDF"
            if len(mesh.triangles) == 0:
                self._set_stage("TSDF vacío, usando Poisson", 0.88)
                mesh = self._poisson(merged)
                method = "Poisson"
            mesh = self._clean_mesh(mesh)
            mesh.compute_vertex_normals()

            self._set_stage("Guardando resultados", 0.95)
            o3d.io.write_triangle_mesh(os.path.join(self.dir, "malla_fusionada.ply"), mesh)
            try:
                o3d.io.write_triangle_mesh(os.path.join(self.dir, "malla_fusionada.obj"), mesh)
            except Exception as e:  # noqa: BLE001
                self._log(f"[Multi] No se pudo escribir OBJ: {e}")

            fitness = [v.link["fitness"] for v in views[1:]]
            resumen = {
                "vistas": n,
                "cierres_de_lazo": loops,
                "voxel_registro_mm": round(voxel * 1000, 2),
                "voxel_tsdf_mm": round(tsdf_voxel * 1000, 2),
                "distancia_maxima_m": self.max_depth,
                "fitness_por_vista": [round(f, 4) for f in fitness],
                "fitness_medio": round(float(np.mean(fitness)), 4) if fitness else None,
                "puntos_fusionados": len(merged.points),
                "vertices_malla": len(mesh.vertices),
                "triangulos_malla": len(mesh.triangles),
                "metodo_malla": method,
                "tiempo_proceso_s": round(time.time() - t0, 1),
                "poses": [p.tolist() for p in poses],
            }
            with open(os.path.join(self.dir, "resumen_multicaptura.json"), "w", encoding="utf-8") as f:
                json.dump(resumen, f, indent=4)

            self.result = {
                "dir": self.rel_dir,
                "views": n,
                "loops": loops,
                "points": len(merged.points),
                "triangles": len(mesh.triangles),
                "method": method,
                "mean_fitness": resumen["fitness_medio"],
                "seconds": resumen["tiempo_proceso_s"],
                "files": [
                    "malla_fusionada.ply", "malla_fusionada.obj",
                    "nube_fusionada.ply", "nube_fusionada_calor.ply",
                    "resumen_multicaptura.json",
                ],
            }
            self._set_stage("Completado", 1.0)
            self.state = "done"
        except Exception as e:  # noqa: BLE001
            traceback.print_exc()
            self.error = f"Error al generar el mapeo: {e}"
            self.stage = ""
            self.progress = 0.0
            self.state = "capturing"  # permite añadir más vistas o reintentar

    @staticmethod
    def _poisson(pcd):
        pcd = o3d.geometry.PointCloud(pcd)
        if not pcd.has_normals():
            pcd.estimate_normals()
        mesh, dens = o3d.geometry.TriangleMesh.create_from_point_cloud_poisson(pcd, depth=9)
        dens = np.asarray(dens)
        mesh.remove_vertices_by_mask(dens < np.quantile(dens, 0.05))
        return mesh.crop(pcd.get_axis_aligned_bounding_box())

    @staticmethod
    def _clean_mesh(mesh):
        mesh.remove_degenerate_triangles()
        mesh.remove_duplicated_triangles()
        mesh.remove_duplicated_vertices()
        mesh.remove_non_manifold_edges()
        if len(mesh.triangles) > 0:
            clusters, counts, _ = mesh.cluster_connected_triangles()
            clusters, counts = np.asarray(clusters), np.asarray(counts)
            if len(counts):
                small = counts[clusters] < max(50, 0.01 * counts.max())
                mesh.remove_triangles_by_mask(small)
        mesh.remove_unreferenced_vertices()
        return mesh
