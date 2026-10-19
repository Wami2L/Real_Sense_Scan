"""
Fuentes de geometría para RealSense Live 3D.

* RealSenseSource: captura real, pensada para latencia mínima.
    - NO alinea color->profundidad en CPU (lo hace la GPU del cliente con las
      extrínsecas). Esto ahorra el paso más caro de librealsense.
    - Decimación + umbral de distancia + filtros opcionales, todo en C (librealsense).
    - Si no hay clientes conectados, no procesa ni codifica nada.
    - Robusta: perfiles de respaldo, watchdog de cuadros, hardware_reset tras
      fallos repetidos y espera a reconexión del USB.
* SyntheticSource: escena animada generada con numpy (sin cámara), útil para
  desarrollar el visor o el futuro cliente de Blender.

Ambas llaman a `on_frame(CapturedFrame)` desde su propio hilo.
"""

import json
import math
import os
import threading
import time
from dataclasses import dataclass, field, asdict

import numpy as np


# --------------------------------------------------------------------------- #
# Ajustes compartidos (thread-safe)
# --------------------------------------------------------------------------- #
PRESETS = {  # rs.rs400_visual_preset
    "custom": 0, "default": 1, "hand": 2,
    "high_accuracy": 3, "high_density": 4, "medium_density": 5,
}


@dataclass
class Settings:
    # Perfil de la cámara (cambiarlo reinicia el pipeline)
    depth_res: tuple = (848, 480)   # resolución óptima de profundidad en D4xx
    color_res: tuple = (848, 480)
    fps: int = 30
    # Ajustes en caliente
    decimation: int = 2              # 1..8  (2 => 424x240 ~ 100k puntos)
    near: float = 0.2                # metros
    far: float = 4.0
    spatial: bool = False
    temporal: bool = True
    hole_fill: bool = False
    emitter: bool = True
    preset: str = "default"
    jpeg_quality: int = 75
    version: int = field(default=0, compare=False)

    PROFILE_KEYS = ("depth_res", "color_res", "fps")

    def public(self):
        d = asdict(self)
        d["depth_res"] = list(self.depth_res)
        d["color_res"] = list(self.color_res)
        return d


class SettingsStore:
    def __init__(self, settings=None):
        self._lock = threading.Lock()
        self._s = settings or Settings()
        self.profile_version = 0

    def get(self):
        with self._lock:
            return Settings(**asdict(self._s))

    def update(self, changes):
        """Aplica cambios validados. Devuelve (settings, perfil_cambiado)."""
        with self._lock:
            s = self._s
            profile_changed = False
            for key, val in (changes or {}).items():
                if key == "decimation":
                    s.decimation = int(min(8, max(1, int(val))))
                elif key == "near":
                    s.near = float(min(max(0.0, float(val)), 20.0))
                elif key == "far":
                    s.far = float(min(max(0.05, float(val)), 20.0))
                elif key in ("spatial", "temporal", "hole_fill", "emitter"):
                    setattr(s, key, bool(val))
                elif key == "preset" and val in PRESETS:
                    s.preset = val
                elif key == "jpeg_quality":
                    s.jpeg_quality = int(min(95, max(20, int(val))))
                elif key in ("depth_res", "color_res"):
                    v = tuple(int(x) for x in val)
                    if v != getattr(s, key):
                        setattr(s, key, v)
                        profile_changed = True
                elif key == "fps":
                    v = int(val)
                    if v != s.fps:
                        s.fps = v
                        profile_changed = True
            if s.far <= s.near:
                s.far = s.near + 0.05
            s.version += 1
            if profile_changed:
                self.profile_version += 1
            return Settings(**asdict(s)), profile_changed


@dataclass
class CapturedFrame:
    seq: int
    timestamp: float
    depth: np.ndarray            # (H, W) uint16, ya decimada y recortada
    color: np.ndarray | None     # (h, w, 3) uint8 RGB, resolución nativa del color
    meta: dict                   # ver protocol.build_meta (incluye meta_id)
    settings: Settings


class NoDeviceError(RuntimeError):
    pass


def save_scan_to_disk(base_dir, color_np, depth_np, heat_np, intrinsics, depth_scale):
    import os
    import time
    import json
    import cv2
    import open3d as o3d

    date_folder = time.strftime("%Y-%m-%d")
    time_folder = time.strftime("scan_%H%M%S")
    scan_dir = os.path.join(base_dir, "Saves", date_folder, time_folder)
    os.makedirs(scan_dir, exist_ok=True)

    o3d_color = o3d.geometry.Image(color_np)
    o3d_heat = o3d.geometry.Image(heat_np)
    o3d_depth = o3d.geometry.Image(depth_np)

    pinhole = o3d.camera.PinholeCameraIntrinsic(
        int(intrinsics["width"]), int(intrinsics["height"]),
        float(intrinsics["fx"]), float(intrinsics["fy"]),
        float(intrinsics["ppx"]), float(intrinsics["ppy"])
    )

    # Nube RGB normal
    rgbd_normal = o3d.geometry.RGBDImage.create_from_color_and_depth(
        o3d_color, o3d_depth, depth_scale=1.0 / depth_scale, depth_trunc=3.0, convert_rgb_to_intensity=False
    )
    pcd_normal = o3d.geometry.PointCloud.create_from_rgbd_image(rgbd_normal, pinhole)

    # Nube Mapa de Calor
    rgbd_calor = o3d.geometry.RGBDImage.create_from_color_and_depth(
        o3d_heat, o3d_depth, depth_scale=1.0 / depth_scale, depth_trunc=3.0, convert_rgb_to_intensity=False
    )
    pcd_calor = o3d.geometry.PointCloud.create_from_rgbd_image(rgbd_calor, pinhole)

    # Guardar archivos idénticos a app.py
    o3d.io.write_point_cloud(os.path.join(scan_dir, "nube_puntos.ply"), pcd_normal)
    o3d.io.write_point_cloud(os.path.join(scan_dir, "nube_calor_profundidad.ply"), pcd_calor)

    cv2.imwrite(os.path.join(scan_dir, "captura_color.jpg"), cv2.cvtColor(color_np, cv2.COLOR_RGB2BGR))
    cv2.imwrite(os.path.join(scan_dir, "mapa_profundidad.jpg"), cv2.cvtColor(heat_np, cv2.COLOR_RGB2BGR))

    metadata = {
        "intrinsics": intrinsics,
        "depth_scale": depth_scale,
        "width": intrinsics["width"],
        "height": intrinsics["height"],
        "intrinsic_matrix": [
            [float(intrinsics["fx"]), 0.0, 0.0],
            [0.0, float(intrinsics["fy"]), 0.0],
            [float(intrinsics["ppx"]), float(intrinsics["ppy"]), 1.0]
        ],
        "filters": [
            "spatial",
            "temporal",
            "colorizer"
        ]
    }
    with open(os.path.join(scan_dir, "metadata_camara.json"), "w", encoding="utf-8") as f:
        json.dump(metadata, f, indent=4)

    rel_folder = f"Saves/{date_folder}/{time_folder}"
    return {
        "status": "success",
        "message": f"Escaneo dual guardado en {rel_folder}/",
        "folder": scan_dir,
        "rel_folder": rel_folder,
        "filename": time_folder
    }


# --------------------------------------------------------------------------- #
# Base
# --------------------------------------------------------------------------- #
class BaseSource:
    name = "base"

    def __init__(self, store, on_frame, has_demand=lambda: True, log=print):
        self.store = store
        self.on_frame = on_frame
        self._has_demand_fn = has_demand
        self.log = log
        self._stop = threading.Event()
        self._reload = threading.Event()
        self._thread = None
        self._lock = threading.Lock()
        self._capture_lock = threading.Lock()
        self._capture_req = None
        self._seq = 0
        self._meta_id = 0
        self._meta_key = None
        self._meta = None
        self._status = {"state": "starting", "message": "Inicializando...", "device": None,
                        "serial": None, "usb": None, "profile": None,
                        "fps": 0.0, "proc_ms": 0.0, "source": self.name}
        self._fps_n, self._fps_t0 = 0, time.time()
        self._proc_acc = 0.0

    def has_demand(self):
        return bool(self._has_demand_fn() or self._capture_req is not None)

    def start(self):
        self._thread = threading.Thread(target=self._run, name=f"{self.name}-source", daemon=True)
        self._thread.start()

    def stop(self, timeout=5.0):
        self._stop.set()
        self._reload.set()
        if self._thread:
            self._thread.join(timeout)

    def request_restart(self):
        self._reload.set()

    def get_snapshot(self, timeout=5.0):
        """Pide al hilo de captura un snapshot completo (color, depth, heat, intrinsics, depth_scale)."""
        evt = threading.Event()
        req = {"event": evt, "result": None, "error": None}
        with self._capture_lock:
            self._capture_req = req
        if not evt.wait(timeout):
            with self._capture_lock:
                if self._capture_req is req:
                    self._capture_req = None
            raise TimeoutError("Tiempo de espera agotado capturando cuadro de la cámara")
        if req["error"]:
            raise req["error"]
        return req["result"]

    def capture_scan(self, timeout=5.0):
        """Pide al hilo de captura procesar y guardar un escaneo completo en Saves/."""
        import os
        base_dir = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
        snap = self.get_snapshot(timeout=timeout)
        return save_scan_to_disk(base_dir, snap["color"], snap["depth"], snap["heat"], snap["intrinsics"], snap["depth_scale"])

    def status(self):
        with self._lock:
            st = dict(self._status)
        st["simulated"] = getattr(self, "simulated", False)
        return st

    def _set_status(self, **kw):
        with self._lock:
            self._status.update(kw)
            if kw.get("state") not in (None, "streaming"):
                self._status["fps"] = 0.0

    def _tick_fps(self, proc_s):
        self._fps_n += 1
        self._proc_acc += proc_s
        now = time.time()
        if now - self._fps_t0 >= 1.0:
            fps = self._fps_n / (now - self._fps_t0)
            proc = 1000.0 * self._proc_acc / max(1, self._fps_n)
            self._set_status(fps=round(fps, 1), proc_ms=round(proc, 2))
            self._fps_n, self._fps_t0, self._proc_acc = 0, now, 0.0

    def _get_meta(self, depth_intr, depth_scale, color_intr, rot, trans):
        """Reutiliza el meta mientras la geometría no cambie (nuevo meta_id si cambia)."""
        from .protocol import build_meta
        key = (tuple(round(v, 4) for v in depth_intr.values()), depth_scale,
               tuple(round(v, 4) for v in color_intr.values()) if color_intr else None,
               tuple(round(v, 6) for v in rot), tuple(round(v, 6) for v in trans))
        if key != self._meta_key:
            self._meta_id = (self._meta_id + 1) & 0xFFFF
            self._meta_key = key
            st = self.status()
            self._meta = build_meta(self._meta_id, depth_intr, depth_scale, color_intr, rot, trans,
                                    extra={"device": st["device"], "profile": st["profile"],
                                           "source": self.name})
        return self._meta

    def _emit(self, depth, color, meta, settings, t0):
        self._seq += 1
        frame = CapturedFrame(self._seq, time.time(), depth, color, meta, settings)
        self.on_frame(frame)
        self._tick_fps(time.perf_counter() - t0)

    def _run(self):
        raise NotImplementedError


# --------------------------------------------------------------------------- #
# RealSense
# --------------------------------------------------------------------------- #
class RealSenseSource(BaseSource):
    name = "realsense"
    simulated = False
    FALLBACK_PROFILES = [
        ((848, 480), (848, 480), 30),
        ((848, 480), (640, 480), 30),
        ((640, 480), (640, 480), 30),
        ((640, 480), (640, 480), 15),
        ((480, 270), (424, 240), 30),
    ]
    FIRST_FRAME_TIMEOUT_MS = 8000
    FRAME_TIMEOUT_MS = 2000
    WARMUP_FRAMES = 10
    STALL_SECONDS = 6.0
    HARD_RESET_AFTER_FAILURES = 2

    def __init__(self, *a, **kw):
        super().__init__(*a, **kw)
        import pyrealsense2 as rs  # import diferido: el modo sintético no lo necesita
        self.rs = rs
        self._fail = 0
        self._sensor = None

    # ---------------- ciclo de vida ----------------
    def _run(self):
        self.log("[Live3D] Hilo de captura RealSense iniciado")
        while not self._stop.is_set():
            self._reload.clear()
            if self._fail >= self.HARD_RESET_AFTER_FAILURES:
                self._hardware_reset()
                self._fail = 0
            try:
                pipeline, profile = self._open()
            except NoDeviceError:
                self._set_status(state="no_device", message="No se detecta ninguna RealSense (USB 3).")
                self._reload.wait(2.0)
                continue
            except Exception as e:  # noqa: BLE001
                self._fail += 1
                self.log(f"[Live3D] Error abriendo cámara (intento {self._fail}): {e}")
                self._set_status(state="error", message=f"No se pudo abrir la cámara: {e}")
                self._reload.wait(min(2.0 * self._fail, 10.0))
                continue
            try:
                self._stream(pipeline, profile)
            except Exception as e:  # noqa: BLE001
                self._fail += 1
                self.log(f"[Live3D] Streaming interrumpido: {e}")
                self._set_status(state="error", message=f"Señal perdida ({e}). Reconectando...")
            finally:
                try:
                    pipeline.stop()
                except Exception:  # noqa: BLE001
                    pass
                self._sensor = None
            if not self._stop.is_set():
                time.sleep(1.0)
        self._set_status(state="stopped", message="Captura detenida")
        self.log("[Live3D] Hilo de captura detenido")

    def _open(self):
        rs = self.rs
        ctx = rs.context()
        devs = ctx.query_devices()
        if len(devs) == 0:
            raise NoDeviceError()
        dev = devs[0]

        def info(k):
            try:
                return dev.get_info(k) if dev.supports(k) else None
            except Exception:  # noqa: BLE001
                return None

        name = info(rs.camera_info.name) or "RealSense"
        serial = info(rs.camera_info.serial_number)
        usb = info(rs.camera_info.usb_type_descriptor)
        self._set_status(state="starting", message=f"Abriendo {name}...", device=name,
                         serial=serial, usb=usb)

        s = self.store.get()
        candidates = [(tuple(s.depth_res), tuple(s.color_res), s.fps)]
        if usb and usb.startswith("2"):
            self.log(f"[Live3D] AVISO: USB {usb}; se priorizan perfiles ligeros")
            candidates += self.FALLBACK_PROFILES[2:]
        candidates += [p for p in self.FALLBACK_PROFILES if p not in candidates]

        pipeline = rs.pipeline(ctx)
        last_err = None
        for (dw, dh), (cw, ch), fps in candidates:
            cfg = rs.config()
            if serial:
                cfg.enable_device(serial)
            cfg.enable_stream(rs.stream.depth, dw, dh, rs.format.z16, fps)
            cfg.enable_stream(rs.stream.color, cw, ch, rs.format.rgb8, fps)
            try:
                if not cfg.can_resolve(rs.pipeline_wrapper(pipeline)):
                    continue
                profile = pipeline.start(cfg)
            except Exception as e:  # noqa: BLE001
                last_err = e
                time.sleep(0.5)
                continue
            try:
                self._sensor = profile.get_device().first_depth_sensor()
                self._apply_sensor_options(s, force=True)
                self._set_status(message=f"Calentando sensor de {name}...")
                pipeline.wait_for_frames(self.FIRST_FRAME_TIMEOUT_MS)
                for _ in range(self.WARMUP_FRAMES):
                    if self._stop.is_set() or self._reload.is_set():
                        break
                    pipeline.wait_for_frames(self.FRAME_TIMEOUT_MS)
            except Exception:
                try:
                    pipeline.stop()
                except Exception:  # noqa: BLE001
                    pass
                raise
            desc = f"D {dw}x{dh} · RGB {cw}x{ch} @ {fps} FPS"
            self._set_status(profile=desc)
            self._meta_key = None  # forzar meta nuevo
            self.log(f"[Live3D] {name} ({serial}) USB {usb}: {desc}")
            return pipeline, profile
        raise RuntimeError(f"ningún perfil compatible ({last_err})")

    def _apply_sensor_options(self, s, force=False, prev=None):
        rs, sensor = self.rs, self._sensor
        if sensor is None:
            return
        try:
            if (force or prev is None or prev.preset != s.preset) and sensor.supports(rs.option.visual_preset):
                sensor.set_option(rs.option.visual_preset, PRESETS[s.preset])
                force = True  # el preset puede tocar el emisor -> reaplicar
            if (force or prev is None or prev.emitter != s.emitter) and sensor.supports(rs.option.emitter_enabled):
                sensor.set_option(rs.option.emitter_enabled, 1 if s.emitter else 0)
        except Exception as e:  # noqa: BLE001
            self.log(f"[Live3D] No se pudo aplicar opción del sensor: {e}")

    def _build_filters(self, s):
        rs = self.rs
        f = {"decimation": None, "threshold": rs.threshold_filter(s.near, s.far)}
        if s.decimation > 1:
            f["decimation"] = rs.decimation_filter()
            f["decimation"].set_option(rs.option.filter_magnitude, s.decimation)
        if s.spatial or s.temporal:
            f["to_disp"] = rs.disparity_transform(True)
            f["to_depth"] = rs.disparity_transform(False)
            f["spatial"] = rs.spatial_filter() if s.spatial else None
            f["temporal"] = rs.temporal_filter() if s.temporal else None
        f["hole"] = rs.hole_filling_filter(1) if s.hole_fill else None
        return f

    def _stream(self, pipeline, profile):
        rs = self.rs
        depth_scale = float(self._sensor.get_depth_scale())
        dprof = profile.get_stream(rs.stream.depth).as_video_stream_profile()
        cprof = profile.get_stream(rs.stream.color).as_video_stream_profile()
        ex = dprof.get_extrinsics_to(cprof)
        r = list(ex.rotation)  # librealsense: column-major -> convertir a fila-mayor
        rot = [r[0], r[3], r[6], r[1], r[4], r[7], r[2], r[5], r[8]]
        trans = list(ex.translation)
        ci = cprof.get_intrinsics()
        color_intr = {"width": ci.width, "height": ci.height,
                      "fx": ci.fx, "fy": ci.fy, "ppx": ci.ppx, "ppy": ci.ppy}

        s = self.store.get()
        filters = self._build_filters(s)
        cap_align = rs.align(rs.stream.color)
        cap_colorizer = rs.colorizer()
        cap_spatial = rs.spatial_filter()
        cap_temporal = rs.temporal_filter()
        profile_version = self.store.profile_version
        last_ok = time.time()
        first = True

        while not self._stop.is_set() and not self._reload.is_set():
            try:
                frames = pipeline.wait_for_frames(self.FRAME_TIMEOUT_MS)
            except RuntimeError:
                if time.time() - last_ok > self.STALL_SECONDS:
                    raise RuntimeError("la cámara dejó de enviar cuadros")
                continue
            last_ok = time.time()

            if self._capture_req is not None:
                with self._capture_lock:
                    req = self._capture_req
                    self._capture_req = None
                if req is not None:
                    try:
                        aligned = cap_align.process(frames)
                        cdepth = aligned.get_depth_frame()
                        ccolor = aligned.get_color_frame()
                        if cdepth and ccolor:
                            c_ci = ccolor.profile.as_video_stream_profile().get_intrinsics()
                            c_intr = {
                                "width": c_ci.width, "height": c_ci.height,
                                "fx": c_ci.fx, "fy": c_ci.fy, "ppx": c_ci.ppx, "ppy": c_ci.ppy,
                                "model": str(c_ci.model), "coeffs": [float(x) for x in c_ci.coeffs]
                            }
                            f_cdepth = cap_temporal.process(cap_spatial.process(cdepth)).as_depth_frame()
                            heat_f = cap_colorizer.colorize(f_cdepth).as_video_frame()

                            c_color_np = _to_np(ccolor, c_ci.height, c_ci.width, 3, np.uint8)
                            c_depth_np = _to_np(f_cdepth, c_ci.height, c_ci.width, 1, np.uint16)
                            c_heat_np = _to_np(heat_f, c_ci.height, c_ci.width, 3, np.uint8)

                            req["result"] = {
                                "color": c_color_np,
                                "depth": c_depth_np,
                                "heat": c_heat_np,
                                "intrinsics": c_intr,
                                "depth_scale": depth_scale,
                                "timestamp": time.time()
                            }
                        else:
                            req["error"] = RuntimeError("No se pudieron alinear los cuadros de profundidad y color")
                    except Exception as ex:
                        req["error"] = ex
                    finally:
                        req["event"].set()

            if first:
                first = False
                self._fail = 0
                self._set_status(state="streaming", message=self.status()["device"] or "RealSense")
                self.log("[Live3D] Transmitiendo")

            if self.store.profile_version != profile_version:
                self.log("[Live3D] Perfil cambiado: reiniciando pipeline")
                return
            if not self.has_demand():
                self._tick_fps(0.0)  # mantener la cola vacía, sin procesar
                continue

            t0 = time.perf_counter()
            ns = self.store.get()
            if ns.version != s.version:
                self._apply_sensor_options(ns, prev=s)
                if (ns.decimation, ns.near, ns.far, ns.spatial, ns.temporal, ns.hole_fill) != \
                        (s.decimation, s.near, s.far, s.spatial, s.temporal, s.hole_fill):
                    filters = self._build_filters(ns)
                s = ns

            depth = frames.get_depth_frame()
            color = frames.get_color_frame()
            if not depth:
                continue
            f = depth
            if filters["decimation"] is not None:
                f = filters["decimation"].process(f)
            f = filters["threshold"].process(f)
            if "to_disp" in filters:
                f = filters["to_disp"].process(f)
                if filters["spatial"] is not None:
                    f = filters["spatial"].process(f)
                if filters["temporal"] is not None:
                    f = filters["temporal"].process(f)
                f = filters["to_depth"].process(f)
            if filters["hole"] is not None:
                f = filters["hole"].process(f)

            vf = f.as_video_frame()
            w, h = vf.get_width(), vf.get_height()
            di = vf.profile.as_video_stream_profile().get_intrinsics()
            depth_intr = {"width": w, "height": h, "fx": di.fx, "fy": di.fy, "ppx": di.ppx, "ppy": di.ppy}
            depth_np = _to_np(f, h, w, 1, np.uint16)
            color_np = _to_np(color, ci.height, ci.width, 3, np.uint8) if color else None

            meta = self._get_meta(depth_intr, depth_scale, color_intr, rot, trans)
            self._emit(depth_np, color_np, meta, s, t0)

    def _hardware_reset(self):
        rs = self.rs
        self._set_status(state="starting", message="Reiniciando hardware de la cámara (USB)...")
        self.log("[Live3D] hardware_reset()")
        try:
            for dev in rs.context().query_devices():
                dev.hardware_reset()
        except Exception as e:  # noqa: BLE001
            self.log(f"[Live3D] hardware_reset falló: {e}")
            return
        self._stop.wait(3.0)
        deadline = time.time() + 15.0
        while time.time() < deadline and not self._stop.is_set():
            try:
                if len(rs.context().query_devices()) > 0:
                    self._stop.wait(1.0)
                    return
            except Exception:  # noqa: BLE001
                pass
            self._stop.wait(0.5)


def _to_np(frame, h, w, channels, dtype):
    """Copia un frame a numpy de forma robusta (los filtros pueden perder metadatos)."""
    shape = (h, w, channels) if channels > 1 else (h, w)
    n = h * w * channels
    arr = np.asanyarray(frame.get_data())
    if arr.dtype == dtype and arr.size == n:
        return arr.reshape(shape).copy()
    raw = np.frombuffer(frame.get_data(), dtype=np.uint8)
    return raw.view(dtype)[:n].reshape(shape).copy()


# --------------------------------------------------------------------------- #
# Sintética (sin cámara)
# --------------------------------------------------------------------------- #
class SyntheticSource(BaseSource):
    """Escena trazada por rayos con numpy: pared ondulada, suelo y dos esferas
    en movimiento. Simula FOV de la D455, ruido y desplazamiento depth->color."""
    name = "synthetic"
    simulated = True
    DEPTH_HFOV = math.radians(87)
    COLOR_HFOV = math.radians(90)
    BASELINE = (-0.059, 0.0, 0.0)  # traslación depth -> color (m)

    def _run(self):
        self._set_status(state="streaming", message="Fuente sintética (sin cámara)",
                         device="Sintética", serial="SIM-0001", usb="-")
        rng = np.random.default_rng(0)
        cache = {}
        t_start = time.time()
        while not self._stop.is_set():
            s = self.store.get()
            period = 1.0 / max(1, s.fps)
            tick = time.perf_counter()
            if not self.has_demand():
                self._stop.wait(period)
                continue
            t0 = time.perf_counter()
            t = time.time() - t_start

            dw, dh = s.depth_res[0] // s.decimation, s.depth_res[1] // s.decimation
            cw, ch = s.color_res
            if cw > 424:  # el color se traza con numpy: limitar coste
                cw, ch = 424, int(round(ch * 424 / cw))
            key = (dw, dh, cw, ch)
            if key not in cache:
                cache.clear()
                cache[key] = (self._intr(dw, dh, self.DEPTH_HFOV), self._intr(cw, ch, self.COLOR_HFOV))
                self._set_status(profile=f"SIM D {dw}x{dh} · RGB {cw}x{ch} @ {s.fps} FPS")
            di, ci = cache[key]

            z, _ = self._trace(di, (0.0, 0.0, 0.0), t, want_color=False)
            z = z + rng.normal(0, 1, z.shape).astype(np.float32) * (0.0015 * z * z)
            z[(z < s.near) | (z > s.far)] = 0
            depth = np.clip(z * 1000.0, 0, 65535).astype(np.uint16)

            origin = tuple(-v for v in self.BASELINE)  # centro de la cámara de color
            _, color = self._trace(ci, origin, t, want_color=True)

            meta = self._get_meta(di, 0.001, ci, [1, 0, 0, 0, 1, 0, 0, 0, 1], list(self.BASELINE))
            self._emit(depth, color, meta, s, t0)

            if self._capture_req is not None:
                with self._capture_lock:
                    req = self._capture_req
                    self._capture_req = None
                if req is not None:
                    try:
                        import cv2
                        heat_u8 = cv2.applyColorMap(cv2.convertScaleAbs(depth, alpha=0.03), cv2.COLORMAP_JET)
                        heat_rgb = cv2.cvtColor(heat_u8, cv2.COLOR_BGR2RGB)
                        req["result"] = {
                            "color": color.copy(),
                            "depth": depth.copy(),
                            "heat": heat_rgb,
                            "intrinsics": dict(di),
                            "depth_scale": 0.001,
                            "timestamp": time.time()
                        }
                    except Exception as ex:
                        req["error"] = ex
                    finally:
                        req["event"].set()

            self._stop.wait(max(0.0, period - (time.perf_counter() - tick)))

    _NOISE = None

    @classmethod
    def _noise(cls, a, b):
        """Ruido suave fijo (semilla 0) muestreado en coordenadas del mundo (m), valores 0..1."""
        if cls._NOISE is None:
            import cv2
            rnd = np.random.default_rng(0).random((64, 64)).astype(np.float32)
            cls._NOISE = cv2.resize(rnd, (512, 512), interpolation=cv2.INTER_CUBIC).clip(0, 1)
        n = cls._NOISE
        i = (np.nan_to_num(a) * 96).astype(np.int64) % n.shape[1]
        j = (np.nan_to_num(b) * 96).astype(np.int64) % n.shape[0]
        return n[j, i]

    @staticmethod
    def _intr(w, h, hfov):
        f = (w / 2) / math.tan(hfov / 2)
        return {"width": w, "height": h, "fx": f, "fy": f, "ppx": (w - 1) / 2, "ppy": (h - 1) / 2}

    @staticmethod
    def _trace(intr, origin, t, want_color):
        w, h = intr["width"], intr["height"]
        gx = ((np.arange(w, dtype=np.float32) - intr["ppx"]) / intr["fx"])[None, :].repeat(h, 0)
        gy = ((np.arange(h, dtype=np.float32) - intr["ppy"]) / intr["fy"])[:, None].repeat(w, 1)
        ox, oy, oz = origin
        INF = np.float32(1e9)
        zbest = np.full((h, w), INF, np.float32)
        mat = np.zeros((h, w), np.int8)

        # Pared ondulada (z = 2.4 + onda). La onda se evalúa en coordenadas del MUNDO (donde el
        # rayo corta el plano nominal), así todas las cámaras simuladas ven la misma pared.
        xw, yw = ox + gx * (2.4 - oz), oy + gy * (2.4 - oz)
        zw = 2.4 + 0.06 * np.sin(xw * 6.0 + t * 1.5) * np.cos(yw * 5.0)
        zw = zw - oz
        zbest = np.minimum(zbest, zw)
        mat[:] = 1
        # Suelo en y = 0.55 (y hacia abajo en coords RealSense)
        with np.errstate(divide="ignore", invalid="ignore"):
            zf = np.where(gy > 1e-3, (0.55 - oy) / gy, INF).astype(np.float32)
        m = zf < zbest
        zbest[m], mat[m] = zf[m], 2
        # Esferas animadas
        spheres = [((0.35 * math.sin(t * 0.9), 0.05 * math.sin(t * 2.0), 1.3), 0.26, 3),
                   ((0.55 * math.cos(t * 1.4), -0.15, 1.3 + 0.45 * math.sin(t * 1.4)), 0.12, 4)]
        for (cx, cy, cz), rad, mid in spheres:
            cx, cy, cz = cx - ox, cy - oy, cz - oz
            a = gx * gx + gy * gy + 1.0
            b = -2.0 * (gx * cx + gy * cy + cz)
            c = cx * cx + cy * cy + cz * cz - rad * rad
            disc = b * b - 4 * a * c
            hit = disc > 0
            zs = np.where(hit, (-b - np.sqrt(np.maximum(disc, 0))) / (2 * a), INF).astype(np.float32)
            m = (zs > 0) & (zs < zbest)
            zbest[m], mat[m] = zs[m], mid

        zbest[zbest >= INF] = 0
        if not want_color:
            return zbest, None

        X, Y = gx * zbest + ox, gy * zbest + oy
        Z = zbest + oz
        img = np.zeros((h, w, 3), np.float32)
        # pared: damero cálido modulado por manchas no periódicas (pegadas al mundo, no a la
        # imagen) para que la fotogrametría tenga texturas únicas como en una escena real
        chk = ((np.floor(X * 4) + np.floor(Y * 4)) % 2).astype(np.float32)
        tex = SyntheticSource._noise(X, Y)[..., None]
        wall = np.stack([0.55 + 0.25 * chk, 0.42 + 0.15 * chk, 0.35 + 0.1 * chk], -1) * (0.55 + 0.7 * tex)
        img[mat == 1] = wall[mat == 1]
        # suelo: rejilla
        grid = ((np.abs((X * 5) % 1 - 0.5) > 0.46) | (np.abs((Z * 5) % 1 - 0.5) > 0.46)).astype(np.float32)
        floor = (np.stack([0.15 + 0.5 * grid, 0.2 + 0.6 * grid, 0.28 + 0.7 * grid], -1)
                 * (0.6 + 0.6 * SyntheticSource._noise(X + 7.0, Z)[..., None]))
        img[mat == 2] = floor[mat == 2]
        # esferas: sombreado lambert
        for (cx, cy, cz), rad, mid in spheres:
            m = mat == mid
            if not m.any():
                continue
            nx, ny, nz = (X[m] - cx) / rad, (Y[m] - cy) / rad, (Z[m] - cz) / rad
            lam = np.clip(-0.4 * nx - 0.7 * ny - 0.6 * nz, 0, 1)[:, None]
            base = np.array([0.95, 0.35, 0.25] if mid == 3 else [0.25, 0.75, 0.95], np.float32)
            img[m] = base * (0.25 + 0.75 * lam)
        return zbest, (np.clip(img, 0, 1) * 255).astype(np.uint8)


# --------------------------------------------------------------------------- #
# Calibración, modo de vista y estado por cámara compartidos por las fuentes duales
# --------------------------------------------------------------------------- #
ROOT_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
DUAL_CONFIG_PATH = os.path.join(ROOT_DIR, "config", "dual_camera_config.json")


class _DualMixin:
    """Extrínsecas T(cam2 -> cam1), modo de vista y estado por cámara.

    config_path=None: la calibración vive solo en memoria (simulación). Así una fuente
    simulada nunca sobrescribe la calibración real del disco.
    """
    simulated = False

    def _init_dual(self, config_path, serials, roles=("master", "slave")):
        self.config_path = config_path
        self._config_lock = threading.Lock()
        self.config = self._load_config()
        self.serials = list(serials)
        self.view_mode = "dual"       # 'dual': ambas cámaras; 'solo': solo la activa
        self._active_cam_idx = 0
        self._emit_lock = threading.Lock()  # _emit puede llamarse desde varios hilos
        self._cams = {i: {"idx": i, "serial": s, "role": roles[i] if i < len(roles) else "slave",
                          "state": "starting", "message": "Inicializando...", "fps": 0.0,
                          "last_frame": None, "frames": 0}
                      for i, s in enumerate(self.serials)}

    # ---- configuración / extrínsecas ----
    def _load_config(self):
        if self.config_path and os.path.isfile(self.config_path):
            try:
                with open(self.config_path, "r", encoding="utf-8") as f:
                    return json.load(f)
            except Exception as e:  # noqa: BLE001
                self.log(f"[Dual] Error leyendo {self.config_path}: {e}")
        return {"cameras": [], "extrinsics_cam2_to_cam1": {"is_calibrated": False,
                                                           "matrix_4x4": np.eye(4).tolist()}}

    def get_extrinsics(self):
        with self._config_lock:
            mat = self.config.get("extrinsics_cam2_to_cam1", {}).get("matrix_4x4")
        return np.eye(4, dtype=np.float64) if mat is None else np.array(mat, dtype=np.float64)

    def calibration_info(self):
        """Metadatos de la calibración vigente (sin la matriz)."""
        with self._config_lock:
            ext = dict(self.config.get("extrinsics_cam2_to_cam1", {}))
        ext.pop("matrix_4x4", None)
        ext["persisted"] = bool(self.config_path)
        return ext

    HISTORY_MAX = 20

    def set_extrinsics(self, matrix_4x4, **info):
        """Fija T(cam2 -> cam1) y la guarda de forma atómica. `info` (método, calidad, métricas)
        sustituye a los metadatos de la calibración anterior para que no queden mezclados.
        La calibración anterior pasa al principio de `extrinsics_history` (últimas HISTORY_MAX)."""
        T = np.asarray(matrix_4x4, dtype=np.float64)
        if T.shape != (4, 4) or not np.all(np.isfinite(T)):
            raise ValueError("matriz 4x4 inválida")
        R = T[:3, :3]
        if not np.allclose(R @ R.T, np.eye(3), atol=1e-3) or np.linalg.det(R) < 0.99:
            raise ValueError("la rotación no es ortonormal (det != 1)")
        entry = {"is_calibrated": True, "calibration_date": time.strftime("%Y-%m-%d %H:%M:%S"),
                 "matrix_4x4": T.tolist()}
        entry.update({k: v for k, v in info.items() if v is not None})
        with self._config_lock:
            old = self.config.get("extrinsics_cam2_to_cam1")
            if old and old.get("matrix_4x4") is not None and old.get("is_calibrated"):
                hist = self.config.setdefault("extrinsics_history", [])
                hist.insert(0, old)
                del hist[self.HISTORY_MAX:]
            self.config["extrinsics_cam2_to_cam1"] = entry
            if self.config_path:
                tmp = self.config_path + ".tmp"
                os.makedirs(os.path.dirname(self.config_path), exist_ok=True)
                with open(tmp, "w", encoding="utf-8") as f:
                    json.dump(self.config, f, indent=4, ensure_ascii=False)
                os.replace(tmp, self.config_path)
        return entry

    def calibration_history(self):
        """Calibraciones anteriores (la más reciente primero), con su matriz."""
        with self._config_lock:
            return [dict(e) for e in self.config.get("extrinsics_history", [])]

    def restore_extrinsics(self, index=0):
        """Vuelve a una calibración del historial (0 = la anterior: deshacer). La actual pasa al
        historial, así que deshacer dos veces vuelve a donde estabas: no se pierde nada."""
        hist = self.calibration_history()
        if not 0 <= index < len(hist):
            raise IndexError("no hay esa calibración en el historial")
        entry = dict(hist[index])
        with self._config_lock:  # quitarla del historial antes de que set_extrinsics meta la actual
            self.config["extrinsics_history"].pop(index)
        mat = entry.pop("matrix_4x4")
        for k in ("is_calibrated", "calibration_date", "persisted"):
            entry.pop(k, None)
        entry["restored_from"] = hist[index].get("calibration_date")
        return self.set_extrinsics(mat, **entry)

    # ---- modo de vista ----
    def set_active_camera(self, idx):
        self._active_cam_idx = 0 if idx == 0 else 1

    def set_view_mode(self, mode):
        self.view_mode = "dual" if mode == "dual" else "solo"

    def _emit(self, depth, color, meta, settings, t0):
        with self._emit_lock:
            BaseSource._emit(self, depth, color, meta, settings, t0)

    # ---- estado por cámara ----
    def _set_cam(self, idx, **kw):
        with self._lock:
            self._cams[idx].update(kw)
            if kw.get("state") not in (None, "streaming"):
                self._cams[idx]["fps"] = 0.0

    def _cam_frame(self, idx, fps=None):
        with self._lock:
            c = self._cams[idx]
            c["last_frame"] = time.time()
            c["frames"] += 1
            if fps is not None:
                c["fps"] = fps

    def cameras_status(self, stall_s=2.5):
        now = time.time()
        out = []
        with self._lock:
            cams = [dict(c) for c in self._cams.values()]
        for c in cams:
            last = c.pop("last_frame")
            c["age_s"] = None if last is None else round(now - last, 2)
            if c["state"] == "streaming" and c["age_s"] is not None and c["age_s"] > stall_s:
                c["state"], c["fps"] = "stalled", 0.0
                c["message"] = f"Sin cuadros nuevos desde hace {c['age_s']:.0f} s"
            c["simulated"] = self.simulated
            out.append(c)
        return out

    def status(self):
        st = BaseSource.status(self)
        cams = self.cameras_status()
        live = [c for c in cams if c["state"] == "streaming"]
        st["dual"] = True
        st["simulated"] = self.simulated
        st["cams"] = cams
        st["serial"] = " + ".join(c["serial"] for c in cams)
        if cams and len(live) == len(cams):
            st["state"] = "streaming"
            st["message"] = ("Simulación: 2 cámaras sintéticas" if self.simulated
                             else f"{len(cams)} cámaras en vivo")
        elif live:
            down = [c for c in cams if c["state"] != "streaming"]
            st["state"] = "degraded"
            st["message"] = "; ".join(f"Cam {c['idx'] + 1} ({c['serial'][-4:]}): {c['message']}" for c in down)
        elif cams and all(c["state"] == "no_device" for c in cams):
            st["state"], st["message"] = "no_device", "Ninguna de las 2 RealSense configuradas está conectada"
        elif any(c["state"] == "starting" for c in cams):
            st["state"], st["message"] = "starting", "Abriendo cámaras..."
        elif st.get("state") != "stopped":
            st["state"] = "error"
            st["message"] = "; ".join(f"Cam {c['idx'] + 1}: {c['message']}" for c in cams) or "Sin cámaras"
        st["fps"] = round(sum(c["fps"] for c in live) / len(live), 1) if live else 0.0
        return st


# --------------------------------------------------------------------------- #
# Sintética Dual (Simulación de 2 cámaras en vivo con extrínsecas)
# --------------------------------------------------------------------------- #
class SyntheticDualSource(_DualMixin, SyntheticSource):
    """Dos cámaras sintéticas que ven la MISMA escena; Cam 2 desplazada CAM2_OFFSET en x.
    Verdad de referencia: T(2->1) = traslación (+CAM2_OFFSET, 0, 0). Útil para tests."""
    name = "synthetic_dual"
    simulated = True
    CAM2_OFFSET = 0.25

    def __init__(self, store, on_frame, has_demand=lambda: True, log=print):
        super().__init__(store, on_frame, has_demand=has_demand, log=log)
        self._init_dual(None, ["SIM-CAM1-0777", "SIM-CAM2-1709"])
        T = np.eye(4)
        T[0, 3] = self.CAM2_OFFSET
        self.config["extrinsics_cam2_to_cam1"] = {"is_calibrated": True, "matrix_4x4": T.tolist(),
                                                   "method": "verdad de referencia (simulación)"}

    def _render(self, idx, di, ci, t, s, rng):
        off = self.CAM2_OFFSET if idx == 1 else 0.0
        z, _ = self._trace(di, (off, 0.0, 0.0), t, want_color=False)
        z = z + rng.normal(0, 1, z.shape).astype(np.float32) * (0.0015 * z * z)
        z[(z < s.near) | (z > s.far)] = 0
        depth = np.clip(z * 1000.0, 0, 65535).astype(np.uint16)
        origin_color = (off - self.BASELINE[0], -self.BASELINE[1], -self.BASELINE[2])
        _, color = self._trace(ci, origin_color, t, want_color=True)
        return depth, color

    def _snapshot(self, idx, di, depth, t):
        """Snapshot ALINEADO como el de la cámara real: color re-trazado desde el centro de
        profundidad con sus intrínsecas, de modo que color[v, u] corresponde a depth[v, u]."""
        import cv2
        off = self.CAM2_OFFSET if idx == 1 else 0.0
        _, color = self._trace(di, (off, 0.0, 0.0), t, want_color=True)
        heat = cv2.cvtColor(cv2.applyColorMap(cv2.convertScaleAbs(depth, alpha=0.03), cv2.COLORMAP_JET),
                            cv2.COLOR_BGR2RGB)
        return {"color": color, "depth": depth.copy(), "heat": heat, "intrinsics": dict(di),
                "depth_scale": 0.001, "timestamp": time.time(), "serial": self.serials[idx], "cam_idx": idx}

    def _run(self):
        self._set_status(state="streaming", message="Simulación: 2 cámaras sintéticas",
                         device="Dual RealSense D455 (Simulado)", serial=" + ".join(self.serials), usb="-", dual=True)
        for i in self._cams:
            self._set_cam(i, state="streaming", message="Simulada")
        rng = np.random.default_rng(0)
        cache = {}
        t_start = time.time()
        from .protocol import build_meta

        while not self._stop.is_set():
            s = self.store.get()
            period = 1.0 / max(1, s.fps)
            tick = time.perf_counter()
            if not self.has_demand():
                for i in self._cams:
                    self._cam_frame(i)  # la cámara simulada sigue "viva" aunque nadie mire
                self._stop.wait(period)
                continue
            t0 = time.perf_counter()
            t = time.time() - t_start

            dw, dh = s.depth_res[0] // s.decimation, s.depth_res[1] // s.decimation
            cw, ch = s.color_res
            if cw > 424:
                cw, ch = 424, int(round(ch * 424 / cw))
            key = (dw, dh, cw, ch)
            if key not in cache:
                cache.clear()
                cache[key] = (self._intr(dw, dh, self.DEPTH_HFOV), self._intr(cw, ch, self.COLOR_HFOV))
                self._set_status(profile=f"SIM D {dw}x{dh} · RGB {cw}x{ch} @ {s.fps} FPS")
            di, ci = cache[key]

            with self._capture_lock:
                req = self._capture_req
                self._capture_req = None
            depths = {}
            for idx in (0, 1):
                visible = self.view_mode == "dual" or self._active_cam_idx == idx
                if req is None and not visible:
                    continue
                depth, color = self._render(idx, di, ci, t, s, rng)
                depths[idx] = depth
                self._cam_frame(idx, fps=float(s.fps))
                if visible:
                    meta = build_meta(idx + 1, di, 0.001, ci, [1, 0, 0, 0, 1, 0, 0, 0, 1], list(self.BASELINE),
                                      extra={"device": f"RealSense D455 ({'MASTER' if idx == 0 else 'SLAVE'}, simulada)",
                                             "serial": self.serials[idx], "source": self.name, "cam_idx": idx})
                    self._emit(depth, color, meta, s, t0)

            if req is not None:
                try:
                    snap0, snap1 = (self._snapshot(i, di, depths[i], t) for i in (0, 1))
                    req["result"] = {"cam1": snap0, "cam2": snap1, "color": snap0["color"], "depth": snap0["depth"],
                                     "heat": snap0["heat"], "intrinsics": snap0["intrinsics"], "depth_scale": 0.001,
                                     "timestamp": snap0["timestamp"], "dt_ms": 0.0, "dt_source": "simulación",
                                     "extrinsics": self.get_extrinsics()}
                except Exception as ex:  # noqa: BLE001
                    req["error"] = ex
                finally:
                    req["event"].set()

            self._stop.wait(max(0.0, period - (time.perf_counter() - tick)))

    def capture_dual_scan(self, timeout=8.0):
        return DualRealSenseSource.capture_dual_scan(self, timeout=timeout)


# --------------------------------------------------------------------------- #
# Dual RealSense (Soporte y Captura Multi-Cámara Simultánea)
# --------------------------------------------------------------------------- #
class DualRealSenseSource(_DualMixin, BaseSource):
    """Dos D4xx sin cable de sincronía (inter_cam_sync_mode = 0), un hilo por cámara.

    Cada worker es independiente: si una cámara se desconecta, la otra sigue transmitiendo,
    el estado pasa a 'degraded' y el worker reintenta (con hardware_reset tras fallos
    repetidos) hasta que la cámara vuelve.
    """
    name = "dual_realsense"
    FIRST_FRAME_TIMEOUT_MS = 8000
    FRAME_TIMEOUT_MS = 2000
    STALL_SECONDS = 6.0
    HARD_RESET_AFTER_FAILURES = 2

    def __init__(self, store, on_frame, has_demand=lambda: True, log=print, config_path=None):
        super().__init__(store, on_frame, has_demand=has_demand, log=log)
        import pyrealsense2 as rs
        self.rs = rs
        self.config_path = config_path or DUAL_CONFIG_PATH
        self.config = self._load_config()
        # Seriales: los del JSON fijan cuál es Cam 1 y cuál Cam 2; si faltan, se completan con las conectadas
        serials = [c["serial"] for c in self.config.get("cameras", []) if c.get("serial")][:2]
        if len(serials) < 2:
            found = [d.get_info(rs.camera_info.serial_number) for d in rs.context().query_devices()]
            serials = (serials + [s for s in found if s not in serials])[:2]
        self._init_dual(self.config_path, serials)
        self.log("[DualSource] Cámaras configuradas: " +
                 ", ".join(f"Cam {i + 1}={s}" for i, s in enumerate(self.serials)))
        self._workers = []

    def capture_scan(self, timeout=8.0):
        """En modo dual, la 'Captura Simple' también usa el sensor virtual fusionado (ambas cámaras)."""
        return self.capture_dual_scan(timeout=timeout)

    def _cam_config(self, serial):
        return next((c for c in self.config.get("cameras", []) if c.get("serial") == serial), {})

    def _emitter_for(self, serial, is_master):
        c = self._cam_config(serial)
        return bool(c["emitter_enabled"]) if "emitter_enabled" in c else True

    def _run(self):
        if len(self.serials) < 2:
            self._set_status(state="error", message="Se requieren 2 cámaras RealSense (configura sus seriales)")
            return
        self.log("[DualSource] Iniciando workers concurrentes para ambas cámaras...")
        self._set_status(state="starting", message="Iniciando cámaras RealSense Dual...",
                         device="Dual RealSense D455", dual=True)
        # Slave primero, luego Master
        t_slave = threading.Thread(target=self._cam_worker, args=(1, self.serials[1], False),
                                   name="CamWorker-Slave", daemon=True)
        t_master = threading.Thread(target=self._cam_worker, args=(0, self.serials[0], True),
                                    name="CamWorker-Master", daemon=True)
        self._workers = [t_master, t_slave]
        t_slave.start()
        self._stop.wait(0.5)
        t_master.start()
        self._stop.wait()
        self.log("[DualSource] Deteniendo workers de cámara...")
        for w in self._workers:
            w.join(timeout=5.0)
        self._set_status(state="stopped", message="Captura detenida")
        self.log("[DualSource] Workers detenidos.")

    def _cam_worker(self, cam_idx, serial, is_master):
        fails = 0
        while not self._stop.is_set():
            if fails >= self.HARD_RESET_AFTER_FAILURES:
                self._reset_device(cam_idx, serial)
                fails = 0
            try:
                self._cam_session(cam_idx, serial, is_master)
            except NoDeviceError:
                self._set_cam(cam_idx, state="no_device", message="No conectada (revisa el cable USB 3)")
                self._stop.wait(2.0)
            except Exception as e:  # noqa: BLE001
                fails += 1
                self.log(f"[CamWorker {cam_idx}] ({serial}) Fallo #{fails}: {e}")
                self._set_cam(cam_idx, state="error", message=f"Señal perdida ({e}). Reintentando...")
                self._stop.wait(min(2.0 * fails, 10.0))
        self._set_cam(cam_idx, state="stopped", message="Detenida")
        self.log(f"[CamWorker {cam_idx}] ({serial}) Worker finalizado.")

    def _find_device(self, serial):
        rs = self.rs
        for d in rs.context().query_devices():
            if d.supports(rs.camera_info.serial_number) and d.get_info(rs.camera_info.serial_number) == serial:
                return d
        return None

    def _reset_device(self, cam_idx, serial):
        self._set_cam(cam_idx, state="starting", message="Reiniciando hardware (USB)...")
        dev = self._find_device(serial)
        if dev is None:
            return
        try:
            dev.hardware_reset()
        except Exception as e:  # noqa: BLE001
            self.log(f"[CamWorker {cam_idx}] hardware_reset falló: {e}")
            return
        self._stop.wait(5.0)  # Windows necesita unos segundos para re-enumerar el USB

    def _cam_session(self, cam_idx, serial, is_master):
        """Abre la cámara y transmite hasta que se pida parar (retorna) o falle (excepción)."""
        rs = self.rs
        role_str = "MASTER" if is_master else "SLAVE"
        dev = self._find_device(serial)
        if dev is None:
            raise NoDeviceError(serial)
        usb = dev.get_info(rs.camera_info.usb_type_descriptor) \
            if dev.supports(rs.camera_info.usb_type_descriptor) else None
        self._set_cam(cam_idx, state="starting", message=f"Abriendo ({role_str}, USB {usb or '?'})...", usb=usb)
        if usb and usb.startswith("2"):
            self.log(f"[CamWorker {cam_idx}] AVISO: {serial} por USB {usb}; 2 x 848x480@30 requiere USB 3")

        w, h, fps = (self._cam_config(serial).get("profile") or [848, 480, 30])[:3]
        pipeline = rs.pipeline(rs.context())
        cfg = rs.config()
        cfg.enable_device(serial)
        cfg.enable_stream(rs.stream.depth, int(w), int(h), rs.format.z16, int(fps))
        cfg.enable_stream(rs.stream.color, int(w), int(h), rs.format.rgb8, int(fps))
        align = rs.align(rs.stream.color)
        spatial = rs.spatial_filter()
        colorizer = rs.colorizer()

        profile = pipeline.start(cfg)
        try:
            sensor = profile.get_device().first_depth_sensor()
            emitter = self._emitter_for(serial, is_master)
            if sensor.supports(rs.option.emitter_enabled):
                try:
                    sensor.set_option(rs.option.emitter_enabled, 1 if emitter else 0)
                except Exception as ex:  # noqa: BLE001
                    self.log(f"[CamWorker {cam_idx}] Aviso al configurar emisor IR: {ex}")
            self._set_cam(cam_idx, emitter=emitter)
            depth_scale = float(sensor.get_depth_scale())

            from .protocol import build_meta
            cam_meta = None
            pipeline.wait_for_frames(self.FIRST_FRAME_TIMEOUT_MS)  # calentamiento
            self._set_cam(cam_idx, state="streaming", message=f"En vivo ({role_str})")
            self._set_status(profile=f"D+RGB {w}x{h} @ {fps} FPS por cámara")
            self.log(f"[CamWorker {cam_idx}] ({serial}) Transmitiendo {w}x{h}@{fps}.")

            last_ok = time.time()
            fps_n, fps_t0 = 0, last_ok
            while not self._stop.is_set():
                try:
                    frames = pipeline.wait_for_frames(self.FRAME_TIMEOUT_MS)
                except RuntimeError:
                    if time.time() - last_ok > self.STALL_SECONDS:
                        raise RuntimeError(f"sin cuadros durante {self.STALL_SECONDS:.0f} s")
                    continue
                arrival = time.perf_counter()  # resolución sub-ms (time.time() va a saltos de ~16 ms en Windows)
                last_ok = time.time()
                fps_n += 1
                fps_now = None
                if last_ok - fps_t0 >= 1.0:
                    fps_now = round(fps_n / (last_ok - fps_t0), 1)
                    fps_n, fps_t0 = 0, last_ok
                self._cam_frame(cam_idx, fps=fps_now)

                with self._capture_lock:
                    req = self._capture_req
                    wants_snap = req is not None and cam_idx not in req.setdefault("dual_snaps", {})
                visible = (self.view_mode == "dual" or cam_idx == self._active_cam_idx) and self.has_demand()
                if not wants_snap and not visible:
                    continue  # nadie necesita este cuadro: no alinear ni copiar (ahorra CPU)

                t0 = time.perf_counter()
                aligned = align.process(frames)
                depth_f = aligned.get_depth_frame()
                color_f = aligned.get_color_frame()
                if not depth_f or not color_f:
                    continue
                c_ci = color_f.profile.as_video_stream_profile().get_intrinsics()
                c_intr = {"width": c_ci.width, "height": c_ci.height,
                          "fx": c_ci.fx, "fy": c_ci.fy, "ppx": c_ci.ppx, "ppy": c_ci.ppy,
                          "model": str(c_ci.model), "coeffs": [float(x) for x in c_ci.coeffs]}

                if wants_snap:
                    self._store_snapshot(cam_idx, serial, frames, arrival, depth_f, color_f, c_intr, depth_scale,
                                         spatial, colorizer)
                if visible:
                    if cam_meta is None:  # profundidad alineada al color: mismas intrínsecas y E = I
                        cam_meta = build_meta(cam_idx + 1, c_intr, depth_scale, c_intr,
                                              [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0], [0.0, 0.0, 0.0],
                                              extra={"device": f"RealSense D455 ({role_str})", "serial": serial,
                                                     "source": self.name, "cam_idx": cam_idx})
                    s = self.store.get()
                    depth_np = _to_np(depth_f, c_ci.height, c_ci.width, 1, np.uint16)
                    color_np = _to_np(color_f, c_ci.height, c_ci.width, 3, np.uint8)
                    self._emit(depth_np, color_np, cam_meta, s, t0)
        finally:
            try:
                pipeline.stop()
            except Exception:  # noqa: BLE001
                pass

    def _store_snapshot(self, cam_idx, serial, frames, arrival, depth_f, color_f, c_intr, depth_scale,
                        spatial, colorizer):
        """Guarda el primer cuadro de esta cámara tras la petición; con las dos, responde.

        Δt entre cámaras: con los timestamps de hardware si ambas están en el dominio de tiempo
        global de librealsense (comparables entre cámaras); si no, con la hora de llegada al host
        (perf_counter, tomada justo después de wait_for_frames). Una cámara cae fuera del dominio
        global cuando no entrega metadatos de cuadro (en Windows: registro de metadatos UVC sin
        instalar para ese dispositivo).
        """
        rs = self.rs
        try:
            f_depth = spatial.process(depth_f).as_depth_frame()
            heat_f = colorizer.colorize(f_depth).as_video_frame()
            h, w = c_intr["height"], c_intr["width"]
            snap = {
                "color": _to_np(color_f, h, w, 3, np.uint8),
                "depth": _to_np(f_depth, h, w, 1, np.uint16),
                "heat": _to_np(heat_f, h, w, 3, np.uint8),
                "intrinsics": c_intr,
                "depth_scale": depth_scale,
                "timestamp": time.time(),
                "frame_timestamp_ms": float(frames.get_timestamp()),
                "arrival_s": arrival,
                "global_time": frames.get_frame_timestamp_domain() == rs.timestamp_domain.global_time,
                "serial": serial,
                "cam_idx": cam_idx,
            }
        except Exception as ex:  # noqa: BLE001
            self.log(f"[CamWorker {cam_idx}] Error procesando snapshot: {ex}")
            return
        with self._capture_lock:
            req = self._capture_req
            if req is None or cam_idx in req["dual_snaps"]:
                return
            req["dual_snaps"][cam_idx] = snap
            self._try_complete(req)

    def _try_complete(self, req):
        """Responde la petición de snapshot si ya están los cuadros de todas las cámaras EN VIVO
        (con una cámara caída, responde con la otra). Se llama con _capture_lock tomado."""
        snaps = req["dual_snaps"]
        live = {c["idx"] for c in self.cameras_status() if c["state"] == "streaming"} or {0, 1}
        if not live.issubset(snaps):
            return  # faltan cuadros de alguna cámara que está en vivo
        if len(snaps) < 2:
            # solo una cámara en vivo (la otra caída): snapshot de una sola cámara
            only = next(iter(snaps.values()))
            self._capture_req = None
            req["result"] = {f"cam{only['cam_idx'] + 1}": only, "color": only["color"], "depth": only["depth"],
                             "heat": only["heat"], "intrinsics": only["intrinsics"],
                             "depth_scale": only["depth_scale"], "timestamp": only["timestamp"],
                             "dt_ms": None, "extrinsics": self.get_extrinsics(), "degraded": True}
            req["event"].set()
            return
        s0, s1 = snaps[0], snaps[1]
        if s0["global_time"] and s1["global_time"]:
            dt_s, dt_source = (s1["frame_timestamp_ms"] - s0["frame_timestamp_ms"]) / 1000.0, "hardware (global_time)"
        else:
            dt_s, dt_source = s1["arrival_s"] - s0["arrival_s"], "llegada al host"
        s1["timestamp"] = s0["timestamp"] + dt_s
        self._capture_req = None
        req["result"] = {"cam1": s0, "cam2": s1, "color": s0["color"], "depth": s0["depth"],
                         "heat": s0["heat"], "intrinsics": s0["intrinsics"], "depth_scale": s0["depth_scale"],
                         "timestamp": s0["timestamp"], "dt_ms": round(abs(dt_s) * 1000.0, 1),
                         "dt_source": dt_source, "extrinsics": self.get_extrinsics()}
        req["event"].set()

    def capture_dual_scan(self, timeout=8.0):
        """Captura simultáneamente de ambas cámaras y fusiona las nubes en Saves/."""
        import cv2
        import open3d as o3d

        dual_snap = self.get_snapshot(timeout=timeout)
        if dual_snap.get("cam1") is None or dual_snap.get("cam2") is None:
            raise RuntimeError("La captura dual necesita las dos cámaras en vivo (ahora solo hay una)")
        snap1 = dual_snap["cam1"]
        snap2 = dual_snap["cam2"]
        T_2_to_1 = self.get_extrinsics()

        base_dir = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
        date_folder = time.strftime("%Y-%m-%d")
        time_folder = time.strftime("dual_scan_%H%M%S")
        scan_dir = os.path.join(base_dir, "Saves", date_folder, time_folder)
        os.makedirs(scan_dir, exist_ok=True)

        # 1. Crear nubes de puntos de cada cámara
        def snap_to_pcd(s):
            col = o3d.geometry.Image(s["color"])
            dep = o3d.geometry.Image(s["depth"])
            intr = s["intrinsics"]
            pinhole = o3d.camera.PinholeCameraIntrinsic(
                int(intr["width"]), int(intr["height"]),
                float(intr["fx"]), float(intr["fy"]),
                float(intr["ppx"]), float(intr["ppy"])
            )
            rgbd = o3d.geometry.RGBDImage.create_from_color_and_depth(
                col, dep, depth_scale=1.0 / s["depth_scale"], depth_trunc=4.0, convert_rgb_to_intensity=False
            )
            return o3d.geometry.PointCloud.create_from_rgbd_image(rgbd, pinhole)

        pcd1 = snap_to_pcd(snap1)
        pcd2 = snap_to_pcd(snap2)

        # 2. Transformar Cámara 2 al sistema de coordenadas de la Cámara 1
        pcd2_transformed = o3d.geometry.PointCloud(pcd2)
        pcd2_transformed.transform(T_2_to_1)

        # 3. Fusión y limpieza de nube de puntos
        pcd_fused = pcd1 + pcd2_transformed
        # Voxel grid downsample (2.5 mm) para deduplicar la zona de solape promediando baricentros y color
        pcd_fused = pcd_fused.voxel_down_sample(voxel_size=0.0025)
        # Eliminar ruido flotante estadístico
        pcd_clean, _ = pcd_fused.remove_statistical_outlier(nb_neighbors=25, std_ratio=1.2)
        pcd_clean.estimate_normals(o3d.geometry.KDTreeSearchParamHybrid(radius=0.01, max_nn=30))

        # 4. Integración TSDF y Reconstrucción de Superficie Continua
        has_mesh = False
        triangles_count = 0
        try:
            from .registration import integrate_dual_tsdf, create_overlap_diagnostic_pcd
            tsdf_mesh, _ = integrate_dual_tsdf(snap1, snap2, T_2_to_1, voxel_length=0.003, sdf_trunc=0.02)
            if len(tsdf_mesh.vertices) > 100:
                o3d.io.write_triangle_mesh(os.path.join(scan_dir, "malla_fusionada.obj"), tsdf_mesh)
                has_mesh = True
                triangles_count = len(tsdf_mesh.triangles)
            else:
                # Reconstrucción de superficie continua con Poisson si TSDF es disperso
                pcd_clean.estimate_normals(o3d.geometry.KDTreeSearchParamHybrid(radius=0.02, max_nn=30))
                pcd_clean.orient_normals_towards_camera_location(np.zeros(3))
                poisson_mesh, densities = o3d.geometry.TriangleMesh.create_from_point_cloud_poisson(pcd_clean, depth=8)
                if len(poisson_mesh.vertices) > 0:
                    densities_np = np.asarray(densities)
                    poisson_mesh.remove_vertices_by_mask(densities_np < np.quantile(densities_np, 0.05))
                    poisson_mesh.compute_vertex_normals()
                    o3d.io.write_triangle_mesh(os.path.join(scan_dir, "malla_fusionada.obj"), poisson_mesh)
                    has_mesh = True
                    triangles_count = len(poisson_mesh.triangles)

            diag_pcd = create_overlap_diagnostic_pcd(pcd1, pcd2, T_2_to_1)
            o3d.io.write_point_cloud(os.path.join(scan_dir, "diagnostico_solape.ply"), diag_pcd)
        except Exception as e:
            self.log(f"[DualScan] Advertencia en reconstrucción de malla/diagnóstico: {e}")

        # 5. Guardar archivos de nubes y texturas en disco
        o3d.io.write_point_cloud(os.path.join(scan_dir, "nube_fusionada.ply"), pcd_clean)
        o3d.io.write_point_cloud(os.path.join(scan_dir, "nube_cam1.ply"), pcd1)
        o3d.io.write_point_cloud(os.path.join(scan_dir, "nube_cam2.ply"), pcd2)

        cv2.imwrite(os.path.join(scan_dir, "cam1_color.jpg"), cv2.cvtColor(snap1["color"], cv2.COLOR_RGB2BGR))
        cv2.imwrite(os.path.join(scan_dir, "cam2_color.jpg"), cv2.cvtColor(snap2["color"], cv2.COLOR_RGB2BGR))
        cv2.imwrite(os.path.join(scan_dir, "cam1_mapa_profundidad.jpg"), cv2.cvtColor(snap1["heat"], cv2.COLOR_RGB2BGR))
        cv2.imwrite(os.path.join(scan_dir, "cam2_mapa_profundidad.jpg"), cv2.cvtColor(snap2["heat"], cv2.COLOR_RGB2BGR))

        # Metadatos completos con referencias y calidad
        metadata = {
            "type": "dual_scan",
            "date": date_folder,
            "id": time_folder,
            "cameras": {
                "cam1": {"serial": self.serials[0], "role": "master", "intrinsics": snap1["intrinsics"]},
                "cam2": {"serial": self.serials[1], "role": "slave", "intrinsics": snap2["intrinsics"]}
            },
            "extrinsics_cam2_to_cam1": T_2_to_1.tolist(),
            "calibration": self.calibration_info(),
            "dt_ms": dual_snap.get("dt_ms"),
            "dt_source": dual_snap.get("dt_source"),
            "simulated": self.simulated,
            "points_cam1": len(pcd1.points),
            "points_cam2": len(pcd2.points),
            "points_fused": len(pcd_clean.points),
            "has_mesh": has_mesh,
            "mesh_triangles": triangles_count,
            "mesh_file": "malla_fusionada.obj" if has_mesh else None,
            "overlap_diagnostic_file": "diagnostico_solape.ply"
        }

        with open(os.path.join(scan_dir, "metadata_dual.json"), "w", encoding="utf-8") as f:
            json.dump(metadata, f, indent=4)

        rel_folder = f"Saves/{date_folder}/{time_folder}"
        mesh_info = f", malla 3D: {triangles_count:,} triángulos" if has_mesh else ""
        return {
            "status": "success",
            "message": f"Escaneo Dual 360° guardado en {rel_folder}/ ({len(pcd_clean.points):,} puntos{mesh_info})",
            "folder": scan_dir,
            "rel_folder": rel_folder,
            "filename": time_folder,
            "points_fused": len(pcd_clean.points),
            "has_mesh": has_mesh
        }

