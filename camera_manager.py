"""
Gestor robusto de la cámara Intel RealSense.

Principios de diseño (para que abrir / recargar la cámara nunca rompa el servidor):

1. Un ÚNICO hilo ("CameraWorker") es dueño del pipeline. Ningún otro hilo llama a
   start()/stop()/wait_for_frames(), así se eliminan las carreras que antes
   provocaban RuntimeError y bucles infinitos al recargar.
2. Los cuadros se convierten a arrays de NumPy (copias) dentro del worker y se
   liberan inmediatamente. Así no se agota el pool interno de frames de librealsense
   y los filtros (spatial/temporal/colorizer) solo se usan desde un hilo.
3. Apertura tolerante a fallos:
     - Espera a que haya un dispositivo conectado (estado "no_device").
     - Prueba varios perfiles de resolución (fallback si el USB no da ancho de banda).
     - Calentamiento (auto-exposición) antes de declarar la cámara lista.
     - Tras 2 fallos seguidos hace hardware_reset() del dispositivo y espera
       a que se vuelva a enumerar.
4. Watchdog: si dejan de llegar cuadros durante STALL_SECONDS, el pipeline se
   reinicia solo.
5. Los streams MJPEG nunca terminan: mientras la cámara no está lista se envía
   una imagen de estado ("Conectando cámara..."), y cuando vuelve, el vídeo
   continúa sin recargar la página.
"""

import threading
import time
import unicodedata

import cv2
import numpy as np
import pyrealsense2 as rs


class NoDeviceError(RuntimeError):
    """No hay ninguna cámara RealSense conectada."""


class CameraManager:
    # (ancho, alto, fps) en orden de preferencia
    PROFILES = [(1280, 720, 30), (848, 480, 30), (640, 480, 30), (640, 480, 15)]
    WARMUP_FRAMES = 20
    FIRST_FRAME_TIMEOUT_MS = 8000
    FRAME_TIMEOUT_MS = 2000
    STALL_SECONDS = 6.0
    HARD_RESET_AFTER_FAILURES = 2
    REENUMERATE_TIMEOUT_S = 15.0

    PREVIEW_FPS = 15
    PREVIEW_MAX_WIDTH = 960
    JPEG_QUALITY = 80

    def __init__(self, log=print):
        self._log = log
        self._cond = threading.Condition()
        self._reload_evt = threading.Event()
        self._stop_evt = threading.Event()
        self._hard_reset_requested = False
        self._thread = None
        self._fail_count = 0
        self._generation = 0  # se incrementa cada vez que el streaming arranca

        # Estado público (solo lectura desde fuera)
        self.state = "starting"  # starting | streaming | no_device | error | stopped
        self.message = "Inicializando cámara..."
        self.device_name = None
        self.serial = None
        self.usb_type = None
        self.profile_desc = None
        self.fps = 0.0

        self._snapshot = None
        self._frame_seq = 0
        self._jpeg = {"color": None, "depth": None}
        self._jpeg_seq = 0
        self._placeholder_cache = {}

    # ------------------------------------------------------------------ #
    # API pública
    # ------------------------------------------------------------------ #
    def start(self):
        with self._cond:
            if self._thread is not None and self._thread.is_alive():
                return
            self._stop_evt.clear()
            self._thread = threading.Thread(target=self._run, name="CameraWorker", daemon=True)
            self._thread.start()

    def shutdown(self, timeout=5.0):
        self._stop_evt.set()
        self._reload_evt.set()
        if self._thread is not None:
            self._thread.join(timeout)
        self._set_state("stopped", "Cámara detenida")

    def request_reload(self, hard=False):
        """Pide al worker reiniciar el pipeline. Devuelve la 'generación' actual
        para poder esperar a que arranque una nueva con wait_for_stream()."""
        with self._cond:
            if hard:
                self._hard_reset_requested = True
            gen = self._generation
            self._reload_evt.set()
            self._set_state_locked("starting", "Reiniciando cámara...")
        return gen

    def wait_for_stream(self, after_generation=-1, timeout=20.0):
        """Espera a que la cámara esté transmitiendo (en una generación posterior
        a `after_generation`). Devuelve True/False sin lanzar excepciones."""
        with self._cond:
            return self._cond.wait_for(
                lambda: self.state == "streaming"
                and self._generation > after_generation
                and self._snapshot is not None,
                timeout,
            )

    def is_streaming(self):
        return self.state == "streaming"

    def get_snapshot(self, max_age=1.5):
        """Último cuadro procesado (dict de arrays NumPy) o None si no está listo."""
        with self._cond:
            snap = self._snapshot
            state = self.state
        if state != "streaming" or snap is None:
            return None
        if time.time() - snap["timestamp"] > max_age:
            return None
        return snap

    def wait_fresh_snapshot(self, timeout=3.0):
        """Espera al SIGUIENTE cuadro (garantiza que la captura es actual)."""
        with self._cond:
            seq = self._frame_seq
            ok = self._cond.wait_for(
                lambda: self.state == "streaming" and self._frame_seq > seq, timeout
            )
            return self._snapshot if ok else None

    def wait_jpeg(self, kind, last_seq, timeout=0.5):
        """Bloquea hasta que haya un JPEG nuevo (o timeout). No consume CPU en espera."""
        with self._cond:
            self._cond.wait_for(
                lambda: self._jpeg_seq != last_seq or self.state != "streaming", timeout
            )
            return self._jpeg.get(kind), self._jpeg_seq, self.state

    def status(self):
        with self._cond:
            return {
                "state": self.state,
                "message": self.message,
                "device": self.device_name,
                "serial": self.serial,
                "usb": self.usb_type,
                "profile": self.profile_desc,
                "fps": round(self.fps, 1),
                "streaming": self.state == "streaming",
            }

    # ------------------------------------------------------------------ #
    # Worker
    # ------------------------------------------------------------------ #
    def _run(self):
        self._log("[Cámara] Worker iniciado")
        while not self._stop_evt.is_set():
            self._reload_evt.clear()

            with self._cond:
                hard = self._hard_reset_requested or self._fail_count >= self.HARD_RESET_AFTER_FAILURES
                self._hard_reset_requested = False
            if hard:
                self._hardware_reset()
                self._fail_count = 0
                if self._stop_evt.is_set():
                    break

            # ---------- Apertura ----------
            try:
                pipeline, depth_scale = self._open_pipeline()
            except NoDeviceError:
                self._set_state(
                    "no_device",
                    "No se detecta ninguna cámara RealSense. Conéctala a un puerto USB 3.",
                )
                self._fail_count = 0
                self._reload_evt.wait(2.0)
                continue
            except Exception as e:  # noqa: BLE001
                self._fail_count += 1
                self._log(f"[Cámara] Error abriendo (intento {self._fail_count}): {e}")
                self._set_state(
                    "error",
                    f"No se pudo abrir la cámara (intento {self._fail_count}). Reintentando...",
                )
                self._reload_evt.wait(min(1.5 * self._fail_count, 10.0))
                continue

            # ---------- Streaming ----------
            try:
                self._stream(pipeline, depth_scale)
            except Exception as e:  # noqa: BLE001
                self._fail_count += 1
                self._log(f"[Cámara] Streaming interrumpido: {e}")
                self._set_state("error", f"Se perdió la señal de la cámara ({e}). Reconectando...")
            finally:
                self._safe_stop(pipeline)
                with self._cond:
                    self._snapshot = None
                    self._jpeg = {"color": None, "depth": None}
                    self._cond.notify_all()

            if not self._stop_evt.is_set():
                # Dar tiempo al driver USB para liberar el dispositivo
                time.sleep(1.0)

        self._log("[Cámara] Worker detenido")

    def _open_pipeline(self):
        ctx = rs.context()
        devices = ctx.query_devices()
        if len(devices) == 0:
            raise NoDeviceError()

        dev = devices[0]
        name = self._dev_info(dev, rs.camera_info.name) or "RealSense"
        serial = self._dev_info(dev, rs.camera_info.serial_number)
        usb = self._dev_info(dev, rs.camera_info.usb_type_descriptor)

        self._set_state("starting", f"Abriendo {name}...")
        if usb and usb.startswith("2"):
            self._log(f"[Cámara] AVISO: conectada por USB {usb}; se usará menor resolución.")

        pipeline = rs.pipeline(ctx)
        profile = None
        chosen = None
        last_err = None

        for w, h, fps in self.PROFILES:
            cfg = rs.config()
            if serial:
                cfg.enable_device(serial)
            cfg.enable_stream(rs.stream.depth, w, h, rs.format.z16, fps)
            cfg.enable_stream(rs.stream.color, w, h, rs.format.rgb8, fps)
            try:
                if not cfg.can_resolve(rs.pipeline_wrapper(pipeline)):
                    continue
                profile = pipeline.start(cfg)
                chosen = (w, h, fps)
                break
            except Exception as e:  # noqa: BLE001
                last_err = e
                self._safe_stop(pipeline)
                time.sleep(0.5)

        if profile is None:
            raise RuntimeError(f"ningún perfil de resolución compatible ({last_err})")

        try:
            depth_sensor = profile.get_device().first_depth_sensor()
            if depth_sensor.supports(rs.option.emitter_enabled):
                depth_sensor.set_option(rs.option.emitter_enabled, 1)
            depth_scale = float(depth_sensor.get_depth_scale())

            # Calentamiento: el primer cuadro puede tardar; luego dejamos
            # estabilizar la auto-exposición.
            self._set_state("starting", f"Calentando sensor de {name}...")
            pipeline.wait_for_frames(self.FIRST_FRAME_TIMEOUT_MS)
            for _ in range(self.WARMUP_FRAMES):
                if self._stop_evt.is_set() or self._reload_evt.is_set():
                    break
                pipeline.wait_for_frames(self.FRAME_TIMEOUT_MS)
        except Exception:
            self._safe_stop(pipeline)
            raise

        with self._cond:
            self.device_name = name
            self.serial = serial
            self.usb_type = usb
            self.profile_desc = f"{chosen[0]}x{chosen[1]} @ {chosen[2]} FPS"
        self._log(f"[Cámara] {name} ({serial}) abierta en {self.profile_desc}, USB {usb}")
        return pipeline, depth_scale

    def _stream(self, pipeline, depth_scale):
        align = rs.align(rs.stream.color)
        spatial = rs.spatial_filter()
        temporal = rs.temporal_filter()
        colorizer = rs.colorizer()
        colorizer.set_option(rs.option.color_scheme, 0)  # Jet

        intr = None
        last_ok = time.time()
        last_preview = 0.0
        last_err_log = 0.0
        fps_count, fps_t0 = 0, time.time()
        first = True

        with self._cond:
            self._generation += 1

        while not self._stop_evt.is_set() and not self._reload_evt.is_set():
            try:
                frames = pipeline.wait_for_frames(self.FRAME_TIMEOUT_MS)
            except RuntimeError:
                if time.time() - last_ok > self.STALL_SECONDS:
                    raise RuntimeError("la cámara dejó de enviar cuadros")
                continue

            try:
                aligned = align.process(frames)
                depth = aligned.get_depth_frame()
                color = aligned.get_color_frame()
                if not depth or not color:
                    continue

                if intr is None:
                    i = color.profile.as_video_stream_profile().intrinsics
                    intr = {
                        "width": i.width, "height": i.height,
                        "fx": i.fx, "fy": i.fy, "ppx": i.ppx, "ppy": i.ppy,
                    }
                h, w = intr["height"], intr["width"]

                depth_f = temporal.process(spatial.process(depth)).as_depth_frame()
                heat_f = colorizer.colorize(depth_f).as_video_frame()

                color_np = self._frame_to_np(color, h, w, 3, np.uint8)
                depth_np = self._frame_to_np(depth_f, h, w, 1, np.uint16)
                heat_np = self._frame_to_np(heat_f, h, w, 3, np.uint8)
            except Exception as e:  # noqa: BLE001
                now = time.time()
                if now - last_err_log > 5:
                    self._log(f"[Cámara] Error procesando cuadro: {e}")
                    last_err_log = now
                if now - last_ok > self.STALL_SECONDS:
                    raise
                continue

            now = time.time()
            last_ok = now

            fps_count += 1
            if now - fps_t0 >= 1.0:
                self.fps = fps_count / (now - fps_t0)
                fps_count, fps_t0 = 0, now

            preview = None
            if now - last_preview >= 1.0 / self.PREVIEW_FPS:
                last_preview = now
                preview = (self._encode_jpeg(color_np), self._encode_jpeg(heat_np))

            with self._cond:
                if self._reload_evt.is_set() or self._stop_evt.is_set():
                    break
                self._snapshot = {
                    "color": color_np,
                    "depth": depth_np,
                    "heat": heat_np,
                    "intrinsics": intr,
                    "depth_scale": depth_scale,
                    "timestamp": now,
                }
                self._frame_seq += 1
                if preview is not None and preview[0] is not None and preview[1] is not None:
                    self._jpeg = {"color": preview[0], "depth": preview[1]}
                    self._jpeg_seq += 1
                if self.state != "streaming":
                    self.state = "streaming"
                    self.message = f"{self.device_name} · {self.profile_desc}"
                self._cond.notify_all()

            if first:
                first = False
                self._fail_count = 0
                self._log("[Cámara] Transmitiendo")

    # ------------------------------------------------------------------ #
    # Utilidades
    # ------------------------------------------------------------------ #
    def _hardware_reset(self):
        self._set_state("starting", "Reiniciando hardware de la cámara (USB)...")
        self._log("[Cámara] hardware_reset()")
        try:
            ctx = rs.context()
            for dev in ctx.query_devices():
                try:
                    dev.hardware_reset()
                except Exception as e:  # noqa: BLE001
                    self._log(f"[Cámara] hardware_reset falló: {e}")
        except Exception as e:  # noqa: BLE001
            self._log(f"[Cámara] No se pudo consultar dispositivos: {e}")
            return

        # El dispositivo desaparece y vuelve a enumerarse en el bus USB.
        self._stop_evt.wait(3.0)
        deadline = time.time() + self.REENUMERATE_TIMEOUT_S
        while time.time() < deadline and not self._stop_evt.is_set():
            try:
                if len(rs.context().query_devices()) > 0:
                    self._stop_evt.wait(1.0)
                    return
            except Exception:  # noqa: BLE001
                pass
            self._stop_evt.wait(0.5)

    @staticmethod
    def _safe_stop(pipeline):
        if pipeline is None:
            return
        try:
            pipeline.stop()
        except Exception:  # noqa: BLE001
            pass

    @staticmethod
    def _dev_info(dev, key):
        try:
            return dev.get_info(key) if dev.supports(key) else None
        except Exception:  # noqa: BLE001
            return None

    @staticmethod
    def _frame_to_np(frame, h, w, channels, dtype):
        """Convierte un frame a un array (H, W[, C]) propio. Los filtros de
        librealsense a veces pierden metadatos y get_data() devuelve un buffer
        plano, así que se reinterpreta de forma explícita."""
        shape = (h, w, channels) if channels > 1 else (h, w)
        n = h * w * channels
        arr = np.asanyarray(frame.get_data())
        if arr.dtype == dtype and arr.size == n:
            return arr.reshape(shape).copy()
        raw = np.frombuffer(frame.get_data(), dtype=np.uint8)
        return raw.view(dtype)[:n].reshape(shape).copy()

    def _encode_jpeg(self, rgb):
        try:
            bgr = cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR)
            h, w = bgr.shape[:2]
            if w > self.PREVIEW_MAX_WIDTH:
                nh = int(h * self.PREVIEW_MAX_WIDTH / w)
                bgr = cv2.resize(bgr, (self.PREVIEW_MAX_WIDTH, nh), interpolation=cv2.INTER_AREA)
            ok, buf = cv2.imencode(".jpg", bgr, [cv2.IMWRITE_JPEG_QUALITY, self.JPEG_QUALITY])
            return buf.tobytes() if ok else None
        except Exception:  # noqa: BLE001
            return None

    def _set_state(self, state, message):
        with self._cond:
            self._set_state_locked(state, message)

    def _set_state_locked(self, state, message):
        self.state = state
        self.message = message
        if state != "streaming":
            self.fps = 0.0
        self._cond.notify_all()

    # ------------------------------------------------------------------ #
    # Imagen de estado para los streams cuando no hay vídeo
    # ------------------------------------------------------------------ #
    def placeholder_jpeg(self, kind):
        state, message = self.state, self.message
        dots = int(time.time() * 2) % 4
        key = (kind, state, message, dots)
        cached = self._placeholder_cache.get(key)
        if cached is not None:
            return cached

        def ascii_text(s):
            return unicodedata.normalize("NFKD", s or "").encode("ascii", "ignore").decode()

        w, h = 960, 540
        img = np.zeros((h, w, 3), np.uint8)
        # Degradado oscuro (BGR)
        top = np.array([42, 23, 15], np.float32)
        bottom = np.array([20, 10, 5], np.float32)
        t = np.linspace(0, 1, h, dtype=np.float32)[:, None]
        img[:] = (top * (1 - t) + bottom * t)[:, None, :].astype(np.uint8)

        accent = {
            "starting": (11, 158, 245),
            "error": (68, 68, 239),
            "no_device": (184, 163, 148),
            "stopped": (184, 163, 148),
        }.get(state, (246, 130, 59))
        titles = {
            "starting": "CONECTANDO CAMARA",
            "error": "RECONECTANDO",
            "no_device": "CAMARA NO DETECTADA",
            "stopped": "CAMARA DETENIDA",
        }
        title = titles.get(state, "ESPERANDO VIDEO") + "." * dots
        subtitle = "RGB" if kind == "color" else "MAPA DE CALOR"

        cx, cy = w // 2, h // 2 - 50
        cv2.circle(img, (cx, cy), 46, accent, 3, cv2.LINE_AA)
        start_angle = (time.time() * 180) % 360
        cv2.ellipse(img, (cx, cy), (46, 46), 0, start_angle, start_angle + 90, (255, 255, 255), 3, cv2.LINE_AA)

        font = cv2.FONT_HERSHEY_DUPLEX
        (tw, _), _ = cv2.getTextSize(title, font, 0.9, 2)
        cv2.putText(img, title, (cx - tw // 2, cy + 100), font, 0.9, (248, 250, 252), 2, cv2.LINE_AA)
        (sw, _), _ = cv2.getTextSize(subtitle, font, 0.55, 1)
        cv2.putText(img, subtitle, (cx - sw // 2, cy + 130), font, 0.55, accent, 1, cv2.LINE_AA)

        msg = ascii_text(message)
        lines, line = [], ""
        for word in msg.split():
            if len(line) + len(word) + 1 > 70:
                lines.append(line)
                line = word
            else:
                line = f"{line} {word}".strip()
        if line:
            lines.append(line)
        for i, ln in enumerate(lines[:3]):
            (lw, _), _ = cv2.getTextSize(ln, cv2.FONT_HERSHEY_SIMPLEX, 0.55, 1)
            cv2.putText(img, ln, (cx - lw // 2, cy + 170 + i * 26),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.55, (184, 163, 148), 1, cv2.LINE_AA)

        ok, buf = cv2.imencode(".jpg", img, [cv2.IMWRITE_JPEG_QUALITY, 75])
        data = buf.tobytes() if ok else b""
        if len(self._placeholder_cache) > 64:
            self._placeholder_cache.clear()
        self._placeholder_cache[key] = data
        return data
