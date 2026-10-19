"""
Benchmark de línea base del servidor Live 3D (Fase 0, plan 10).

Se suscribe a /ws igual que el navegador (zlib + JPEG + ACK), mide durante N
segundos lo que recibe por cámara y lo que reporta el servidor en `status`, y
opcionalmente cronometra una captura dual.

    python tools/bench_live3d.py                         # contra un servidor ya en marcha (5002)
    python tools/bench_live3d.py --spawn                 # lanza su propio servidor sintético dual
    python tools/bench_live3d.py --seconds 20 --capture --json bench_antes.json

La latencia (time.time() - timestamp del cuadro) solo es válida si cliente y
servidor corren en la misma máquina.
"""

import argparse
import asyncio
import json
import os
import socket
import statistics
import subprocess
import sys
import time

import aiohttp

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if ROOT not in sys.path:
    sys.path.insert(0, ROOT)

from live3d import protocol  # noqa: E402


def free_port():
    with socket.socket() as s:
        s.bind(("127.0.0.1", 0))
        return s.getsockname()[1]


def spawn_server(port, extra_args=(), log=None):
    """Lanza `python -m live3d` en un subproceso. El llamador debe terminarlo.
    `log`: archivo abierto donde volcar stdout/stderr (por defecto se descarta)."""
    cmd = [sys.executable, "-u", "-m", "live3d", "--port", str(port), *extra_args]
    out = log if log is not None else subprocess.DEVNULL
    env = dict(os.environ, PYTHONIOENCODING="utf-8")
    return subprocess.Popen(cmd, cwd=ROOT, stdout=out, stderr=subprocess.STDOUT, env=env)


async def wait_ready(base_url, timeout=30.0):
    deadline = time.time() + timeout
    async with aiohttp.ClientSession() as s:
        while time.time() < deadline:
            try:
                async with s.get(f"{base_url}/api/status", timeout=aiohttp.ClientTimeout(total=1)) as r:
                    if r.status == 200:
                        return await r.json()
            except (aiohttp.ClientError, asyncio.TimeoutError):
                pass
            await asyncio.sleep(0.3)
    raise TimeoutError(f"El servidor {base_url} no respondió en {timeout:.0f} s")


class CamStats:
    def __init__(self):
        self.frames = 0
        self.bytes = 0
        self.depth_bytes = 0
        self.color_bytes = 0
        self.latency_ms = []
        self.resolution = None
        self.t_first = None
        self.t_last = None

    def add(self, hdr, size, meta):
        now = time.perf_counter()
        if self.t_first is None:
            self.t_first = now
        self.t_last = now
        self.frames += 1
        self.bytes += size
        self.depth_bytes += hdr["depth_len"]
        self.color_bytes += hdr["color_len"]
        self.latency_ms.append((time.time() - hdr["timestamp"]) * 1000.0)
        if meta:
            d = meta["depth"]
            self.resolution = f"{d['width']}x{d['height']}"

    def summary(self):
        span = (self.t_last - self.t_first) if self.frames > 1 else 0.0
        lat = sorted(self.latency_ms)
        return {
            "frames": self.frames,
            "fps": round((self.frames - 1) / span, 1) if span > 0 else 0.0,
            "mbps": round(self.bytes * 8 / span / 1e6, 2) if span > 0 else 0.0,
            "kb_per_frame": round(self.bytes / max(1, self.frames) / 1024, 1),
            "depth_kb_per_frame": round(self.depth_bytes / max(1, self.frames) / 1024, 1),
            "color_kb_per_frame": round(self.color_bytes / max(1, self.frames) / 1024, 1),
            "latency_ms_p50": round(statistics.median(lat), 1) if lat else None,
            "latency_ms_p95": round(lat[int(0.95 * (len(lat) - 1))], 1) if lat else None,
            "depth_resolution": self.resolution,
        }


async def measure_stream(base_url, seconds, depth_codec="zlib", color_codec="jpeg", ack=True):
    ws_url = base_url.replace("http", "ws", 1) + "/ws"
    metas = {}
    cams = {}
    status_samples = []
    async with aiohttp.ClientSession() as s:
        async with s.ws_connect(ws_url, max_msg_size=0, compress=0) as ws:
            await ws.send_str(json.dumps({"type": "subscribe", "client": "bench",
                                          "depth": depth_codec, "color": color_codec, "ack": ack}))
            t_end = time.perf_counter() + seconds
            while True:
                remaining = t_end - time.perf_counter()
                if remaining <= 0:
                    break
                try:
                    msg = await ws.receive(timeout=remaining)
                except asyncio.TimeoutError:
                    break
                if msg.type == aiohttp.WSMsgType.TEXT:
                    data = json.loads(msg.data)
                    if data.get("type") == "meta":
                        metas[data["meta_id"]] = data
                    elif data.get("type") == "status":
                        status_samples.append(data)
                elif msg.type == aiohttp.WSMsgType.BINARY:
                    if ack:
                        await ws.send_str('{"type":"ack"}')
                    hdr = protocol.parse_header(msg.data)
                    cam = hdr["flags"] & 1
                    cams.setdefault(cam, CamStats()).add(hdr, len(msg.data), metas.get(hdr["meta_id"]))
                else:
                    break
    return cams, status_samples


def summarize_status(samples):
    """Promedia lo que el servidor reporta (ignora la primera muestra, de arranque)."""
    samples = samples[1:] or samples
    if not samples:
        return {}

    def avg(vals):
        vals = [v for v in vals if v is not None]
        return round(statistics.mean(vals), 2) if vals else None

    cam = samples[-1].get("camera", {})
    return {
        "source": cam.get("source"),
        "device": cam.get("device"),
        "profile": cam.get("profile"),
        "camera_fps": avg([x.get("camera", {}).get("fps") for x in samples]),
        "proc_ms": avg([x.get("camera", {}).get("proc_ms") for x in samples]),
        "encode_ms": avg([x.get("encode_ms") for x in samples]),
        "server_drop_fps": avg([x.get("you", {}).get("drop_fps") for x in samples]),
        "settings": samples[-1].get("settings"),
    }


async def time_capture(base_url, timeout=60.0):
    async with aiohttp.ClientSession() as s:
        t0 = time.perf_counter()
        async with s.post(f"{base_url}/api/dual/capture", timeout=aiohttp.ClientTimeout(total=timeout)) as r:
            body = await r.json()
        return {"capture_s": round(time.perf_counter() - t0, 2), "http_status": r.status,
                "status": body.get("status"), "message": body.get("message"),
                "rel_folder": body.get("rel_folder")}


async def run_bench(base_url, seconds, capture=False, depth_codec="zlib", color_codec="jpeg"):
    await wait_ready(base_url)
    cams, samples = await measure_stream(base_url, seconds, depth_codec, color_codec)
    report = {
        "date": time.strftime("%Y-%m-%d %H:%M:%S"),
        "url": base_url,
        "seconds": seconds,
        "codecs": {"depth": depth_codec, "color": color_codec},
        "server": summarize_status(samples),
        "cameras": {f"cam{k + 1}": v.summary() for k, v in sorted(cams.items())},
    }
    if capture:
        report["capture"] = await time_capture(base_url)
    return report


def print_report(r):
    srv = r["server"]
    print(f"\n=== Benchmark Live3D · {r['date']} · {r['seconds']} s · {r['url']} ===")
    print(f"Fuente: {srv.get('source')} · {srv.get('device')} · {srv.get('profile')}")
    print(f"Servidor: cámara {srv.get('camera_fps')} FPS · proc {srv.get('proc_ms')} ms · "
          f"encode {srv.get('encode_ms')} ms · descartes {srv.get('server_drop_fps')} /s")
    print(f"{'cam':<6}{'res':>10}{'fps':>8}{'Mb/s':>9}{'KB/cuadro':>11}{'depth KB':>10}"
          f"{'color KB':>10}{'lat p50':>9}{'lat p95':>9}")
    for name, c in r["cameras"].items():
        print(f"{name:<6}{str(c['depth_resolution']):>10}{c['fps']:>8}{c['mbps']:>9}{c['kb_per_frame']:>11}"
              f"{c['depth_kb_per_frame']:>10}{c['color_kb_per_frame']:>10}"
              f"{str(c['latency_ms_p50']):>9}{str(c['latency_ms_p95']):>9}")
    if not r["cameras"]:
        print("(no se recibió ningún cuadro)")
    if "capture" in r:
        c = r["capture"]
        print(f"Captura dual: {c['capture_s']} s · HTTP {c['http_status']} · {c['message']}")


def main(argv=None):
    p = argparse.ArgumentParser(description="Benchmark de línea base del servidor Live 3D")
    p.add_argument("--host", default="127.0.0.1")
    p.add_argument("--port", type=int, default=5002)
    p.add_argument("--seconds", type=float, default=10.0)
    p.add_argument("--depth", default="zlib", choices=sorted(protocol.DEPTH_CODECS))
    p.add_argument("--color", default="jpeg", choices=sorted(protocol.COLOR_CODECS))
    p.add_argument("--capture", action="store_true", help="cronometrar también POST /api/dual/capture")
    p.add_argument("--spawn", action="store_true", help="lanzar un servidor sintético dual propio")
    p.add_argument("--json", default=None, help="guardar el informe en este archivo JSON")
    args = p.parse_args(argv)

    proc = None
    if args.spawn:
        args.port = free_port()
        proc = spawn_server(args.port, ("--synthetic", "--dual"))
    base_url = f"http://{args.host}:{args.port}"
    try:
        report = asyncio.run(run_bench(base_url, args.seconds, args.capture, args.depth, args.color))
    finally:
        if proc:
            proc.terminate()
            proc.wait(10)

    print_report(report)
    if args.json:
        with open(args.json, "w", encoding="utf-8") as f:
            json.dump(report, f, indent=2, ensure_ascii=False)
        print(f"Informe guardado en {args.json}")


if __name__ == "__main__":
    main()
