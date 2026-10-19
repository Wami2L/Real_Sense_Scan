"""
Cliente de prueba del stream Live 3D (valida el protocolo de punta a punta).

Es, a propósito, el esqueleto del futuro cliente de Blender: misma suscripción
(color RGB crudo + ACK), misma decodificación y deproyección con numpy.

    python -m live3d.tools.probe_client --frames 90
    python -m live3d.tools.probe_client --frames 1 --ply ultimo_cuadro.ply
"""

import argparse
import asyncio
import json
import time

import aiohttp
import numpy as np

from live3d import protocol


def write_ply(path, pts, cols):
    """PLY binario (Blender/Open3D/MeshLab)."""
    data = np.empty(len(pts), dtype=[("x", "<f4"), ("y", "<f4"), ("z", "<f4"),
                                    ("red", "u1"), ("green", "u1"), ("blue", "u1")])
    # RealSense (y abajo, z delante) -> convención Z-arriba de Blender
    data["x"], data["y"], data["z"] = pts[:, 0], pts[:, 2], -pts[:, 1]
    data["red"], data["green"], data["blue"] = cols[:, 0], cols[:, 1], cols[:, 2]
    header = ("ply\nformat binary_little_endian 1.0\n"
              f"element vertex {len(pts)}\n"
              "property float x\nproperty float y\nproperty float z\n"
              "property uchar red\nproperty uchar green\nproperty uchar blue\nend_header\n")
    with open(path, "wb") as f:
        f.write(header.encode("ascii"))
        f.write(data.tobytes())


async def run(args):
    url = f"ws://{args.host}:{args.port}/ws"
    meta, grid = None, None
    n, t_start = 0, None
    dec_ms = proj_ms = bytes_rx = 0.0
    async with aiohttp.ClientSession() as s:
        async with s.ws_connect(url, max_msg_size=0, compress=0) as ws:
            await ws.send_str(json.dumps({"type": "subscribe", "client": "probe",
                                          "depth": args.depth, "color": "raw", "ack": True}))
            async for msg in ws:
                if msg.type == aiohttp.WSMsgType.TEXT:
                    data = json.loads(msg.data)
                    if data["type"] == "meta":
                        meta, grid = data, protocol.pixel_grid(data)
                        d = meta["depth"]
                        print(f"meta #{meta['meta_id']}: depth {d['width']}x{d['height']} "
                              f"color {meta['color']['width']}x{meta['color']['height']}")
                    elif data["type"] == "status" and n == 0:
                        print("estado cámara:", data["camera"]["state"], "-", data["camera"]["message"])
                    continue
                if msg.type != aiohttp.WSMsgType.BINARY:
                    break
                buf = msg.data
                await ws.send_str('{"type":"ack"}')  # liberar crédito cuanto antes
                if meta is None:
                    continue
                t0 = time.perf_counter()
                try:
                    fr = protocol.decode_frame(buf, meta)
                except ValueError:
                    continue
                t1 = time.perf_counter()
                pts, _ = protocol.deproject(fr["depth"], meta, grid)
                cols = protocol.project_colors(pts, fr["color"], meta) if fr["color"] is not None else \
                    np.full((len(pts), 3), 200, np.uint8)
                t2 = time.perf_counter()
                if t_start is None:
                    t_start = time.perf_counter()
                n += 1
                dec_ms += (t1 - t0) * 1000
                proj_ms += (t2 - t1) * 1000
                bytes_rx += len(buf)
                if n % 30 == 0 or n == args.frames:
                    el = max(1e-6, time.perf_counter() - t_start)
                    print(f"cuadro {fr['seq']:>6}  {len(pts):>7} pts  {n / el:5.1f} fps  "
                          f"{bytes_rx * 8 / el / 1e6:6.1f} Mbps  decode {dec_ms / n:5.2f} ms  "
                          f"deproj+color {proj_ms / n:5.2f} ms  latencia~{(time.time() - fr['timestamp']) * 1000:5.1f} ms")
                if n >= args.frames:
                    if args.ply:
                        write_ply(args.ply, pts, cols)
                        print(f"PLY guardado: {args.ply} ({len(pts)} puntos)")
                    break


def main():
    p = argparse.ArgumentParser()
    p.add_argument("--host", default="127.0.0.1")
    p.add_argument("--port", type=int, default=5002)
    p.add_argument("--frames", type=int, default=90)
    p.add_argument("--depth", default="zlib", choices=["zlib", "raw"])
    p.add_argument("--ply", default=None, help="guardar el último cuadro como PLY")
    asyncio.run(run(p.parse_args()))


if __name__ == "__main__":
    main()
