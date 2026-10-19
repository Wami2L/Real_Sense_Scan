"""
Protocolo de streaming RealSense Live 3D (versión 1).

Este módulo SOLO depende de la librería estándar + numpy para poder importarse
tal cual desde el Python embebido de Blender (próxima iteración).

Idea clave de optimización
--------------------------
No se envían nubes de puntos (12 bytes/punto en float32). Se envía:

  * Profundidad cruda uint16 (ya decimada en el servidor)  -> 2 bytes/punto
    opcionalmente comprimida con zlib (formato zlib = 'deflate' en navegador).
  * Color en su resolución nativa, como JPEG (navegador) o RGB crudo (Blender).
  * Un mensaje JSON "meta" (solo cuando cambia) con intrínsecas de profundidad
    y color + extrínsecas profundidad->color.

Cada cliente reconstruye la geometría: el navegador en la GPU (vertex shader) y
Blender con numpy (`deproject`). No se alinea color->profundidad en la CPU.

Mensajes
--------
Servidor -> cliente
  * Texto  JSON {"type": "meta", ...}    intrínsecas/extrínsecas (ver build_meta)
  * Texto  JSON {"type": "status", ...}  estado/fps/ajustes (~1 Hz)
  * Binario  cuadro: HEADER (32 bytes, little-endian) + depth + color

Cliente -> servidor (JSON)
  * {"type": "subscribe", "depth": "zlib"|"raw",
     "color": "jpeg"|"raw"|"none", "ack": true|false}
  * {"type": "ack", "seq": N}          control de flujo (si ack=true)
  * {"type": "settings", ...}          cambia ajustes globales del servidor

Cabecera binaria (struct "<4sIdHBBIII", 32 bytes)
  off  tipo  campo
   0   4s    magic        b"RSF1"
   4   u32   seq          número de cuadro
   8   f64   timestamp    segundos (reloj del servidor, time.time())
  16   u16   meta_id      id del mensaje meta que describe este cuadro
  18   u8    depth_codec  0 = raw uint16 LE, 1 = zlib(raw)
  19   u8    color_codec  0 = sin color, 1 = JPEG, 2 = RGB8 crudo
  20   u32   depth_len    bytes de profundidad
  24   u32   color_len    bytes de color
  28   u32   flags        reservado (0)

Sistema de coordenadas: el de RealSense (x derecha, y abajo, z hacia delante,
metros). Cada cliente convierte al suyo (Three.js: (x,-y,-z); Blender: (x,z,-y)).
"""

import json
import struct
import zlib

import numpy as np

MAGIC = b"RSF1"
PROTOCOL_VERSION = 1
HEADER = struct.Struct("<4sIdHBBIII")
HEADER_SIZE = HEADER.size  # 32

DEPTH_CODECS = {"raw": 0, "zlib": 1}
COLOR_CODECS = {"none": 0, "jpeg": 1, "raw": 2}
DEPTH_CODEC_NAMES = {v: k for k, v in DEPTH_CODECS.items()}
COLOR_CODEC_NAMES = {v: k for k, v in COLOR_CODECS.items()}

ZLIB_LEVEL = 1  # nivel 1: ~1-2 ms para 424x240, la mayor parte del beneficio


# --------------------------------------------------------------------------- #
# Meta
# --------------------------------------------------------------------------- #
def build_meta(meta_id, depth_intr, depth_scale, color_intr=None, rotation=None,
               translation=None, extra=None):
    """Construye el mensaje meta.

    depth_intr / color_intr: dict con width, height, fx, fy, ppx, ppy.
    rotation: 9 floats FILA-MAYOR (p_color = R @ p_depth + t).
    """
    meta = {
        "type": "meta",
        "version": PROTOCOL_VERSION,
        "meta_id": int(meta_id),
        "depth": dict(depth_intr, scale=float(depth_scale)),
        "color": dict(color_intr) if color_intr else None,
        "depth_to_color": {
            "rotation": [float(v) for v in (rotation or [1, 0, 0, 0, 1, 0, 0, 0, 1])],
            "translation": [float(v) for v in (translation or [0, 0, 0])],
        },
    }
    if extra:
        meta.update(extra)
    return meta


def meta_to_json(meta):
    return json.dumps(meta, separators=(",", ":"))


# --------------------------------------------------------------------------- #
# Codificación (servidor)
# --------------------------------------------------------------------------- #
def encode_depth(depth_u16, codec="zlib"):
    raw = np.ascontiguousarray(depth_u16, dtype="<u2").tobytes()
    if codec == "zlib":
        return zlib.compress(raw, ZLIB_LEVEL)
    return raw


def pack_frame(seq, timestamp, meta_id, depth_codec, depth_bytes, color_codec, color_bytes, flags=0):
    """flags bit 0: índice de cámara de origen (0 = Cam 1, 1 = Cam 2) en modo dual."""
    color_bytes = color_bytes or b""
    header = HEADER.pack(
        MAGIC, seq & 0xFFFFFFFF, float(timestamp), meta_id & 0xFFFF,
        DEPTH_CODECS[depth_codec], COLOR_CODECS[color_codec],
        len(depth_bytes), len(color_bytes), flags & 0xFFFFFFFF,
    )
    return b"".join((header, depth_bytes, color_bytes))


# --------------------------------------------------------------------------- #
# Decodificación (clientes Python / Blender)
# --------------------------------------------------------------------------- #
def parse_header(buf):
    magic, seq, ts, meta_id, dc, cc, dlen, clen, flags = HEADER.unpack_from(buf, 0)
    if magic != MAGIC:
        raise ValueError(f"magic inválido: {magic!r}")
    return {
        "seq": seq, "timestamp": ts, "meta_id": meta_id,
        "depth_codec": DEPTH_CODEC_NAMES.get(dc, dc),
        "color_codec": COLOR_CODEC_NAMES.get(cc, cc),
        "depth_len": dlen, "color_len": clen, "flags": flags,
    }


def decode_frame(buf, meta):
    """Decodifica un cuadro binario. Devuelve dict con:
    depth (H,W) uint16, color (h,w,3) uint8 | bytes JPEG | None, y la cabecera.
    """
    h = parse_header(buf)
    if h["meta_id"] != meta["meta_id"]:
        raise ValueError("el cuadro pertenece a otro meta_id (esperar nuevo meta)")
    mv = memoryview(buf)
    o = HEADER_SIZE
    dpay = mv[o:o + h["depth_len"]]
    o += h["depth_len"]
    cpay = mv[o:o + h["color_len"]]

    raw = zlib.decompress(dpay) if h["depth_codec"] == "zlib" else bytes(dpay)
    dw, dh = meta["depth"]["width"], meta["depth"]["height"]
    depth = np.frombuffer(raw, dtype="<u2").reshape(dh, dw)

    color = None
    if h["color_codec"] == "raw" and meta.get("color"):
        cw, ch = meta["color"]["width"], meta["color"]["height"]
        color = np.frombuffer(bytes(cpay), dtype=np.uint8).reshape(ch, cw, 3)
    elif h["color_codec"] == "jpeg":
        color = bytes(cpay)  # el cliente decide cómo decodificar

    h.update(depth=depth, color=color)
    return h


def pixel_grid(meta):
    """Rejilla (u, v) precomputada de la profundidad: cachear y reutilizar."""
    d = meta["depth"]
    u = (np.arange(d["width"], dtype=np.float32) - d["ppx"]) / d["fx"]
    v = (np.arange(d["height"], dtype=np.float32) - d["ppy"]) / d["fy"]
    return np.meshgrid(u, v)


def deproject(depth, meta, grid=None, near=0.0, far=np.inf):
    """Profundidad uint16 -> puntos (N,3) float32 en metros (coords RealSense)
    y máscara (H,W) de píxeles válidos. Totalmente vectorizado."""
    gx, gy = grid if grid is not None else pixel_grid(meta)
    z = depth.astype(np.float32) * np.float32(meta["depth"]["scale"])
    mask = (z > 0) & (z >= near) & (z <= far)
    zm = z[mask]
    pts = np.empty((zm.size, 3), np.float32)
    pts[:, 0] = gx[mask] * zm
    pts[:, 1] = gy[mask] * zm
    pts[:, 2] = zm
    return pts, mask


def project_colors(points, color_rgb, meta):
    """Color por punto proyectando a la imagen de color con las extrínsecas.
    Puntos fuera del campo de visión del color reciben gris."""
    c = meta["color"]
    R = np.asarray(meta["depth_to_color"]["rotation"], np.float32).reshape(3, 3)
    t = np.asarray(meta["depth_to_color"]["translation"], np.float32)
    pc = points @ R.T + t
    z = np.maximum(pc[:, 2], 1e-6)
    u = np.round(pc[:, 0] / z * c["fx"] + c["ppx"]).astype(np.int32)
    v = np.round(pc[:, 1] / z * c["fy"] + c["ppy"]).astype(np.int32)
    inside = (u >= 0) & (u < c["width"]) & (v >= 0) & (v < c["height"])
    out = np.full((points.shape[0], 3), 128, np.uint8)
    out[inside] = color_rgb[v[inside], u[inside]]
    return out
