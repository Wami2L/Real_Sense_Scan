"""
Formato RSM ("RealSense Mesh"): malla compacta para Unity en tiempo de ejecución.

Se escribe YA en la convención de Unity (zurdo, Y arriba, metros): z -> -z respecto al marco
base y el orden de los triángulos invertido, para que el cargador C# sea una copia directa.

Disposición (little-endian):
  0   4s   magic  b"RSM1"
  4   u32  versión (1 sin textura, 2 con UV y textura)
  8   u32  n_vértices
 12   u32  n_índices (3 por triángulo)
 16   u32  flags: bit0 normales, bit1 colores RGBA8, bit2 UV, bit3 textura
 20   6f   bounds min xyz, max xyz (Unity)
 44   ...  posiciones f32[3n] · normales f32[3n] (si bit0) · colores u8[4n] (si bit1)
           · UV f32[2n] (si bit2; v hacia arriba, como Unity) · índices u32[m]
           · textura (si bit3): u32 bytes + imagen JPEG/PNG (la carga Texture2D.LoadImage)
"""

import struct

import numpy as np

MAGIC = b"RSM1"
HEADER = struct.Struct("<4sIIII6f")


def to_unity(vertices, normals=None, triangles=None):
    v = np.asarray(vertices, np.float32).copy()
    v[:, 2] *= -1
    n = None
    if normals is not None and len(normals):
        n = np.asarray(normals, np.float32).copy()
        n[:, 2] *= -1
    t = None if triangles is None else np.asarray(triangles, np.uint32)[:, ::-1].copy()
    return v, n, t


def write_rsm(path, vertices, triangles, normals=None, colors=None, uvs=None, texture_bytes=None):
    """vertices/normals en el marco base (diestro, Y arriba); colors en 0..1 o uint8 RGB/RGBA;
    uvs (n, 2) por vértice con v hacia arriba; texture_bytes: imagen ya codificada (JPEG/PNG)."""
    v, n, t = to_unity(vertices, normals, triangles)
    flags = 0
    parts = [v.tobytes()]
    if n is not None:
        flags |= 1
        parts.append(n.tobytes())
    if colors is not None and len(colors):
        c = np.asarray(colors)
        if c.dtype != np.uint8:
            c = np.clip(np.round(c * 255), 0, 255).astype(np.uint8)
        if c.shape[1] == 3:
            c = np.hstack((c, np.full((len(c), 1), 255, np.uint8)))
        flags |= 2
        parts.append(np.ascontiguousarray(c).tobytes())
    if uvs is not None and len(uvs):
        flags |= 4
        parts.append(np.ascontiguousarray(uvs, np.float32).tobytes())
    parts.append(t.ravel().tobytes())
    if texture_bytes:
        flags |= 8
        parts.append(struct.pack("<I", len(texture_bytes)) + bytes(texture_bytes))
    version = 2 if flags & 12 else 1
    lo, hi = (v.min(0), v.max(0)) if len(v) else (np.zeros(3), np.zeros(3))
    with open(path, "wb") as f:
        f.write(HEADER.pack(MAGIC, version, len(v), t.size, flags, *lo.tolist(), *hi.tolist()))
        for p in parts:
            f.write(p)


def read_rsm(path):
    """Lectura (para tests y el visor web): devuelve dict en la convención de Unity."""
    with open(path, "rb") as f:
        buf = f.read()
    magic, ver, nv, ni, flags, *b = HEADER.unpack_from(buf, 0)
    if magic != MAGIC:
        raise ValueError("no es un archivo RSM")
    o = HEADER.size
    out = {"version": ver, "bounds": (np.array(b[:3]), np.array(b[3:]))}
    out["vertices"] = np.frombuffer(buf, np.float32, nv * 3, o).reshape(nv, 3)
    o += nv * 12
    if flags & 1:
        out["normals"] = np.frombuffer(buf, np.float32, nv * 3, o).reshape(nv, 3)
        o += nv * 12
    if flags & 2:
        out["colors"] = np.frombuffer(buf, np.uint8, nv * 4, o).reshape(nv, 4)
        o += nv * 4
    if flags & 4:
        out["uvs"] = np.frombuffer(buf, np.float32, nv * 2, o).reshape(nv, 2)
        o += nv * 8
    out["indices"] = np.frombuffer(buf, np.uint32, ni, o).reshape(-1, 3)
    o += ni * 4
    if flags & 8:
        (n,) = struct.unpack_from("<I", buf, o)
        out["texture"] = buf[o + 4:o + 4 + n]
    return out
