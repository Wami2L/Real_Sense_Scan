"""
PLY de 3D Gaussian Splatting (formato INRIA: x y z, [nx ny nz], f_dc_0..2, f_rest_*, opacity, scale_0..2,
rot_0..3, todo float32 little-endian).

Convenciones del formato:
- opacity guarda el logit: alfa = sigmoid(opacity).
- scale_* guarda el logaritmo de la desviación típica por eje: sigma = exp(scale).
- rot_* es un cuaternión (w, x, y, z) sin normalizar.
- color base = 0.5 + SH_C0 · f_dc. f_rest se agrupa por canal: los primeros K coeficientes son R, luego G y B
  (K = 3 con SH1, 8 con SH2, 15 con SH3).
"""

import os
from dataclasses import dataclass, field

import numpy as np

SH_C0 = 0.28209479177387814
_REST_PER_DEGREE = {0: 0, 1: 3, 2: 8, 3: 15}
_PLY_TYPES = {"float": "<f4", "float32": "<f4", "double": "<f8", "uchar": "u1", "uint8": "u1",
              "int": "<i4", "int32": "<i4", "uint": "<u4", "uint32": "<u4", "short": "<i2", "ushort": "<u2"}


def sigmoid(x):
    return 1.0 / (1.0 + np.exp(-x))


@dataclass
class SplatPly:
    """Cabecera de un PLY de splats y acceso por bloques a sus datos (memmap de solo lectura)."""

    path: str
    count: int
    props: list
    dtype: np.dtype
    header_size: int
    comments: list = field(default_factory=list)

    @classmethod
    def open(cls, path):
        with open(path, "rb") as f:
            head = f.read(64 * 1024)
        end = head.find(b"end_header")
        if not head.startswith(b"ply") or end < 0:
            raise ValueError(f"{path}: no es un PLY")
        header_size = head.index(b"\n", end) + 1
        lines = head[:header_size].decode("ascii").splitlines()
        if "format binary_little_endian 1.0" not in lines:
            raise ValueError(f"{path}: solo se admite PLY binario little-endian")
        count, props, comments, in_vertex = 0, [], [], False
        for ln in lines:
            parts = ln.split()
            if not parts:
                continue
            if parts[0] == "comment":
                comments.append(ln[8:])
            elif parts[0] == "element":
                in_vertex = parts[1] == "vertex"
                if in_vertex:
                    count = int(parts[2])
                elif int(parts[2]) > 0:
                    raise ValueError(f"{path}: elemento '{parts[1]}' no soportado")
            elif parts[0] == "property" and in_vertex:
                if parts[1] == "list":
                    raise ValueError(f"{path}: propiedades lista no soportadas")
                props.append((parts[2], _PLY_TYPES[parts[1]]))
        dtype = np.dtype(props)
        expected = header_size + count * dtype.itemsize
        size = os.path.getsize(path)
        if size < expected:
            raise ValueError(f"{path}: truncado ({size} B, se esperaban {expected} B)")
        return cls(path, count, [p[0] for p in props], dtype, header_size, comments)

    # ------------------------------------------------------------------ descripción

    @property
    def sh_rest(self):
        return sum(1 for p in self.props if p.startswith("f_rest_"))

    @property
    def sh_degree(self):
        per_channel = self.sh_rest // 3
        for deg, k in _REST_PER_DEGREE.items():
            if k == per_channel:
                return deg
        raise ValueError(f"número de coeficientes SH inesperado: {self.sh_rest}")

    @property
    def bytes_per_splat(self):
        return self.dtype.itemsize

    def require(self, *names):
        missing = [n for n in names if n not in self.props]
        if missing:
            raise ValueError(f"{self.path}: faltan propiedades {missing}")

    # ------------------------------------------------------------------ datos

    def memmap(self):
        return np.memmap(self.path, dtype=self.dtype, mode="r", offset=self.header_size, shape=(self.count,))

    def chunks(self, size=1_000_000):
        """Recorre el archivo por bloques: (inicio, array estructurado en memoria)."""
        mm = self.memmap()
        for start in range(0, self.count, size):
            yield start, np.array(mm[start:start + size])
        del mm

    def columns(self, names, size=1_000_000):
        """Lee solo las propiedades pedidas como float32 (N, len(names)), bloque a bloque."""
        self.require(*names)
        out = np.empty((self.count, len(names)), np.float32)
        for start, block in self.chunks(size):
            for j, n in enumerate(names):
                out[start:start + len(block), j] = block[n]
        return out

    def rest_names(self, degree=None):
        """Nombres f_rest_* hasta el grado SH pedido (por defecto, todos)."""
        per = self.sh_rest // 3
        keep = _REST_PER_DEGREE[self.sh_degree if degree is None else min(degree, self.sh_degree)]
        return [f"f_rest_{c * per + k}" for c in range(3) for k in range(keep)]


def write_ply(path, src: SplatPly, mask=None, sh_degree=None, transform=None, comments=(), size=1_000_000):
    """
    Escribe un PLY nuevo a partir de `src` por bloques: filtra con `mask` (bool por splat), recorta SH a
    `sh_degree` (renumerando f_rest_* como espera el formato) y aplica `transform` (función bloque → bloque
    sobre un dict de arrays float32, p. ej. rotar/escalar la escena). Devuelve el número de splats escritos.
    """
    src.require("x", "y", "z", "opacity", "f_dc_0", "f_dc_1", "f_dc_2")
    rest_in = src.rest_names(sh_degree)
    base = [p for p in src.props if not p.startswith("f_rest_") and p not in ("nx", "ny", "nz")]
    out_props = base[:]
    # f_rest al final, renumerados 0..n-1 en el orden canal-mayor del formato.
    out_props += [f"f_rest_{i}" for i in range(len(rest_in))]
    in_for_out = base + rest_in
    out_dtype = np.dtype([(p, "<f4") for p in out_props])

    n_out = int(mask.sum()) if mask is not None else src.count
    header = ["ply", "format binary_little_endian 1.0"]
    header += [f"comment {c}" for c in list(src.comments) + list(comments)]
    header += [f"element vertex {n_out}"] + [f"property float {p}" for p in out_props] + ["end_header"]
    tmp = path + ".tmp"
    written = 0
    with open(tmp, "wb") as f:
        f.write(("\n".join(header) + "\n").encode("ascii"))
        for start, block in src.chunks(size):
            if mask is not None:
                block = block[mask[start:start + len(block)]]
            if not len(block):
                continue
            cols = {p: block[p].astype(np.float32) for p in in_for_out}
            if transform is not None:
                cols = transform(cols)
            out = np.empty(len(block), out_dtype)
            for p_out, p_in in zip(out_props, in_for_out):
                out[p_out] = cols[p_in]
            f.write(out.tobytes())
            written += len(block)
    assert written == n_out, (written, n_out)
    os.replace(tmp, path)
    return written
