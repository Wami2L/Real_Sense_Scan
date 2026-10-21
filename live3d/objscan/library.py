"""
Biblioteca de objetos escaneados: Saves/objetos/<id>/

  objeto.json          manifiesto (ver Library.create)
  objeto.ply           malla del escáner (marco base: metros, Y arriba, pivote en la base)
  objeto_limpio.ply    malla tras Blender (mismo marco), si existe
  objeto_puntos.ply    nube del objeto
  objeto.obj/.mtl      malla con UV y textura (para Blender y otros programas)
  textura.jpg          textura fotográfica (atlas UV)
  modelo.rsm           malla para Unity, con UV y textura (se regenera si la fuente es más nueva;
                       una malla de Blender se re-texturiza con las fotos de la sesión)
  miniatura.jpg

La carpeta es la fuente de verdad: el servidor de escaneo, Blender y el enlace con Unity
leen y escriben aquí. Estados: crudo -> limpio -> (publicado = true/false, aparte).
"""

import json
import os
import re
import time
import unicodedata

import numpy as np

ROOT_DIR = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
LIB_DIR = os.path.join(ROOT_DIR, "Saves", "objetos")


def slug(text):
    t = unicodedata.normalize("NFKD", text or "objeto").encode("ascii", "ignore").decode()
    t = re.sub(r"[^a-zA-Z0-9]+", "_", t).strip("_").lower()
    return t[:40] or "objeto"


class Library:
    def __init__(self, root=LIB_DIR):
        self.root = root
        os.makedirs(root, exist_ok=True)

    # ---- rutas ----
    def path(self, oid, name=""):
        if not re.fullmatch(r"[a-z0-9_]+", oid or ""):
            raise ValueError("id de objeto inválido")
        return os.path.join(self.root, oid, name)

    def exists(self, oid):
        try:
            return os.path.isfile(self.path(oid, "objeto.json"))
        except ValueError:
            return False

    # ---- manifiestos ----
    def get(self, oid):
        with open(self.path(oid, "objeto.json"), encoding="utf-8") as f:
            return json.load(f)

    def _write(self, oid, man):
        p = self.path(oid, "objeto.json")
        tmp = p + ".tmp"
        with open(tmp, "w", encoding="utf-8") as f:
            json.dump(man, f, indent=2, ensure_ascii=False)
        os.replace(tmp, p)

    def update(self, oid, **fields):
        man = self.get(oid)
        man.update(fields)
        man["actualizado"] = time.time()
        self._write(oid, man)
        return man

    def list(self, published_only=False, since=None):
        out = []
        for oid in sorted(os.listdir(self.root), reverse=True):
            if not self.exists(oid):
                continue
            try:
                man = self.get(oid)
            except (OSError, ValueError):
                continue
            if published_only and not man.get("publicado"):
                continue
            if since is not None and man.get("actualizado", 0) <= since:
                continue
            out.append(man)
        return out

    # ---- creación y publicación ----
    def create(self, name, mesh, points=None, meta=None, texture=None):
        """Guarda un objeto nuevo desde el escáner. mesh: open3d TriangleMesh en el marco base
        (con `triangle_uvs` si hay textura); texture: imagen RGB uint8 del atlas UV."""
        import open3d as o3d
        oid = time.strftime("%Y%m%d_%H%M%S_") + slug(name)
        os.makedirs(self.path(oid), exist_ok=True)
        o3d.io.write_triangle_mesh(self.path(oid, "objeto.ply"), mesh, write_ascii=False)
        if points is not None and len(points.points):
            o3d.io.write_point_cloud(self.path(oid, "objeto_puntos.ply"), points)
        V, T = np.asarray(mesh.vertices), np.asarray(mesh.triangles)
        jpeg = None
        if texture is not None and mesh.has_triangle_uvs():
            jpeg = self._save_texture(oid, texture)
            write_obj(self.path(oid, "objeto.obj"), mesh, "textura.jpg")
        self._write_rsm(oid, mesh, jpeg)
        from .thumbnail import render_thumbnail
        import cv2
        cv2.imwrite(self.path(oid, "miniatura.jpg"), render_thumbnail(mesh))
        lo, hi = (V.min(0), V.max(0)) if len(V) else (np.zeros(3), np.zeros(3))
        now = time.time()
        man = {
            "id": oid, "nombre": name or "Objeto", "creado": now, "actualizado": now,
            "estado": "crudo", "publicado": False, "version": 1,
            "tamano_m": [round(float(x), 4) for x in (hi - lo)],
            "vertices": int(len(V)), "triangulos": int(len(T)),
            "marco": "base: metros, Y arriba, diestro, pivote en el centro de la base",
            "fuente": "objeto.ply",
        }
        man.update(meta or {})
        self._write(oid, man)
        return man

    def source_mesh_path(self, oid):
        clean = self.path(oid, "objeto_limpio.ply")
        return clean if os.path.isfile(clean) else self.path(oid, "objeto.ply")

    def _save_texture(self, oid, texture):
        import cv2
        ok, buf = cv2.imencode(".jpg", cv2.cvtColor(np.asarray(texture), cv2.COLOR_RGB2BGR),
                               [cv2.IMWRITE_JPEG_QUALITY, 92])
        jpeg = buf.tobytes()
        with open(self.path(oid, "textura.jpg"), "wb") as f:
            f.write(jpeg)
        return jpeg

    def _write_rsm(self, oid, mesh, jpeg=None):
        from .rsm import write_rsm
        if not mesh.has_vertex_normals():
            mesh.compute_vertex_normals()
        if jpeg and mesh.has_triangle_uvs():
            from .textura import split_by_uv
            V, N, C, UV, F = split_by_uv(mesh)
            write_rsm(self.path(oid, "modelo.rsm"), V, F, N, C, uvs=UV, texture_bytes=jpeg)
        else:
            write_rsm(self.path(oid, "modelo.rsm"), np.asarray(mesh.vertices), np.asarray(mesh.triangles),
                      np.asarray(mesh.vertex_normals),
                      np.asarray(mesh.vertex_colors) if mesh.has_vertex_colors() else None)

    def _retexture(self, oid, mesh):
        """Texturiza una malla nueva (p. ej. la limpia de Blender) con las fotos de la sesión que
        creó el objeto. Devuelve (malla con UV, jpeg) o (mesh, None) si no hay fotos."""
        import cv2
        from .textura import TexView, bake
        man = self.get(oid)
        ses = man.get("sesion")
        folder = os.path.join(ROOT_DIR, ses) if ses else None
        if not folder or not os.path.isdir(folder):
            return mesh, None
        To = np.eye(4)
        To[:3, 3] = man.get("pivote_desplazado_m") or [0, 0, 0]
        views = []
        for d in sorted(os.listdir(folder)):
            vd = os.path.join(folder, d)
            if not (d.startswith("vista_") and os.path.isfile(os.path.join(vd, "vista.json"))):
                continue
            with open(os.path.join(vd, "vista.json"), encoding="utf-8") as f:
                j = json.load(f)
            col = cv2.imread(os.path.join(vd, "color.jpg"))
            if col is None or not j.get("T_cam_a_base"):
                continue
            mk = cv2.imread(os.path.join(vd, "mascara.png"), cv2.IMREAD_GRAYSCALE)
            views.append(TexView(cv2.cvtColor(col, cv2.COLOR_BGR2RGB), j["intrinsics"],
                                 To @ np.array(j["T_cam_a_base"]), None if mk is None else mk > 0))
        if not views:
            return mesh, None
        m, tex, info = bake(mesh, views, max_triangles=max(len(mesh.triangles), 1))
        jpeg = self._save_texture(oid, tex)
        write_obj(self.path(oid, "objeto.obj"), m, "textura.jpg")
        self.update(oid, textura=info)
        return m, jpeg

    def ensure_rsm(self, oid):
        """modelo.rsm al día con la malla fuente (la de Blender si existe). Devuelve la ruta."""
        src, rsm = self.source_mesh_path(oid), self.path(oid, "modelo.rsm")
        if os.path.isfile(rsm) and os.path.getmtime(rsm) >= os.path.getmtime(src):
            return rsm
        import open3d as o3d
        mesh = o3d.io.read_triangle_mesh(src)
        if not mesh.has_vertex_normals():
            mesh.compute_vertex_normals()
        jpeg = None
        try:
            if src.endswith("objeto_limpio.ply"):
                mesh, jpeg = self._retexture(oid, mesh)
            elif os.path.isfile(self.path(oid, "objeto.obj")) and os.path.isfile(self.path(oid, "textura.jpg")):
                tm = o3d.io.read_triangle_mesh(self.path(oid, "objeto.obj"))
                if tm.has_triangle_uvs() and len(tm.vertices) == len(mesh.vertices):
                    tm.vertex_colors = mesh.vertex_colors
                    tm.compute_vertex_normals()
                    mesh = tm
                    with open(self.path(oid, "textura.jpg"), "rb") as f:
                        jpeg = f.read()
        except Exception:  # noqa: BLE001  (sin textura, al menos colores por vértice)
            import traceback
            traceback.print_exc()
            jpeg = None
        self._write_rsm(oid, mesh, jpeg)
        try:
            import cv2
            from .thumbnail import render_thumbnail
            cv2.imwrite(self.path(oid, "miniatura.jpg"), render_thumbnail(mesh))
        except Exception:  # noqa: BLE001  (la miniatura es accesoria)
            pass
        man = self.get(oid)
        V = np.asarray(mesh.vertices)
        self.update(oid, vertices=int(len(V)), triangulos=int(len(mesh.triangles)),
                    tamano_m=[round(float(x), 4) for x in (V.max(0) - V.min(0))] if len(V) else man.get("tamano_m"),
                    fuente=os.path.basename(src))
        return rsm

    def publish(self, oid, value=True):
        man = self.get(oid)
        return self.update(oid, publicado=bool(value), version=int(man.get("version", 1)) + (1 if value else 0))

    def rename(self, oid, name):
        return self.update(oid, nombre=(name or "Objeto")[:80])

    def delete(self, oid):
        import shutil
        shutil.rmtree(self.path(oid), ignore_errors=True)


def write_obj(path, mesh, texture_name):
    """OBJ + MTL con UV por triángulo y la textura (Blender lo importa con su material)."""
    base = os.path.splitext(path)[0]
    mtl = os.path.basename(base) + ".mtl"
    V = np.asarray(mesh.vertices)
    F = np.asarray(mesh.triangles)
    UV = np.asarray(mesh.triangle_uvs)
    N = np.asarray(mesh.vertex_normals) if mesh.has_vertex_normals() else None
    with open(base + ".mtl", "w", encoding="utf-8") as f:
        f.write(f"newmtl escaneo\nKa 1 1 1\nKd 1 1 1\nKs 0 0 0\nillum 1\nmap_Kd {texture_name}\n")
    lines = [f"mtllib {mtl}\n", "o objeto\n"]
    lines += [f"v {x:.6f} {y:.6f} {z:.6f}\n" for x, y, z in V]
    lines += [f"vt {u:.6f} {v:.6f}\n" for u, v in UV]
    if N is not None:
        lines += [f"vn {x:.5f} {y:.5f} {z:.5f}\n" for x, y, z in N]
    lines.append("usemtl escaneo\n")
    for k, (a, b, c) in enumerate(F):
        t = 3 * k + 1
        if N is not None:
            lines.append(f"f {a + 1}/{t}/{a + 1} {b + 1}/{t + 1}/{b + 1} {c + 1}/{t + 2}/{c + 1}\n")
        else:
            lines.append(f"f {a + 1}/{t} {b + 1}/{t + 1} {c + 1}/{t + 2}\n")
    with open(path, "w", encoding="utf-8") as f:
        f.writelines(lines)
