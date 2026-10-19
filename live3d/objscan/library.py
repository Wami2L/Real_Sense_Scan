"""
Biblioteca de objetos escaneados: Saves/objetos/<id>/

  objeto.json          manifiesto (ver Library.create)
  objeto.ply           malla del escáner (marco base: metros, Y arriba, pivote en la base)
  objeto_limpio.ply    malla tras Blender (mismo marco), si existe
  objeto_puntos.ply    nube del objeto
  modelo.rsm           malla para Unity (se regenera si la fuente es más nueva)
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
    def create(self, name, mesh, points=None, meta=None):
        """Guarda un objeto nuevo desde el escáner. mesh: open3d TriangleMesh en el marco base."""
        import open3d as o3d
        from .rsm import write_rsm
        oid = time.strftime("%Y%m%d_%H%M%S_") + slug(name)
        os.makedirs(self.path(oid), exist_ok=True)
        o3d.io.write_triangle_mesh(self.path(oid, "objeto.ply"), mesh, write_ascii=False)
        if points is not None and len(points.points):
            o3d.io.write_point_cloud(self.path(oid, "objeto_puntos.ply"), points)
        V, T = np.asarray(mesh.vertices), np.asarray(mesh.triangles)
        write_rsm(self.path(oid, "modelo.rsm"), V, T, np.asarray(mesh.vertex_normals),
                  np.asarray(mesh.vertex_colors) if mesh.has_vertex_colors() else None)
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

    def ensure_rsm(self, oid):
        """modelo.rsm al día con la malla fuente (la de Blender si existe). Devuelve la ruta."""
        src, rsm = self.source_mesh_path(oid), self.path(oid, "modelo.rsm")
        if os.path.isfile(rsm) and os.path.getmtime(rsm) >= os.path.getmtime(src):
            return rsm
        import open3d as o3d
        from .rsm import write_rsm
        mesh = o3d.io.read_triangle_mesh(src)
        if not mesh.has_vertex_normals():
            mesh.compute_vertex_normals()
        write_rsm(rsm, np.asarray(mesh.vertices), np.asarray(mesh.triangles), np.asarray(mesh.vertex_normals),
                  np.asarray(mesh.vertex_colors) if mesh.has_vertex_colors() else None)
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
