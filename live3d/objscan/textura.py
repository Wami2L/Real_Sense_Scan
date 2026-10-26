"""
Textura fotográfica del objeto: atlas UV + proyección de las fotos de color de cada toma.

La malla sale del TSDF (geometría de la D455, a escala); el color, de las fotos con su pose
exacta (marcadores). Por cada texel del atlas:
  - se proyecta en cada vista y se comprueba que la vista lo ve (raycast contra la propia malla),
  - se pesa por ángulo de incidencia, distancia y lejanía al borde de la silueta (en los bordes
    profundidad y color no casan y se cuela el fondo),
  - se mezclan las vistas dando casi todo el peso a la mejor (nitidez sin costuras duras),
  - se compensa la exposición de cada vista (la D455 ajusta su autoexposición en cada toma),
  - lo que ninguna vista ve (la base apoyada en la mesa) se rellena por "push-pull".
Todo en CPU con numpy/OpenCV y el raycasting de Open3D: segundos, no minutos.
"""

import cv2
import numpy as np
import open3d as o3d


class TexView:
    """Una foto para texturizar: color RGB (rectificado), intrínsecas, pose cámara->malla y,
    opcional, máscara del objeto (bool, misma resolución)."""

    def __init__(self, color, intr, T_cam_to_mesh, mask=None):
        self.color = color
        self.K = np.array([[intr["fx"], 0, intr["ppx"]], [0, intr["fy"], intr["ppy"]], [0, 0, 1]], np.float64)
        self.T = np.asarray(T_cam_to_mesh, np.float64)
        self.mask = mask


def _prepare(mesh, max_triangles):
    m = o3d.geometry.TriangleMesh(mesh)
    if len(m.triangles) > max_triangles:
        m = m.simplify_quadric_decimation(max_triangles)
    m.remove_degenerate_triangles()
    m.remove_duplicated_triangles()
    m.remove_duplicated_vertices()
    m.remove_non_manifold_edges()
    m.remove_unreferenced_vertices()
    m.compute_vertex_normals()
    return m


def _texel_map(tmesh, size):
    """Posición y normal 3D de cada texel (H, W, 3) + máscara de texels dentro de las islas UV."""
    tex = tmesh.bake_vertex_attr_textures(size, {"positions", "normals"}, margin=0.0, fill=np.nan,
                                          update_material=False)
    P = tex["positions"].numpy().astype(np.float64)
    N = tex["normals"].numpy().astype(np.float64)
    inside = np.isfinite(P).all(-1)
    return P, N, inside


def _render_depth(scene, view, w, h):
    """Profundidad z (m) de la malla vista desde la cámara (inf = nada)."""
    K, T = view.K, view.T
    uu, vv = np.meshgrid(np.arange(w, dtype=np.float64), np.arange(h, dtype=np.float64))
    d = np.stack([(uu - K[0, 2]) / K[0, 0], (vv - K[1, 2]) / K[1, 1], np.ones_like(uu)], -1)  # z = 1
    dirs = d @ T[:3, :3].T
    orig = np.broadcast_to(T[:3, 3], dirs.shape)
    rays = o3d.core.Tensor(np.concatenate([orig, dirs], -1).astype(np.float32))
    t = scene.cast_rays(rays)["t_hit"].numpy()      # paramétrico: con |d_z| = 1 en cámara, t = z
    return t.astype(np.float64)


def _edge_weight(mask, ramp_px):
    """0 en el borde de la silueta -> 1 a `ramp_px` píxeles hacia dentro."""
    m = mask.astype(np.uint8)
    dist = cv2.distanceTransform(m, cv2.DIST_L2, 5)
    return np.clip((dist - 1.0) / ramp_px, 0.0, 1.0)


def _sample(img, u, v):
    """Muestreo bilineal de img (H, W, C float32) en coordenadas (u, v) de píxel."""
    n = u.size
    cols = 4096
    rows = (n + cols - 1) // cols
    mu = np.full(rows * cols, -1, np.float32)
    mv = np.full(rows * cols, -1, np.float32)
    mu[:n], mv[:n] = u, v
    out = cv2.remap(img, mu.reshape(rows, cols), mv.reshape(rows, cols), cv2.INTER_LINEAR,
                    borderMode=cv2.BORDER_REPLICATE)
    return out.reshape(rows * cols, -1)[:n]


def _push_pull(img, known):
    """Rellena los texels sin dato con la media de sus vecinos a escalas cada vez más gruesas."""
    levels = [(img * known[..., None], known.astype(np.float32))]
    while min(levels[-1][1].shape) > 4:
        c, w = levels[-1]
        h2, w2 = c.shape[0] // 2, c.shape[1] // 2
        c2 = c[:h2 * 2, :w2 * 2].reshape(h2, 2, w2, 2, -1).sum((1, 3))
        k2 = w[:h2 * 2, :w2 * 2].reshape(h2, 2, w2, 2).sum((1, 3))
        levels.append((c2, k2))
    c, w = levels[-1]
    filled = c / np.maximum(w, 1e-6)[..., None]
    for c, w in reversed(levels[:-1]):
        up = cv2.resize(filled, (c.shape[1], c.shape[0]), interpolation=cv2.INTER_LINEAR)
        if up.ndim == 2:
            up = up[..., None]
        own = c / np.maximum(w, 1e-6)[..., None]
        a = np.clip(w, 0, 1)[..., None]
        filled = own * a + up * (1 - a)
    return filled


def _blend(Wn, S, gains):
    """Media ponderada de las vistas con su ganancia, vista a vista (sin el array float V×M×3)."""
    acc = np.zeros(S.shape[1:], np.float32)
    for k in range(len(S)):
        w = Wn[k]
        nz = w > 0
        acc[nz] += w[nz, None] * (S[k][nz].astype(np.float32) * gains[k])
    return acc / np.maximum(Wn.sum(0), 1e-12)[:, None]


def bake(mesh, views, size=2048, max_triangles=80000, edge_ramp_px=10.0, log=None):
    """Malla (legacy, marco de las poses) + vistas -> (malla con UV por triángulo, textura RGB uint8, info).

    La malla devuelta es la preparada (decimada y manifold) con `triangle_uvs`, la textura y
    colores por vértice sacados de la propia textura (para quien no lea texturas).
    """
    log = log or (lambda *_: None)
    m = _prepare(mesh, max_triangles)
    tm = o3d.t.geometry.TriangleMesh.from_legacy(m)
    # 4 particiones: con una sola, UVAtlas se dispara en mallas con muchos trozos (trípode con 3 500 islas:
    # 8 part. 16 s, 4 part. 24 s, 2 part. 92 s, 1 part. > 13 min con el servidor bloqueado). En objetos
    # sencillos una pieza daba algo menos de islas (caja: 330 frente a 397), no compensa el riesgo.
    try:
        stretch, charts, _ = tm.compute_uvatlas(size=size, gutter=4.0, max_stretch=0.2, parallel_partitions=4)
    except RuntimeError as ex:  # UVAtlas puede fallar con algún vértice no manifold: con más trozos sí sale
        log(f"atlas UV falló ({ex}); reintento con 8 particiones")
        tm = o3d.t.geometry.TriangleMesh.from_legacy(m)
        stretch, charts, _ = tm.compute_uvatlas(size=size, gutter=4.0, max_stretch=0.2, parallel_partitions=8)
    log(f"atlas UV: {charts} islas, estiramiento {stretch:.3f}")
    P, N, inside = _texel_map(tm, size)
    idx = np.flatnonzero(inside)
    X = P.reshape(-1, 3)[idx]
    Nn = N.reshape(-1, 3)[idx]
    Nn /= np.maximum(np.linalg.norm(Nn, axis=1, keepdims=True), 1e-9)
    # islas del atlas (separadas por el margen): cada una, un trozo continuo de superficie
    n_isl, isl = cv2.connectedComponents(inside.astype(np.uint8), connectivity=8)
    isl_of = isl.reshape(-1)[idx]

    scene = o3d.t.geometry.RaycastingScene()
    scene.add_triangles(o3d.t.geometry.TriangleMesh.from_legacy(m))

    def weigh(v, pts, nrm, zbuf, edge_map, w, h):
        """Peso de cada punto en la vista v (0 = no la ve) y sus coordenadas de píxel."""
        Tinv = np.linalg.inv(v.T)
        Xc = pts @ Tinv[:3, :3].T + Tinv[:3, 3]
        z = Xc[:, 2]
        ok = z > 0.05
        zs = np.where(ok, z, 1)
        u = np.where(ok, v.K[0, 0] * Xc[:, 0] / zs + v.K[0, 2], -1)
        vv = np.where(ok, v.K[1, 1] * Xc[:, 1] / zs + v.K[1, 2], -1)
        ok &= (u >= 1) & (u < w - 2) & (vv >= 1) & (vv < h - 2)
        ui, vi = np.clip(np.round(u).astype(int), 0, w - 1), np.clip(np.round(vv).astype(int), 0, h - 1)
        ok &= z <= zbuf[vi, ui] + 0.002 + 0.004 * z          # nada delante (oclusión)
        ray = v.T[:3, 3] - pts
        ray /= np.maximum(np.linalg.norm(ray, axis=1, keepdims=True), 1e-9)
        cos = np.clip(np.sum(ray * nrm, axis=1), 0, 1)
        wgt = np.where(ok & (cos >= 0.15), (cos ** 2) * edge_map[vi, ui] * (0.5 / np.maximum(z, 0.1)) ** 2, 0.0)
        return wgt, u, vv

    samples, weights = [], []
    for k, v in enumerate(views):
        h, w = v.color.shape[:2]
        zbuf = _render_depth(scene, v, w, h)
        # silueta: la de la malla en esta vista y, si hay, la máscara del objeto (profundidad)
        sil = np.isfinite(zbuf)
        if v.mask is not None:
            sil &= cv2.dilate(v.mask.astype(np.uint8), np.ones((3, 3), np.uint8)) > 0
        edge_map = _edge_weight(sil, edge_ramp_px)
        wgt, u, vv = weigh(v, X, Nn, zbuf, edge_map, w, h)
        col = _sample(v.color.astype(np.float32), u, vv)
        wgt *= np.where(col.max(1) > 250, 0.15, 1.0)            # brillo especular: casi no cuenta
        samples.append(np.clip(np.round(col), 0, 255).astype(np.uint8))   # 1 byte por canal: 36 vistas HD caben
        weights.append(wgt.astype(np.float32))
        log(f"vista {k + 1}/{len(views)}: {int((wgt > 0).sum())} texels")

    S = np.stack(samples)                         # (V, M, 3) uint8
    W = np.stack(weights)                         # (V, M)
    best = W.max(0, keepdims=True)
    # UNA vista por isla del atlas: la que mejor ve la isla en conjunto. Mezclar vistas con la
    # geometría desviada 2-3 mm (superficies negras o brillantes en la D455) duplica letras y
    # detalles; una sola por zona los deja nítidos y las costuras caen en los bordes de las islas.
    score = np.stack([np.bincount(isl_of, weights=W[k], minlength=n_isl) for k in range(len(views))])
    label = np.where(score.max(0) > 0, score.argmax(0), -1)
    lab_tex = label[isl_of]
    # la vista elegida manda; las demás solo rellenan lo que ella no ve en ese texel
    Wn = np.where(np.arange(len(views))[:, None] == lab_tex[None, :], W, 0.01 * W ** 2 / np.maximum(best, 1e-12))
    Wn = np.where(best > 0, Wn, 0)

    gains = np.ones((len(views), 3), np.float32)
    for _ in range(2):                            # compensación de exposición por vista
        blend = _blend(Wn, S, gains)
        for k in range(len(views)):
            sel = W[k] > 0.3 * best[0]
            if sel.sum() < 500:
                continue
            ratio = np.median(blend[sel] + 1, axis=0) / np.median(S[k][sel].astype(np.float32) + 1, axis=0)
            gains[k] = np.clip(ratio, 0.6, 1.6)
    blend = _blend(Wn, S, gains)
    seen = Wn.sum(0) > 0
    used = np.unique(label[label >= 0])
    log(f"vistas usadas en la textura: {len(used)}/{len(views)}")

    tex = np.zeros((size * size, 3), np.float32)
    known = np.zeros(size * size, bool)
    tex[idx[seen]] = blend[seen]
    known[idx[seen]] = True
    filled = _push_pull(tex.reshape(size, size, 3), known.reshape(size, size))
    texture = np.clip(filled, 0, 255).astype(np.uint8)

    # UV por triángulo (convención Open3D: v hacia arriba en la imagen -> fila = (1 - v) * size)
    uvs = tm.triangle["texture_uvs"].numpy().reshape(-1, 2).astype(np.float64)
    m.triangle_uvs = o3d.utility.Vector2dVector(uvs)
    m.triangle_material_ids = o3d.utility.IntVector(np.zeros(len(m.triangles), np.int32))
    # color por vértice desde la textura (miniaturas, PLY, partículas)
    tri = np.asarray(m.triangles).reshape(-1)
    px = np.clip(uvs[:, 0] * size, 0, size - 1)
    py = np.clip((1 - uvs[:, 1]) * size, 0, size - 1)
    vc = _sample(texture.astype(np.float32), px, py)
    acc = np.zeros((len(m.vertices), 3))
    cnt = np.zeros(len(m.vertices))
    np.add.at(acc, tri, vc)
    np.add.at(cnt, tri, 1)
    m.vertex_colors = o3d.utility.Vector3dVector(acc / np.maximum(cnt, 1)[:, None] / 255.0)
    info = {"texels_vistos": round(float(seen.mean()), 3), "islas_uv": int(charts), "vistas_usadas": int(len(used)),
            "ganancias": [[round(float(g), 3) for g in gk] for gk in gains], "tamano": size}
    return m, texture, info


def split_by_uv(mesh):
    """Malla con UV por triángulo -> vértices duplicados en las costuras (UV por vértice, para Unity).
    Devuelve (V, N, C, UV, F) en arrays numpy."""
    F = np.asarray(mesh.triangles).reshape(-1)
    uv = np.asarray(mesh.triangle_uvs)
    key = np.column_stack([F, np.round(uv * 1e6).astype(np.int64)])
    uniq, first, inv = np.unique(key, axis=0, return_index=True, return_inverse=True)
    src = F[first]
    V = np.asarray(mesh.vertices)[src]
    N = np.asarray(mesh.vertex_normals)[src] if mesh.has_vertex_normals() else None
    C = np.asarray(mesh.vertex_colors)[src] if mesh.has_vertex_colors() else None
    return V, N, C, uv[first], inv.reshape(-1, 3)
