"""
Destilación de una estación (plan 15 §11.3-D y §12.6): optimiza un LOD de pocos splats (tools/splats.py lod) para
que, visto desde la zona de movimiento de la estación, se parezca lo más posible a la escena completa.

El maestro es la escena limpia (tools/splats.py limpiar) renderizada con gsplat desde vistas aleatorias dentro de la
estación; el alumno parte del LOD y se optimiza (posición, forma, opacidad, color/SH) con L1 + SSIM, sin cambiar el
número de splats. Se ejecuta en el entorno aparte con PyTorch + gsplat (CUDA):

    .venv-splats\\Scripts\\python tools/destilar.py Saves/entornos/escena1/lod_nave_2.ply \\
        --maestro Saves/entornos/escena1/limpio_sh1.ply

Escribe <alumno>_dest.ply (+ .json) en el mismo marco que el LOD (el banco de Unity lo recoloca igual).
"""

import argparse
import json
import math
import os
import sys
import time

import numpy as np
import torch
import torch.nn.functional as F
from gsplat import rasterization

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from live3d.splats.ply import SplatPly, write_columns  # noqa: E402

DEV = "cuda"


# ---------------------------------------------------------------------------------------------- datos

def load_splats(path, sh_degree, device=DEV):
    """PLY 3DGS -> dict de tensores (medias, cuaterniones, log-escalas, logit-opacidad, sh0 (N,1,3), shN (N,K,3))."""
    ply = SplatPly.open(path)
    per = ply.sh_rest // 3
    k = {0: 0, 1: 3, 2: 8, 3: 15}[min(sh_degree, ply.sh_degree)]
    names = ["x", "y", "z", "rot_0", "rot_1", "rot_2", "rot_3", "scale_0", "scale_1", "scale_2", "opacity",
             "f_dc_0", "f_dc_1", "f_dc_2"] + [f"f_rest_{c * per + j}" for c in range(3) for j in range(k)]
    a = torch.from_numpy(ply.columns(names))
    sh_n = a[:, 14:].reshape(-1, 3, k).transpose(1, 2) if k else torch.zeros(len(a), 0, 3)
    return {
        "means": a[:, 0:3].to(device), "quats": a[:, 3:7].to(device), "scales": a[:, 7:10].to(device),
        "opacities": a[:, 10].to(device), "sh0": a[:, 11:14].reshape(-1, 1, 3).to(device),
        "shN": sh_n.contiguous().to(device),
    }, min(sh_degree, ply.sh_degree)


def render(s, viewmats, K, w, h, sh_degree, with_meta=False):
    colors = torch.cat([s["sh0"], s["shN"]], 1)
    img, _, meta = rasterization(
        s["means"], F.normalize(s["quats"], dim=-1), torch.exp(s["scales"]), torch.sigmoid(s["opacities"]),
        colors, viewmats, K, w, h, sh_degree=sh_degree, packed=True)     # fondo negro por defecto
    img = img.clamp(0, 1)
    return (img, meta) if with_meta else img


def tile_pairs(meta):
    """Pares splat-tesela (16×16 px) que rasteriza gsplat: indicador del overdraw (coste de fragmentos)."""
    return int(meta["tiles_per_gauss"].sum())


# ---------------------------------------------------------------------------------------------- cámaras

def look_dirs(yaw, pitch):
    """Dirección de vista en RDF (y abajo): yaw alrededor del eje vertical, pitch positivo hacia arriba."""
    return np.stack([np.cos(pitch) * np.sin(yaw), -np.sin(pitch), np.cos(pitch) * np.cos(yaw)], -1)


def viewmat(pos, fwd):
    """Mundo -> cámara OpenCV (x derecha, y abajo, z adelante) en un mundo RDF con arriba = -y."""
    up = np.array([0.0, -1.0, 0.0])
    r = np.cross(fwd, up)
    r /= np.linalg.norm(r)
    d = np.cross(fwd, r)
    R = np.stack([r, d, fwd], 1)                     # columnas: ejes de la cámara en el mundo
    m = np.eye(4)
    m[:3, :3] = R.T
    m[:3, 3] = -R.T @ pos
    return m


def station_cameras(center, n, radius, rng, heights=(1.2, 1.9)):
    """n poses aleatorias dentro de la estación: 65 % cerca del horizonte, 25 % hacia la bóveda, 10 % al suelo."""
    ang = rng.uniform(0, 2 * np.pi, n)
    rad = radius * np.sqrt(rng.uniform(0, 1, n))
    hgt = rng.uniform(*heights, n)
    pos = center + np.stack([rad * np.sin(ang), -hgt, rad * np.cos(ang)], 1)
    kind = rng.uniform(0, 1, n)
    pitch = np.where(kind < 0.65, rng.uniform(-25, 25, n),
                     np.where(kind < 0.9, rng.uniform(25, 80, n), rng.uniform(-70, -25, n)))
    yaw = rng.uniform(0, 2 * np.pi, n)
    fwd = look_dirs(yaw, np.radians(pitch))
    return np.stack([viewmat(p, f) for p, f in zip(pos, fwd)])


# ---------------------------------------------------------------------------------------------- pérdida

def ssim(x, y, win=11, sigma=1.5):
    """SSIM medio de imágenes (B,H,W,3) en [0,1] con ventana gaussiana."""
    x, y = x.permute(0, 3, 1, 2), y.permute(0, 3, 1, 2)
    g = torch.exp(-(torch.arange(win, device=x.device) - win // 2) ** 2 / (2 * sigma ** 2))
    g = (g / g.sum()).float()
    k = (g[:, None] * g[None, :]).expand(3, 1, win, win).contiguous()

    def blur(t):
        return F.conv2d(t, k, padding=win // 2, groups=3)

    mx, my = blur(x), blur(y)
    vx, vy, cxy = blur(x * x) - mx * mx, blur(y * y) - my * my, blur(x * y) - mx * my
    c1, c2 = 0.01 ** 2, 0.03 ** 2
    return (((2 * mx * my + c1) * (2 * cxy + c2)) / ((mx * mx + my * my + c1) * (vx + vy + c2))).mean()


def psnr(x, y):
    return float(-10 * torch.log10(F.mse_loss(x, y)))


# ---------------------------------------------------------------------------------------------- main

def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("alumno", help="LOD de la estación (tools/splats.py lod), con su .json")
    ap.add_argument("--maestro", required=True, help="escena limpia en el mismo marco (tools/splats.py limpiar)")
    ap.add_argument("--vistas", type=int, default=400, help="vistas del maestro para entrenar")
    ap.add_argument("--validacion", type=int, default=24, help="vistas apartadas para medir")
    ap.add_argument("--res", type=int, default=512, help="lado de las vistas (px), FOV 90°")
    ap.add_argument("--iters", type=int, default=4000)
    ap.add_argument("--lote", type=int, default=2, help="vistas por iteración")
    ap.add_argument("--sh", type=int, default=1)
    ap.add_argument("--cap-escala", type=float, default=1.5,
                    help="ningún eje de un splat puede superar este factor × su eje mayor inicial (0 = sin límite): "
                         "acota el overdraw")
    ap.add_argument("--semilla", type=int, default=0)
    ap.add_argument("--sufijo", default="dest")
    a = ap.parse_args()
    if hasattr(sys.stdout, "reconfigure"):
        sys.stdout.reconfigure(encoding="utf-8", errors="replace")
    t0 = time.time()
    torch.manual_seed(a.semilla)
    rng = np.random.default_rng(a.semilla)

    with open(a.alumno[:-4] + ".json", encoding="utf-8") as f:
        man = json.load(f)
    t = np.array(man["traslacion_restada"])
    ux, uz = man["lod"]["centro_unity_xz"]
    radius = man["lod"]["radio"]
    center = np.array([ux - t[0], 0.0, uz - t[2]])               # suelo de la estación en el marco del archivo

    w = h = a.res
    f = (a.res / 2) / math.tan(math.radians(45))
    K1 = torch.tensor([[f, 0, w / 2], [0, f, h / 2], [0, 0, 1]], dtype=torch.float32, device=DEV)
    n_all = a.vistas + a.validacion
    vms = torch.tensor(station_cameras(center, n_all, radius, rng), dtype=torch.float32, device=DEV)

    # Maestro: render de todas las vistas y fuera de la GPU.
    teacher, sh_t = load_splats(a.maestro, a.sh)
    print(f"Maestro: {len(teacher['means']):,} splats SH{sh_t} ({time.time() - t0:.0f} s)")
    targets = torch.empty(n_all, h, w, 3, dtype=torch.uint8)
    with torch.no_grad():
        for i in range(n_all):                                   # de una en una: 9,8 M splats en 4 GB de VRAM
            img = render(teacher, vms[i:i + 1], K1[None], w, h, sh_t)
            targets[i] = (img[0] * 255 + 0.5).byte().cpu()
    del teacher
    torch.cuda.empty_cache()
    print(f"{n_all} vistas del maestro a {w}×{h} ({time.time() - t0:.0f} s)")

    student, sh_s = load_splats(a.alumno, a.sh)
    n = len(student["means"])
    params = {k: torch.nn.Parameter(v.clone()) for k, v in student.items()}
    lr = {"means": 4e-4, "quats": 1e-3, "scales": 5e-3, "opacities": 5e-2, "sh0": 2.5e-3, "shN": 2.5e-3 / 20}
    opt = torch.optim.Adam([{"params": [params[k]], "lr": lr[k], "name": k} for k in params], eps=1e-15)
    decay = 0.01 ** (1 / a.iters)                                # medias: lr × 0,01 al final

    # Límite de tamaño: cada eje <= cap × eje mayor inicial (en log: escalas <= max0 + log cap).
    scale_cap = None
    if a.cap_escala > 0:
        scale_cap = (params["scales"].detach().max(dim=1, keepdim=True).values + math.log(a.cap_escala))

    val = list(range(a.vistas, n_all))

    def evaluate(s):
        with torch.no_grad():
            ps, pairs = [], []
            for i in val:
                img, meta = render(s, vms[i:i + 1], K1[None], w, h, sh_s, with_meta=True)
                ps.append(psnr(img, targets[i:i + 1].to(DEV).float() / 255))
                pairs.append(tile_pairs(meta))
            return float(np.mean(ps)), float(np.mean(pairs))

    p0, pairs0 = evaluate(params)
    print(f"Alumno: {n:,} splats SH{sh_s} · PSNR de validación inicial {p0:.2f} dB · "
          f"{pairs0 / 1e6:.2f} M pares splat-tesela por vista")
    for it in range(1, a.iters + 1):
        idx = torch.randint(0, a.vistas, (a.lote,))
        gt = targets[idx].to(DEV, non_blocking=True).float() / 255
        img = render(params, vms[idx.to(DEV)], K1.expand(a.lote, 3, 3), w, h, sh_s)
        loss = 0.8 * F.l1_loss(img, gt) + 0.2 * (1 - ssim(img, gt))
        opt.zero_grad(set_to_none=True)
        loss.backward()
        opt.step()
        if scale_cap is not None:
            with torch.no_grad():
                params["scales"].copy_(torch.minimum(params["scales"], scale_cap))
        for g in opt.param_groups:
            if g["name"] == "means":
                g["lr"] *= decay
        if it % 1000 == 0 or it == a.iters:
            pv, pr = evaluate(params)
            print(f"  iter {it}: pérdida {loss.item():.4f} · PSNR val {pv:.2f} dB · "
                  f"pares ×{pr / pairs0:.2f} ({time.time() - t0:.0f} s)")

    p1, pairs1 = evaluate(params)
    # Escribir en el mismo marco y formato (f_rest por canal, como el original).
    s = {k: v.detach().float().cpu().numpy() for k, v in params.items()}
    q = s["quats"] / np.linalg.norm(s["quats"], axis=1, keepdims=True)
    cols = {"x": s["means"][:, 0], "y": s["means"][:, 1], "z": s["means"][:, 2]}
    for c in range(3):
        cols[f"f_dc_{c}"] = s["sh0"][:, 0, c]
    k = s["shN"].shape[1]
    for c in range(3):
        for j in range(k):
            cols[f"f_rest_{c * k + j}"] = s["shN"][:, j, c]
    cols["opacity"] = s["opacities"]
    for i in range(3):
        cols[f"scale_{i}"] = s["scales"][:, i]
    for i in range(4):
        cols[f"rot_{i}"] = q[:, i]
    out = a.alumno[:-4] + f"_{a.sufijo}.ply"
    write_columns(out, {kk: np.asarray(v, np.float32) for kk, v in cols.items()},
                  comments=[f"destilado desde {os.path.basename(a.maestro)}: {a.iters} iteraciones, "
                            f"{a.vistas} vistas {w}x{h}"])
    man_out = dict(man, salida=out, partes=[out], destilacion={
        "maestro": a.maestro, "alumno": a.alumno, "vistas": a.vistas, "validacion": a.validacion, "res": a.res,
        "iters": a.iters, "cap_escala": a.cap_escala, "psnr_val_inicial": p0, "psnr_val_final": p1,
        "pares_tesela_inicial": pairs0, "pares_tesela_final": pairs1, "segundos": round(time.time() - t0)})
    with open(out[:-4] + ".json", "w", encoding="utf-8") as f:
        json.dump(man_out, f, indent=2, ensure_ascii=False)
    print(f"PSNR de validación {p0:.2f} → {p1:.2f} dB · pares splat-tesela ×{pairs1 / pairs0:.2f} · {out} "
          f"({time.time() - t0:.0f} s)")
    return 0


if __name__ == "__main__":
    sys.exit(main())
