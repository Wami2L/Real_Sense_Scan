"""
Experiencia de visita de un entorno 3DGS (rama ExperienceDemo): puntos de interés con ficha y narración, y objetos
recortados de los propios splats para verlos sueltos en la sala de inspección de Unity.

La definición es un JSON versionado (experiencias/<nombre>.json) con los textos y, por objeto, una caja en ejes de
Unity del PLY limpio (x, -y, z del archivo RDF: el mismo marco local que las pirámides y colision.json). Esto genera,
en Assets/ScanWorld/Entornos/<nombre>/experiencia/ (ignorado en git):
    <id>.ply   splats del objeto con el pivote en el centro de su base (importar en gsplat como RDF)
    <id>.wav   narración con una voz de Windows (System.Speech, sin conexión)
    experiencia.json   la definición resuelta (cajas finales, pivotes, audios, duraciones) que lee Unity
Toda la lógica de splats está aquí; en Unity solo se colocan y se muestran (Experiencia.cs).
"""

import json
import os
import subprocess
import tempfile
import wave
from dataclasses import dataclass

import numpy as np

from .analisis import neighbourhood_counts
from .ply import SplatPly, sigmoid, write_ply

_RDF = np.array([1.0, -1.0, 1.0], np.float32)       # archivo RDF <-> ejes de Unity (involutivo)


@dataclass
class Opciones:
    opacidad_min: float = 0.02      # fuera los casi transparentes (en la sala se ven contra un fondo liso)
    vecinos_min: int = 3            # fuera los sueltos (en el cubo 3×3×3 de vóxeles)
    voxel: float = 0.08             # m, para contar vecinos (los splats de zonas poco vistas son grandes y ralos)
    margen: float = 0.0             # m que se añaden a cada lado de la caja
    max_quest: int = 150_000        # splats de la variante de Quest 2 (<id>_quest.ply) si el recorte pasa de esto
    gigante: float = 0.04           # fuera los splats con σ mayor que esta fracción de la diagonal de la caja: son
                                    # manchas de pared o de fondo que en la escena quedan detrás y en la sala, delante


def _caja(d, margen=0.0):
    lo = np.asarray(d["min"], np.float32) - margen
    hi = np.asarray(d["max"], np.float32) + margen
    if np.any(hi <= lo):
        raise ValueError(f"caja vacía: {d}")
    return lo, hi


def _dentro(pos, caja):
    lo, hi = caja
    return np.all((pos >= lo) & (pos <= hi), axis=1)


@dataclass
class Splats:
    """Lo que hace falta de cada splat para recortar: centro (ejes de Unity), opacidad, σ mayor e importancia
    (opacidad × área de las dos σ mayores)."""
    pos: np.ndarray
    alpha: np.ndarray
    sigma: np.ndarray
    importancia: np.ndarray


def leer_splats(ply: SplatPly) -> Splats:
    pos = ply.columns(["x", "y", "z"]) * _RDF
    alpha = sigmoid(ply.columns(["opacity"])[:, 0])
    ls = np.sort(ply.columns(["scale_0", "scale_1", "scale_2"]), axis=1)
    return Splats(pos, alpha, np.exp(ls[:, 2]), alpha * np.exp(ls[:, 1] + ls[:, 2]))


def mascara_objeto(s: Splats, obj, o: Opciones):
    """Splats del objeto: dentro de la caja, fuera de las cajas de `excluir`, opacos, no gigantes y no aislados."""
    pos = s.pos
    caja = _caja(obj["caja"], obj.get("margen", o.margen))
    diag = float(np.linalg.norm(caja[1] - caja[0]))
    m = (_dentro(pos, caja) & (s.alpha >= obj.get("opacidad_min", o.opacidad_min)) &
         (s.sigma <= obj.get("gigante", o.gigante) * diag))
    for ex in obj.get("excluir", []):
        m &= ~_dentro(pos, _caja(ex))
    idx = np.nonzero(m)[0]
    if len(idx) and o.vecinos_min > 0:
        cnt, _ = neighbourhood_counts(pos[idx], o.voxel, caja[0])
        m[idx[cnt < o.vecinos_min]] = False
    return m


def recortar(ply: SplatPly, s: Splats, obj, out_path, o: Opciones):
    """
    Escribe el PLY del objeto (RDF) con el pivote en el centro de la base de lo recortado y devuelve su ficha:
    pivote (ejes de Unity del entorno), caja ajustada (relativa al pivote) y número de splats. Si pasa de
    `o.max_quest` splats escribe además <id>_quest.ply con los más importantes (en Quest 2 la sala de inspección
    no puede dibujar cientos de miles: plan 15 §13).
    """
    m = mascara_objeto(s, obj, o)
    n = int(m.sum())
    if n == 0:
        raise ValueError(f"{obj.get('id')}: la caja no contiene splats")
    p = s.pos[m]
    lo, hi = np.percentile(p, [0.5, 99.5], axis=0)
    pivote = np.array([(lo[0] + hi[0]) / 2, lo[1], (lo[2] + hi[2]) / 2], np.float32)
    piv_rdf = pivote * _RDF

    def trasladar(cols):
        cols["x"] = cols["x"] - piv_rdf[0]
        cols["y"] = cols["y"] - piv_rdf[1]
        cols["z"] = cols["z"] - piv_rdf[2]
        return cols

    nota = f"recorte {obj.get('id')} de {os.path.basename(ply.path)}; pivote (Unity) {pivote.round(4).tolist()}"
    write_ply(out_path, ply, mask=m, transform=trasladar, comments=[nota])
    ficha = {"pivote": pivote.round(4).tolist(), "min": (lo - pivote).round(4).tolist(),
             "max": (hi - pivote).round(4).tolist(), "splats": n}
    quest = out_path[:-4] + "_quest.ply"
    if n > o.max_quest:
        idx = np.nonzero(m)[0]
        mq = np.zeros_like(m)
        mq[idx[np.argsort(-s.importancia[idx])[:o.max_quest]]] = True
        write_ply(quest, ply, mask=mq, transform=trasladar, comments=[nota + "; variante Quest 2"])
        ficha["ply_quest"] = os.path.basename(quest)
    elif os.path.exists(quest):
        os.remove(quest)
    return ficha


# ---------------------------------------------------------------------------------------------- narración

_PS_TTS = r"""
param([string]$lista, [string]$voz, [int]$ritmo)
Add-Type -AssemblyName System.Speech
$s = New-Object System.Speech.Synthesis.SpeechSynthesizer
if ($voz) { $s.SelectVoice($voz) }
$s.Rate = $ritmo
foreach ($linea in [IO.File]::ReadAllLines($lista, [Text.Encoding]::UTF8)) {
    $p = $linea.Split("`t", 2)
    $s.SetOutputToWaveFile($p[0])
    $s.Speak([IO.File]::ReadAllText($p[1], [Text.Encoding]::UTF8))
}
$s.SetOutputToNull()
$s.Dispose()
"""


def voces():
    """Voces de System.Speech instaladas (nombre)."""
    r = subprocess.run(["powershell", "-NoProfile", "-Command",
                        "Add-Type -AssemblyName System.Speech; (New-Object System.Speech.Synthesis.SpeechSynthesizer)"
                        ".GetInstalledVoices() | % { $_.VoiceInfo.Name + '|' + $_.VoiceInfo.Culture }"],
                       capture_output=True, text=True, check=True)
    return [ln.split("|") for ln in r.stdout.splitlines() if "|" in ln]


def elegir_voz(preferida=None, cultura="es-ES"):
    disponibles = voces()
    if preferida:
        if any(n == preferida for n, _ in disponibles):
            return preferida
        raise SystemExit(f"No está la voz «{preferida}». Hay: {[n for n, _ in disponibles]}")
    for n, c in disponibles:
        if c == cultura:
            return n
    for n, c in disponibles:
        if c.startswith(cultura.split("-")[0]):
            return n
    raise SystemExit(f"No hay ninguna voz {cultura} instalada (Configuración › Hora e idioma › Voz)")


def narrar(textos, voz, ritmo=-1):
    """textos: [(wav_path, texto)]. Una sola llamada a PowerShell para todos (arrancar la voz cuesta)."""
    with tempfile.TemporaryDirectory() as tmp:
        lineas = []
        for i, (wav, texto) in enumerate(textos):
            txt = os.path.join(tmp, f"{i}.txt")
            with open(txt, "w", encoding="utf-8") as f:
                f.write(texto)
            lineas.append(f"{os.path.abspath(wav)}\t{txt}")
        lista = os.path.join(tmp, "lista.txt")
        with open(lista, "w", encoding="utf-8") as f:
            f.write("\n".join(lineas))
        script = os.path.join(tmp, "tts.ps1")
        with open(script, "w", encoding="utf-8-sig") as f:
            f.write(_PS_TTS)
        subprocess.run(["powershell", "-NoProfile", "-ExecutionPolicy", "Bypass", "-File", script,
                        "-lista", lista, "-voz", voz, "-ritmo", str(ritmo)], check=True)


def duracion_wav(path):
    with wave.open(path, "rb") as w:
        return w.getnframes() / float(w.getframerate())


def texto_narracion(punto, pronunciacion=None):
    """
    La narración explícita o, si no hay, el título seguido del texto de la ficha. `pronunciacion` cambia nombres
    propios por cómo debe leerlos la voz española (Kroměříž, Schrattenbach…); la ficha en pantalla no cambia.
    """
    texto = punto.get("narracion") or f"{punto['titulo']}. {punto.get('texto', '')}"
    for escrito, dicho in sorted((pronunciacion or {}).items(), key=lambda kv: -len(kv[0])):
        texto = texto.replace(escrito, dicho)
    return texto


def cargar_definicion(path):
    with open(path, encoding="utf-8") as f:
        d = json.load(f)
    ids = [p["id"] for p in d.get("puntos", [])]
    if len(ids) != len(set(ids)):
        raise ValueError("ids de puntos repetidos")
    for p in d.get("puntos", []):
        if not p["id"].replace("_", "").isalnum():
            raise ValueError(f"id no válido (solo letras, números y _): {p['id']}")
    return d
