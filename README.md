# RealSenseScans

### Escáner 3D con dos Intel RealSense, streaming en vivo, calibración dual y paso a Unity VR

[![Python](https://img.shields.io/badge/Python-3.10%20%7C%203.11%20%7C%203.12-blue.svg)](https://www.python.org/)
[![Intel RealSense](https://img.shields.io/badge/Intel%20RealSense-D400-informational.svg)](https://github.com/IntelRealSense/librealsense)
[![Open3D](https://img.shields.io/badge/Open3D-0.18%2B-orange.svg)](http://www.open3d.org/)
[![License: MIT](https://img.shields.io/badge/License-MIT-green.svg)](LICENSE)

Sistema para capturar objetos y espacios con una o dos cámaras de profundidad Intel RealSense (probado con dos
D455), verlos en 3D en el navegador en tiempo real, calibrar las cámaras entre sí y llevar el resultado a Blender
y a una galería en realidad virtual hecha en Unity
([VR ScanWorld](https://github.com/Wami2L/Unity_Realsense_Reciever)). Incluye además herramientas para preparar
escenas de *Gaussian Splatting* (3DGS) como entornos VR.

Todo funciona también **sin cámaras**, con escenas sintéticas (`--synthetic`, `--synthetic-object`).

---

## Características

- **Escáner de objetos → Blender → Unity** (`http://localhost:5002/objetos`)
  - Hoja imprimible con marcadores ArUco (`imprimibles/base_escaneo_A3.pdf`) que da la pose y la escala real
    de cada toma.
  - Encuadre guiado: el objeto se resalta y el resto se ignora; avisos según el alcance de la D455.
  - Malla limpia, a escala, de pie y con pivote en la base, texturizada con las fotos de las tomas.
  - Con dos cámaras corrige sola la calibración entre ellas.
  - Ajuste opcional en Blender (addon: abrir, limpieza rápida, *Publicar en Unity*).
  - Unity recibe los objetos por un enlace local (puerto 5010) y los construye con partículas.
- **Live 3D**: servidor `aiohttp` + WebSockets con protocolo binario propio y visor Three.js; una cámara o las
  dos fusionadas con su calibración.
- **Estudio de calibración dual** (`http://localhost:5005`): alineación SIFT 3D + RANSAC + ICP robusto,
  marcadores ArUco, puntos a mano, mapas de error e historial de calibraciones con deshacer.
- **Escenas 3DGS** (`tools/splats.py`): limpieza, poda, LOD, colisiones y exportación a Unity de escenas
  propias (vídeo → COLMAP → entrenamiento → `preparar`), más visitas narradas definidas en `experiencias/*.json`.
- **Blender**: addon `RealSense_Importer.py` (Blender ≥ 4.0) que importa los escaneos con sus ejes, cámaras y
  color.

---

## Inicio rápido

### Windows

1. `instalar_dependencias.bat` — crea `.venv` e instala `requirements.txt`.
2. `iniciar_sistema.bat` — levanta los tres servidores y abre el escáner de objetos.

| Servicio | URL |
|---|---|
| Escáner de objetos | http://localhost:5002/objetos |
| Visor Live 3D | http://localhost:5002 |
| Estudio de calibración | http://localhost:5005 |
| Enlace con Unity | http://127.0.0.1:5010 (solo este equipo; `--red` con token para otros) |

### Linux / macOS

```bash
bash instalar_dependencias.sh
bash iniciar_sistema.sh
```

### Sin cámaras

```bash
python app.py --synthetic --dual      # Live 3D con dos cámaras simuladas
python app.py --synthetic-object      # escáner de objetos con plato giratorio simulado
```

Si arranca en simulación por falta de cámaras, cambia solo a las reales al conectarlas.

---

## Comandos útiles

```bash
python tools/generar_base_escaneo.py --hoja A3                 # hoja imprimible de la base de escaneo
python tools/reprocesar_objeto.py Saves/<fecha>/objeto_HHMMSS  # rehacer un objeto sin cámara
python tools/splats.py preparar <escena.ply> --nombre X --colmap <sparse/0> --unity <proyecto>
python -m pytest                                               # tests (lanzan su propio servidor sintético)
python tools/bench_live3d.py --spawn --capture                 # benchmark sintético
```

`tools/destilar.py` necesita un entorno aparte con PyTorch y gsplat (ver el plan 15, §12.6).

---

## Estructura

```text
RealSenseScans/
├── app.py                     # Servidor principal (Live 3D + escáner de objetos, puerto 5002)
├── live3d/
│   ├── server.py              # aiohttp: /ws, /api/*, visor y /objetos
│   ├── capture.py             # Fuentes RealSense (1 o 2 cámaras) y sintéticas
│   ├── protocol.py            # Protocolo binario RSF1
│   ├── alignment.py           # Motor de alineación Cam 2 → Cam 1
│   ├── registration.py        # Nubes, TSDF dual, convención de ejes
│   ├── objscan/               # Escáner de objetos (base ArUco, segmentación, TSDF, textura, RSM)
│   ├── splats/                # Escenas Gaussian Splatting (PLY, limpieza, LOD, experiencias)
│   ├── unity_link.py          # Enlace con Unity (puerto 5010)
│   └── web/                   # Visor Three.js e interfaz /objetos
├── test_fotogrametria/        # Estudio de calibración dual (puerto 5005)
├── multiscan.py               # Sesión multi-vista (pose graph + Poisson)
├── generar_malla_offline.py   # Malla Delaunay 2.5D de un escaneo simple
├── RealSense_Importer.py      # Addon de Blender
├── script_blender.py          # Abre un objeto escaneado en Blender
├── tools/                     # CLI: base de escaneo, marcadores, splats, benchmark...
├── experiencias/              # Visitas VR narradas (JSON)
├── imprimibles/               # Hojas de marcadores en PDF
├── config/                    # Configuración y calibración de las cámaras
├── documentacion/             # Guías y planes técnicos (MD + PDF)
├── tests/                     # pytest
└── Saves/                     # Escaneos y objetos (local, ignorado por git)
```

---

## Requisitos

- Python 3.10–3.12, 64 bits.
- 1 o 2 cámaras Intel RealSense D400 (probado con D455) en USB 3. Sin cable de sincronía, las dos cámaras
  funcionan en modo independiente.
- Navegador con WebGL.
- Opcional: Blender ≥ 4.0, Unity 6 con el proyecto [VR ScanWorld](https://github.com/Wami2L/Unity_Realsense_Reciever).

`config/dual_camera_config.json` trae los números de serie y la calibración de las cámaras del autor: con
otras cámaras, cambia los seriales y vuelve a calibrar desde el estudio o desde `/objetos`.

---

## Documentación

- [Guía del escáner de objetos](documentacion/GUIA_ESCANER_OBJETOS.md)
- [Instalación y dependencias](documentacion/05_Dependencias_e_Instalacion.md)
- [Malla offline](documentacion/03_Procesamiento_Malla_3D.md) · [Blender](documentacion/04_Integracion_Blender.md)
- [Estudio de calibración](test_fotogrametria/README.md) · [Live 3D](live3d/README.md)
- [Planes técnicos](documentacion/planes/README.md): historia del diseño, del soporte dual a los entornos 3DGS.

---

## Contenido de terceros

- **three.js** (MIT) va incluido en `live3d/web/vendor/` (ver [THIRD_PARTY_NOTICES.md](THIRD_PARTY_NOTICES.md)).
- Las escenas de Gaussian Splatting **no se incluyen**. La visita de `experiencias/iglesia_san_mauricio.json`
  usa el escaneo *Parish Church of Saint Maurice* de Pavel Tanhäuser (superspl.at, **CC BY-NC-ND 4.0**):
  descárgalo de su fuente y no redistribuyas versiones modificadas ni recortes de él.

## Licencia

Código bajo licencia [MIT](LICENSE) © 2026 Wami.
