# 05. Dependencias, Instalación y Ejecución

## 1. Requisitos previos

- **Python 3.10, 3.11 o 3.12** (64 bits).
- **Cámara(s)**: una o dos Intel RealSense serie D400 (probado con dos D455) en puertos USB 3. Sin cámaras se
  puede usar el modo sintético.
- **Navegador** con WebGL para el visor 3D y el escáner de objetos.
- **Blender ≥ 4.0** (opcional) para el addon `RealSense_Importer.py`.
- **Unity 6** (opcional) con el proyecto [VR ScanWorld](https://github.com/Wami2L/Unity_Realsense_Reciever).

## 2. Librerías (Python)

Todas están en `requirements.txt`:

| Librería | Uso |
|---|---|
| `pyrealsense2` | Acceso a las cámaras: color, profundidad alineada, opciones del sensor. |
| `open3d` | Nubes de puntos, ICP, TSDF, mallas y atlas UV. |
| `opencv-python` | Marcadores ArUco, SIFT, rectificación de la lente, imágenes. |
| `numpy`, `scipy` | Álgebra, Delaunay 2.5D, optimización. |
| `pillow` | Lectura y escritura de imágenes (texturas). |
| `aiohttp` | Servidores web y WebSockets (5002, 5005 y 5010). |
| `markdown`, `xhtml2pdf` | Generar los PDF de la documentación desde los `.md`. |

Para los tests: `requirements-dev.txt` (añade `pytest`). La destilación de splats (`tools/destilar.py`) usa un
entorno aparte con PyTorch y gsplat (plan 15, §12.6).

## 3. Instalación

**Windows:** doble clic en `instalar_dependencias.bat`.

**Linux / macOS:** `bash instalar_dependencias.sh`.

Ambos crean el entorno virtual `.venv` e instalan `requirements.txt`. A mano:

```bash
python -m venv .venv
.venv\Scripts\activate          # Windows
source .venv/bin/activate       # Linux / macOS
pip install -r requirements.txt
```

## 4. Ejecución

`iniciar_sistema.bat` (o `iniciar_sistema.sh`) arranca los tres servidores y abre el escáner de objetos:

| Proceso | Comando | URL |
|---|---|---|
| Live 3D + escáner de objetos | `python app.py` | http://localhost:5002 y http://localhost:5002/objetos |
| Estudio de calibración dual | `python test_fotogrametria/servidor_test.py` | http://localhost:5005 |
| Enlace con Unity | `python -m live3d.unity_link` | http://127.0.0.1:5010 |

Sin cámaras:

```bash
python app.py --synthetic --dual    # dos cámaras simuladas
python app.py --synthetic-object    # escáner de objetos simulado
```

Los escaneos y objetos se guardan en `Saves/`. Para generar la malla de un escaneo simple:
`python generar_malla_offline.py`. Para Blender, ver [04_Integracion_Blender.md](04_Integracion_Blender.md).
