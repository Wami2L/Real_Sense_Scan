# RealSenseScans 🎥✨
### Dual Intel RealSense Live 3D Streaming, Calibration Studio & 3D Scanning Suite

[![Python](https://img.shields.io/badge/Python-3.10%20%7C%203.11%20%7C%203.12-blue.svg)](https://www.python.org/)
[![Intel RealSense](https://img.shields.io/badge/Intel%20RealSense-D400%20Series-informational.svg)](https://www.intelrealsense.com/)
[![Open3D](https://img.shields.io/badge/Open3D-3D%20Data%20Processing-orange.svg)](http://www.open3d.org/)
[![Three.js](https://img.shields.io/badge/Three.js-WebGL%20Stream-black.svg)](https://threejs.org/)

Sistema integral para captura RGB-D, transmisión de nubes de puntos 3D en tiempo real vía WebSockets/WebGL (Three.js), estudio de calibración extrínseca multicámara interactivo y reconstrucción de mallas 3D con integración en Blender.

---

## 🌟 Características Principales

- **Streaming 3D en Vivo (Live3D):**
  - Servidor asíncrono de alto rendimiento (`aiohttp` + WebSockets).
  - Visor WebGL interactivo con Three.js y shaders personalizados para nubes de puntos de ultra baja latencia.
  - Soporte de renderizado individual o fusión dual calibrada en vivo.
  - Modo simulación sintética (`--synthetic`) para pruebas sin necesidad de cámaras físicas conectadas.

- **Estudio de Calibración y Fotogrametría Dual:**
  - Detección de correspondencias de características 2D (SIFT / ORB / FLANN).
  - Geometría epipolar, matrices fundamentales y homografías.
  - Estimación de transformación rígida 3D (Kabsch / Umeyama) refinada mediante ICP (*Iterative Closest Point*).
  - Panel web de inspección visual en tiempo real: análisis epipolar, superposición anaglífica (3D) y alineación de profundidad.

- **Pipeline de Escaneo 3D y Generación de Malla:**
  - Captura sincronizada de nube de puntos (`.ply`), mapa de profundidad y fotografías RGB.
  - Generación offline de malla sólida con triangulación de Delaunay 2.5D y filtrado de aristas.
  - Mapeo UV y texturizado fotográfico automático exportado en formato `.obj`.

- **Integración con Blender:**
  - Add-on para Blender (`RealSense_Importer.py`) con instanciación de Geometry Nodes para visualización de nubes de puntos en Cycles y Eevee.
  - Script independiente (`script_blender.py`) para carga directa del escaneo más reciente.

---

## 🚀 Inicio Rápido

### En Windows (1 Clic)

1. **Instalar dependencias:**  
   Ejecuta haciendo doble clic en `instalar_dependencias.bat` (o desde PowerShell/CMD).
   > Crea automáticamente un entorno virtual aislado (`.venv`) e instala `pyrealsense2`, `open3d`, `opencv-python`, etc.

2. **Iniciar el sistema:**  
   Ejecuta haciendo doble clic en `iniciar_sistema.bat`.
   > Levanta el servidor Live 3D (puerto 5002) y el Estudio de Calibración (puerto 5005), abriendo el navegador automáticamente.

### En Linux / macOS

```bash
# 1. Instalar dependencias
bash instalar_dependencias.sh

# 2. Iniciar el sistema
bash iniciar_sistema.sh
```

### Ejecución Manual y Modo Simulación

Si no tienes las cámaras RealSense conectadas físicamente o deseas probar el visor en una laptop:

```bash
# Activar entorno virtual
.\.venv\Scripts\activate   # En Windows
source .venv/bin/activate  # En Linux/macOS

# Modo simulación sintética con cámara dual
python app.py --synthetic --dual

# Iniciar estudio de calibración y fotogrametría
python test_fotogrametria/servidor_test.py
```

---

## 📁 Estructura del Proyecto

```text
RealSenseScans/
├── app.py                         # Lanzador unificado del sistema
├── camera_manager.py              # Administrador de dispositivos RealSense
├── config/
│   └── dual_camera_config.json    # Parámetros extrínsecos y configuración dual
├── documentacion/                 # Documentación técnica completa y planes (PDF y MD)
│   ├── Documentacion_RealSenseScans.md
│   ├── GUIA_EMPAQUETADO_Y_GITHUB.md
│   └── planes/                    # Arquitectura, fotogrametría y calibración
├── live3d/                        # Módulo de streaming 3D en tiempo real
│   ├── capture.py                 # Captura asíncrona RGB-D
│   ├── protocol.py                # Protocolo binario de empaquetado de puntos
│   ├── server.py                  # Servidor aiohttp WebSocket
│   └── web/                       # Visor Three.js con shaders custom
├── test_fotogrametria/            # Estudio de calibración multicámara
│   ├── calibrador.py              # Algoritmos SIFT, epipolar y alineación ICP
│   ├── servidor_test.py           # Backend Flask del estudio
│   └── web/                       # Interfaz visual de diagnóstico y captura
├── multiscan.py                   # Motor de escaneo multicámara
├── generar_malla_offline.py       # Reconstrucción de malla Delaunay 2.5D
├── RealSense_Importer.py          # Add-on para Blender (Geometry Nodes)
├── script_blender.py              # Script standalone para Blender
├── Saves/                         # Directorio local de escaneos (ignorado en Git)
├── requirements.txt               # Dependencias de Python
└── .gitignore                     # Filtros para mantener el repositorio limpio
```

---

## ⚙️ Requisitos del Sistema

- **Python:** 3.10, 3.11 o 3.12 (64-bit recomendado).
- **Hardware:**
  - 1 o 2 cámaras Intel RealSense D400 (D435, D435i, D455) conectadas a puertos USB 3.0.
  - Tarjeta gráfica con soporte WebGL para el visor 3D en el navegador.
- **Blender (Opcional):** Versión 3.0 o superior para el add-on de visualización de nubes de puntos.

---

## 📖 Documentación Adicional

Para más detalles sobre la arquitectura interna, planes de mejora y calibración avanzada, consulta la carpeta `documentacion/`:
- `documentacion/Documentacion_RealSenseScans.md`: Detalles técnicos de los módulos.
- `documentacion/GUIA_EMPAQUETADO_Y_GITHUB.md`: Guía de portabilidad y empaquetado.
- `documentacion/planes/`: Informes detallados sobre captura dual y fusión en vivo.

---

## 📄 Licencia

Este proyecto está disponible para propósitos de investigación, desarrollo e ingeniería en visión por computadora.
