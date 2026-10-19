# Documentación del Proyecto: RealSense 3D Scanner

Este documento detalla la arquitectura, las herramientas, los scripts y los métodos utilizados en la implementación del escáner 3D utilizando la cámara Intel RealSense D455, Python, Flask, Open3D y Blender.

## 1. Arquitectura del Sistema
El proyecto consta de tres componentes principales:
1. **Captura y Escaneo (Backend/Frontend):** Una aplicación web en Flask interactúa con el sensor RealSense D455 para capturar nubes de puntos y datos RGB-D en tiempo real.
2. **Generación de Malla 3D:** Un script offline procesa la nube de puntos capturada (con Open3D y SciPy) proyectando los puntos 2D y aplicando triangulación de Delaunay para construir una malla sólida y asignar texturas.
3. **Integración en Blender:** Un Add-on (`RealSense_Importer.py`) y un script independiente automatizan la importación de las nubes de puntos y mallas a Blender, aplicando Geometry Nodes para visualizar puntos como pequeñas esferas.

## 2. Herramientas y Librerías Utilizadas
- **Python 3:** Lenguaje principal.
- **Open3D:** Manejo y visualización en tiempo real de nubes de puntos (`.ply`), geometría computacional y generación de la malla (`TriangleMesh`).
- **Intel RealSense SDK (`pyrealsense2` / Open3D RS API):** Comunicación directa con el sensor D455.
- **OpenCV (`cv2`):** Procesamiento de la imagen RGB para asignarla como textura a la malla.
- **SciPy (`scipy.spatial.Delaunay`):** Usado para aplicar Triangulación de Delaunay 2.5D, generando la estructura base de los polígonos de la malla.
- **Flask & Web Technologies:** Construcción de la interfaz web (`index.html`, CSS, JS) para controlar la cámara en red local.
- **Blender (bpy):** Python API de Blender para desarrollar herramientas de importación nativa.

## 3. Detalle de los Scripts Implementados

### 3.1. Captura en vivo (`escanear.py` y Web UI)
- Inicializa la cámara RealSense utilizando `o3d.t.io.RealSenseSensor()`.
- Captura la imagen RGB y los datos de profundidad (Depth) alineados.
- Transforma los fotogramas en una nube de puntos (`PointCloud`) usando los parámetros intrínsecos de la cámara (`intrinsic_matrix`).
- **Web App (`templates/index.html`):** Presenta un diseño moderno "Glassmorphism" con dos canales de video (RGB y Mapa de Calor). Al presionar "Capturar", guarda el estado actual en la carpeta `Saves/`.

### 3.2. Triangulación y Malla (`generar_malla_offline.py`)
Dado que una nube de puntos solo contiene vértices sueltos, este script construye una superficie texturizada.
1. **Proyección 2.5D:** Extrae las coordenadas X, Y, Z de la nube. Utiliza la matriz intrínseca (focal y centroide de la cámara) para proyectar cada punto 3D en el plano 2D de la imagen (píxeles U, V).
2. **Triangulación de Delaunay:** Crea los triángulos en 2D conectando los píxeles proyectados (`scipy.spatial.Delaunay`). Esto garantiza que no haya agujeros en la malla frontal.
3. **Filtro de Profundidad:** Elimina triángulos "estirados" (falsos bordes) comprobando que las aristas no midan más de 5 cm en el espacio 3D.
4. **Texturizado (UV Mapping):** Normaliza las coordenadas U y V generadas y mapea la fotografía RGB capturada (`captura_color.jpg`) sobre los triángulos. Calcula las normales de los vértices y exporta el resultado como un archivo `malla_texturizada.obj`.

### 3.3. Add-on para Blender (`RealSense_Importer.py`)
Un complemento (Add-on) integrado en el panel lateral de Blender.
- **Importación Raw:** Permite importar la nube de puntos (`.ply`) y el mapa de profundidad.
- **Geometry Nodes (`convert_to_spheres_geometry_nodes`):** Como Blender no renderiza puntos nativos de un `.ply`, este método genera automáticamente un árbol de nodos que instancia una Icosfera (de 3mm) en cada punto. Así, el escaneo es visible en renders de Cycles/Eevee.
- **Operador de Malla (`REALSENSE_OT_generate_mesh`):** Ejecuta de forma asíncrona (vía `subprocess`) el script `generar_malla_offline.py` y, al terminar, importa automáticamente el modelo 3D texturizado (`.obj`) al Viewport.

### 3.4. Script Standalone (`script_blender.py`)
Una herramienta sencilla para ejecutar en la pestaña "Scripting" de Blender.
Busca automáticamente la carpeta del escaneo más reciente dentro del directorio `Saves/` y procede a importarlo y aplicarle materiales y nodos de geometría, optimizando el flujo de trabajo sin requerir instalar el Add-on completo.

## 4. Métodos Clave y Flujo de Trabajo
1. **Captura:** El usuario escanea su entorno desde la Interfaz Web.
2. **Guardado (Saves/):** Se genera `nube_puntos.ply`, `nube_calor_profundidad.ply`, `captura_color.jpg` y `metadata_camara.json`.
3. **Importación:** El usuario abre Blender, usa el Add-on y selecciona la carpeta guardada.
4. **Construcción:** Presiona "Generar Malla Texturizada", lo cual procesa matemáticamente la nube en Python externo y devuelve un modelo sólido texturizado optimizado para renderizado y exportación (a motores como Unity o Unreal Engine).
