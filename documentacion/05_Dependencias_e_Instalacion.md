# 05. Dependencias, Instalación y Librerías

Este documento detalla las librerías necesarias para ejecutar el proyecto **RealSenseScans**, así como los pasos para configurar el entorno de desarrollo y ejecutar la aplicación.

## 1. Requisitos Previos

*   **Python 3.8 - 3.10**: Se recomienda para mantener la compatibilidad con Open3D y el SDK de Intel RealSense.
*   **Hardware**: Cámara de profundidad Intel RealSense (serie D400, como la D455) conectada a un puerto USB 3.0.
*   **Blender (Opcional)**: Para utilizar los scripts de importación y manipulación de geometría (recomendable v3.0 o superior).

## 2. Librerías y Dependencias (Python)

El proyecto hace uso de las siguientes bibliotecas principales para la captura, cálculo matemático, visión por computadora y servidor web:

*   **`pyrealsense2`**: Interfaz oficial de Intel para comunicarse directamente con el sensor, leer los streams de video RGB y de profundidad alineada.
*   **`open3d`**: Herramienta esencial para el procesamiento 3D. Se usa para visualizar y manipular las nubes de puntos y generar modelos en tiempo real.
*   **`opencv-python` (cv2)**: Utilizada para el procesamiento de texturas, redimensionado de imágenes y manipulación de las capas RGB que se aplican sobre la malla.
*   **`scipy`**: Brinda acceso a `scipy.spatial.Delaunay`, necesario para realizar la triangulación 2.5D que convierte la nube de puntos aislados en una malla sólida y continua.
*   **`numpy`**: Soporte matemático para el manejo eficiente de grandes arreglos de datos correspondientes a coordenadas 3D y píxeles 2D.
*   **`Flask`**: Framework de backend que expone los flujos de la cámara en formato web y provee la interfaz ("Glassmorphism") para controlar los escaneos desde el navegador.

*(Nota: Para los scripts que convierten la documentación de Markdown a PDF, se utilizan `markdown` y `xhtml2pdf`).*

## 3. Guía de Instalación

Es altamente recomendable instalar todo dentro de un entorno virtual (`venv`) para no interferir con las dependencias globales del sistema.

### Paso 1: Crear y Activar Entorno Virtual

En la terminal (dentro de la carpeta `RealSenseScans`):

**En Windows:**
```bash
python -m venv venv
.\venv\Scripts\activate
```

**En Linux / macOS:**
```bash
python3 -m venv venv
source venv/bin/activate
```

### Paso 2: Instalar las Librerías

Puedes instalar directamente todas las dependencias requeridas ejecutando el siguiente comando a través de `pip`:

```bash
pip install open3d opencv-python pyrealsense2 scipy numpy Flask
```

*(Si se añade posteriormente un archivo `requirements.txt`, bastaría con correr `pip install -r requirements.txt`)*

## 4. Ejecución del Sistema

### A. Escaneo en Tiempo Real
Para iniciar la interfaz de escáner y la comunicación con la cámara:
```bash
python app.py
```
Abre tu navegador web y entra en: `http://localhost:5000`

### B. Post-Procesamiento (Generar Malla)
Tras presionar el botón "Capturar Ambos" en la web, los datos se guardarán en la carpeta `Saves/`. Para convertir estos datos puros en una malla texturizada `.obj`:
```bash
python generar_malla_offline.py
```

### C. Importar a Blender
1. Abre Blender.
2. Ve a `Edit > Preferences > Add-ons`.
3. Haz clic en **Install...**, busca el archivo `RealSense_Importer.py` y actívalo.
4. Usa el nuevo panel derecho en la vista 3D para importar automáticamente tu nube de puntos o generar y cargar la malla procesada.
