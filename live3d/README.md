# RealSense Live 3D · Streaming y Visualización en Tiempo Real

Sistema de streaming de geometría 3D en tiempo real desde cámaras Intel RealSense (probado y calibrado con RealSense D455 en USB 3.2), optimizado para renderizado acelerado por GPU en el navegador (WebGL2 / Three.js) y preparado arquitectónicamente para Blender.

---

## 🚀 Inicio Rápido

### 1. Iniciar el Servidor con la Cámara RealSense
Asegúrate de que la cámara esté conectada por USB 3.0/3.2 y que no haya otra aplicación usando el sensor (`app.py` cerrada):

```bash
python -m live3d
```
> Opciones útiles:
> * `--port 5002` (por defecto es 5002)
> * `--host 0.0.0.0` (para permitir conectar desde otra PC o tablet en la red local)
> * `--synthetic` (inicia con una escena 3D simulada y animada si no tienes la cámara conectada)

### 2. Abrir el Visor en el Navegador
Abre en Chrome, Edge o Firefox:
**[http://localhost:5002](http://localhost:5002)**

---

## 🎨 Características del Visor WebGL

1. **Reconstrucción 100% en GPU**:
   - La profundidad cruda `uint16` se transfiere directamente a la GPU como una textura `R16UI`.
   - El *Vertex Shader* calcula las coordenadas 3D de cada píxel utilizando las intrínsecas ópticas de la cámara D455.
   - Las coordenadas UV de textura se calculan matemáticamente usando la matriz de extrínsecas (profundidad $\to$ color), eliminando el costoso paso de alineación en CPU de librealsense.
2. **Modos de Geometría**:
   - **Puntos**: Nube de puntos dinámica con tamaño adaptable a la distancia y opción de puntos circulares.
   - **Malla (Mesh)**: Triangulación continua con descarte de bordes y desconexión automática en discontinuidades de profundidad (*flying pixels*).
3. **Modos de Color**:
   - **RGB**: Color real mapeado desde el sensor RGB.
   - **Profundidad**: Mapa de calor con la paleta de alto contraste *Turbo*.
   - **Normales**: Normales de superficie calculadas en tiempo real mediante diferencias centrales.
   - **Arcilla (Clay)**: Sombreado difuso para inspeccionar la geometría y relieve sin la distracción de la textura.
4. **Controles en Caliente**:
   - Rango de profundidad mínimo y máximo (Near / Far).
   - Cambio de resolución y FPS del sensor sin reiniciar el proceso.
   - Activación de filtros de librealsense en el servidor (Filtro temporal, espacial, emisor IR, relleno de huecos).
   - Decimación ajustable (1×, 2×, 3×, 4×).
5. **Herramientas de Exportación**:
   - **Captura PNG**: Captura de alta resolución de la vista actual.
   - **Exportar PLY**: Genera un archivo `.ply` binario con el cuadro actual, con orientación de coordenadas estándar para Blender (+Z arriba).
   - **Congelar / Reanudar**: Pausa la recepción para inspeccionar un cuadro específico.

### Atajos de Teclado
* <kbd>Espacio</kbd>: Congelar / reanudar el stream.
* <kbd>M</kbd>: Alternar entre Nube de Puntos y Malla.
* <kbd>1</kbd> - <kbd>4</kbd>: Cambiar modo de color (RGB, Profundidad, Normales, Arcilla).
* <kbd>H</kbd>: Ocultar / mostrar el panel lateral de ajustes.
* <kbd>F</kbd>: Pantalla completa.
* <kbd>R</kbd>: Restablecer vista frontal.

---

## ⚙️ Arquitectura y Preparación para Blender (Iteración Siguiente)

Para el streaming hacia Blender, se diseñó la arquitectura teniendo en cuenta las limitaciones del hilo principal de Blender:

1. **Protocolo Binario RSF1 (`live3d/protocol.py`)**:
   - Depende **únicamente de Python estándar y NumPy**. No requiere compilar librerías C ni instalar `pyrealsense2` dentro del entorno de Blender.
   - En lugar de enviar nubes de vértices pesadas en float32 (~12-16 MB por cuadro), envía la profundidad comprimida (zlib/raw) y la imagen RGB (~200 KB a 1 MB).
2. **Control de Flujo por Créditos (ACK)**:
   - El cliente notifica cuando terminó de procesar un cuadro (`ack`).
   - El servidor mantiene un máximo de 2 cuadros en tránsito (`MAX_INFLIGHT = 2`) y descarta automáticamente cuadros viejos ("el último cuadro gana"). De esta forma, si Blender está ocupado renderizando o actualizando la vista, **nunca se produce lag acumulativo**.
3. **Cliente de Verificación (`live3d/tools/probe_client.py`)**:
   - Sirve como plantilla base para el futuro operador/timer de Blender:
     ```bash
     python -m live3d.tools.probe_client --port 5002 --frames 60
     ```
