# Plan Técnico: Unificación de RealSense Scans y RealSense Live 3D

## 1. Conclusión y Veredicto
**Sí, es rotundamente recomendable combinar ambas aplicaciones.**

Actualmente existen dos herramientas separadas en el mismo repositorio:
1. **RealSense Scans (`app.py` / Flask):** Especializada en captura estática, multi-escaneo secuencial (ICP / Open3D), reconstrucción de mallas TSDF/Poisson, gestión de archivos guardados (`Saves/`) e integración con Blender.
2. **RealSense Live 3D (`live3d/` / aiohttp + WebGL):** Especializada en streaming de geometría 3D en tiempo real (30 FPS) acelerada por GPU (Vertex Shaders) en Three.js con controles dinámicos de sensor.

---

## 2. Razones Fundamentales para la Unificación

### A. Eliminación del Conflicto de Acceso al Hardware (Dispositivo Ocupado)
* Las cámaras Intel RealSense no permiten que dos procesos del sistema operativo abran streams concurrentes sobre el mismo endpoint USB.
* Si el usuario ejecuta `app.py` y quiere usar `live3d`, debe detener un proceso para arrancar el otro.
* **Con la unificación:** Un único proceso maestro gestiona el pipeline de la cámara y distribuye los datos tanto al streaming WebGL de baja latencia como a las funciones de captura y fotogrametría 3D.

### B. Salto Radical en la Experiencia de Usuario (Flujo de Trabajo)
Actualmente el flujo está fragmentado:
* En `app.py`, el usuario previsualiza en un video 2D plano (MJPEG), "a ciegas" de cómo se ve la geometría 3D real hasta que pulsa escanear.
* En `live3d`, el usuario ve la geometría 3D en tiempo real pero no tiene las funciones de guardado organizado, multi-escaneo o exportación avanzada de `app.py`.

**Con la plataforma unificada:**
```
[ Visor 3D WebGL en Tiempo Real ]
          |
          +--> 1. Encuadrar y ver calidad de profundidad en vivo (Clay, Normales, RGB)
          +--> 2. Boton "Captura Rapida 3D" (guarda cuadro de alta resolucion)
          +--> 3. Boton "Multi-Scan Asistido" (el visor 3D muestra en vivo como se va uniendo)
          +--> 4. Exportar / Enviar a Blender con un clic
```

### C. Reducción de Deuda Técnica y Código Duplicado
* Actualmente existen dos gestores de cámara independientes: `camera_manager.py` y `live3d/capture.py`.
* Ambos implementan lógica de reconexión, filtros de librealsense, perfiles de resolución y watchdogs por separado.
* Al unificarlos, se crea un único `CameraService` modular que además facilita la incorporación de dos cámaras simultáneas.

---

## 3. Arquitectura Propuesta para la Plataforma Unificada

```
+-------------------------------------------------------------+
|                 Intel RealSense (D4xx)                      |
|             (1 o 2 camaras por hardware)                    |
+------------------------------+------------------------------+
                               |
                               v
+-------------------------------------------------------------+
|                    CameraEngine Unificado                   |
|              (Thread-safe, filtros, watchdog)               |
+------------------------------+------------------------------+
                               |
               +---------------+---------------+
               |                               |
        Frames continuos               Snapshots a demanda
               |                               |
               v                               v
+------------------------------+ +----------------------------+
|      Live Stream Engine      | |    Reconstruction Engine   |
|   (WebSocket binario R16UI)  | |   (Open3D, ICP, TSDF)      |
+--------------+---------------+ +--------------+-------------+
               |                                |
               +---------------+----------------+
                               |
                               v
+-------------------------------------------------------------+
|                  Servidor Web Asincrono                     |
|                  (aiohttp / WebSocket)                      |
+------------------------------+------------------------------+
                               |
                               v
+-------------------------------------------------------------+
|                  Frontend Web Studio 3D                     |
|        - Viewport WebGL en tiempo real (Three.js)           |
|        - Panel de controles en vivo (Near/Far/Filtros)      |
|        - Boton "Captura 3D" y "Multi-Scan"                  |
|        - Historial de capturas Saves/                       |
|        - Boton "Exportar a Blender"                         |
+-------------------------------------------------------------+
```

---

## 4. Comparativa: Estado Separado vs. Estado Unificado

| Aspecto | Estado Actual (Separadas) | Estado Unificado (Recomendado) |
| :--- | :--- | :--- |
| **Puertos de red** | 2 puertos (5000 para Flask, 5002 para Live3D) | 1 único puerto web centralizado |
| **Uso de la cámara** | Conflicto si ambas apps están abiertas | Acceso compartido sin interrupciones |
| **Previsualización de captura** | Video 2D tradicional | Malla 3D / Nube de puntos interactiva en vivo |
| **Mantenimiento** | 2 motores de captura duplicados | 1 motor robusto y escalable a multicámara |
| **Integración Blender** | Scripts independientes y manuales | Integración fluida desde la interfaz central |

---

## 5. Estrategia de Transición en 3 Pasos

1. **Paso 1: Backend Base (`aiohttp` unificado):**
   - Adoptar el servidor asíncrono de `live3d` (que ya soporta WebSockets binarios de alta velocidad) y montar sobre él las rutas API de `app.py` (`/api/capture`, `/api/multiscan`, etc.).
2. **Paso 2: Frontend Integrado:**
   - Incorporar en el visor WebGL de `live3d` una barra de herramientas con los controles de escaneo, visor de sesiones anteriores y selector de perfiles.
3. **Paso 3: Motor de Fusión y Multi-Cámara:**
   - Conectar el futuro soporte para dos cámaras directamente a esta arquitectura única, permitiendo ver en tiempo real la combinación de ambas cámaras en el visor 3D antes de generar el escaneo final.

---

## 6. Estado de Implementación: ¡Completado y Operativo al 100%!

La unificación se ha ejecutado y probado exitosamente en el sistema:
* **Punto de Entrada Único:** [app.py](../../app.py) inicia directamente la plataforma unificada (accesible también mediante `python -m live3d` o `python app.py --port 5002`).
* **Respaldo Legacy:** El antiguo servidor Flask ha quedado preservado de forma segura en [app_legacy_flask.py](../../app_legacy_flask.py).
* **Motor de Cámara Concurrente:** [live3d/capture.py](../../live3d/capture.py) centraliza la conexión con el hardware de ambas RealSense D455 (`DualRealSenseSource`), eliminando colisiones de USB.
* **Captura Individual, Multi-Scan 360° y Escaneo Dual:** Integrados en el servidor asíncrono [live3d/server.py](../../live3d/server.py) con soporte para Open3D, calibración ChArUco y almacenamiento automático en `Saves/`.
* **Frontend Web 3D Unificado:** El visor en [live3d/web/index.html](../../live3d/web/index.html) cuenta con:
  - Módulo dinámico **✨ Modo Dual Activo** con conmutación instantánea de cámara en vivo y botón de **Captura Dual 360°**.
  - Panel de mapeo multi-foto asistido con miniaturas en vivo y cálculo de registro Open3D.
  - Explorador modal de historial de escaneos guardados con previsualización de miniaturas tanto para escaneos simples, multi-captura como escaneos duales (`cam1_color.jpg`).


