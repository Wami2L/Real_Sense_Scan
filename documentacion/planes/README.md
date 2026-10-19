# Planes de Desarrollo y Mejoras · RealSenseScans

En esta carpeta se recopilan las propuestas arquitectónicas, planes técnicos, hojas de ruta y estado de avance del sistema `RealSenseScans`. Todos los documentos cuentan con su versión en Markdown (`.md`) y su versión imprimible compilada en PDF (`.pdf`).

---

## Tabla Resumen de Estado de los Planes

| Plan | Título Técnico | Estado Actual | Archivos |
| :---: | :--- | :---: | :--- |
| **01** | Arquitectura y Soporte Multi-Cámara (Dual RealSense) | **Completado & Validado en HW** | [MD](file:///c:/Users/Usuario/Documents/RealSenseScans/documentacion/planes/01_Soporte_Dual_RealSense.md) · [PDF](file:///c:/Users/Usuario/Documents/RealSenseScans/documentacion/planes/01_Soporte_Dual_RealSense.pdf) |
| **02** | Roadmap y Mejoras Sugeridas | **En Progreso** | [MD](file:///c:/Users/Usuario/Documents/RealSenseScans/documentacion/planes/02_Roadmap_y_Mejoras_Sugeridas.md) · [PDF](file:///c:/Users/Usuario/Documents/RealSenseScans/documentacion/planes/02_Roadmap_y_Mejoras_Sugeridas.pdf) |
| **03** | Unificación de RealSense Scans y RealSense Live 3D | **Completado & Operativo** | [MD](file:///c:/Users/Usuario/Documents/RealSenseScans/documentacion/planes/03_Unificacion_Scans_y_Live3D.md) · [PDF](file:///c:/Users/Usuario/Documents/RealSenseScans/documentacion/planes/03_Unificacion_Scans_y_Live3D.pdf) |
| **04** | Implementación Dual Camera Scan y Fusión Geométrica | **Completado & Verificado** | [MD](file:///c:/Users/Usuario/Documents/RealSenseScans/documentacion/planes/04_Implementacion_Dual_Camera_Scan.md) · [PDF](file:///c:/Users/Usuario/Documents/RealSenseScans/documentacion/planes/04_Implementacion_Dual_Camera_Scan.pdf) |
| **05** | Modo de Renderizado Dual 3D en Vivo (WebGL / Three.js) | **Completado & Operativo** | [MD](file:///c:/Users/Usuario/Documents/RealSenseScans/documentacion/planes/05_Modo_Renderizado_Dual_En_Vivo.md) · [PDF](file:///c:/Users/Usuario/Documents/RealSenseScans/documentacion/planes/05_Modo_Renderizado_Dual_En_Vivo.pdf) |
| **06** | Plan Maestro para Claude: Captura y Renderizado Dual | **Completado & Operativo** | [MD](file:///c:/Users/Usuario/Documents/RealSenseScans/documentacion/planes/06_Plan_Implementacion_Captura_Dual_Claude.md) · [PDF](file:///c:/Users/Usuario/Documents/RealSenseScans/documentacion/planes/06_Plan_Implementacion_Captura_Dual_Claude.pdf) |
| **07** | Calibración Interactiva y Fusión Dual sin Duplicados | **Plan de Ejecución Listo** | [MD](file:///c:/Users/Usuario/Documents/RealSenseScans/documentacion/planes/07_Plan_Calibracion_Interactiva_y_Fusion_Sin_Duplicados.md) · [PDF](file:///c:/Users/Usuario/Documents/RealSenseScans/documentacion/planes/07_Plan_Calibracion_Interactiva_y_Fusion_Sin_Duplicados.pdf) |
| **08** | Fotogrametría RGB-D y Calibración Automática en Tiempo Real | **Propuesta Técnica Lista** | [MD](file:///c:/Users/Usuario/Documents/RealSenseScans/documentacion/planes/08_Fotogrametria_RGBD_y_Calibracion_Automatica_Tiempo_Real.md) · [PDF](file:///c:/Users/Usuario/Documents/RealSenseScans/documentacion/planes/08_Fotogrametria_RGBD_y_Calibracion_Automatica_Tiempo_Real.pdf) |
| **09** | Documentación de Cambios: Alineación Virtual y Estudio de Capturas | **Completado & Validado en HW** | [MD](file:///c:/Users/Usuario/Documents/RealSenseScans/documentacion/planes/09_Documentacion_Cambios_Alineacion_Virtual_y_Estudio_Capturas.md) · [PDF](file:///c:/Users/Usuario/Documents/RealSenseScans/documentacion/planes/09_Documentacion_Cambios_Alineacion_Virtual_y_Estudio_Capturas.pdf) |
| **10** | Plan de Optimización y Consolidación para Pre-Producción | **En curso** | [MD](10_Plan_Optimizacion_Pre_Produccion.md) |
| **11** | Ecosistema Unificado: Estudio y Live 3D en un Solo Servidor | **Propuesta** | [MD](11_Plan_Ecosistema_Unificado_Estudio_Live3D.md) |

---

## Detalle de Contenido por Plan

### 1. [01_Soporte_Dual_RealSense.md](file:///c:/Users/Usuario/Documents/RealSenseScans/documentacion/planes/01_Soporte_Dual_RealSense.md)
* Topología de hardware, tablas de ancho de banda USB 3.2 xHCI.
* Mitigación de interferencia de proyectores láser IR (Master con emisor ON, Slave con emisor OFF).
* Análisis de sincronización por hardware Genlock (Pines GPIO 1 y 2 a 1.8V TTL) vs. sincronización por software.
* **Validación en hardware real:** 2 cámaras RealSense D455 (`207322250777` y `203522251709`) transmitiendo simultáneamente a 30.0 FPS y 30.2 FPS.

### 2. [02_Roadmap_y_Mejoras_Sugeridas.md](file:///c:/Users/Usuario/Documents/RealSenseScans/documentacion/planes/02_Roadmap_y_Mejoras_Sugeridas.md)
* Integración del visor interactivo WebGL (Three.js) con GPU Vertex Shaders.
* Procesamiento y filtrado de nubes con Open3D (SOR, Poisson, generación de mallas texturizadas).
* Integración en vivo con Blender vía WebSockets.

### 3. [03_Unificacion_Scans_y_Live3D.md](file:///c:/Users/Usuario/Documents/RealSenseScans/documentacion/planes/03_Unificacion_Scans_y_Live3D.md)
* Diagnóstico de colisión de endpoints USB entre aplicaciones independientes.
* Fusión arquitectónica en un único servidor asíncrono (`app.py` / `live3d.server`).
* Panel web unificado en el puerto `5002` combinando visor 3D, escaneo rápido, mapeo multi-foto 360° e historial de guardados con miniaturas.

### 4. [04_Implementacion_Dual_Camera_Scan.md](file:///c:/Users/Usuario/Documents/RealSenseScans/documentacion/planes/04_Implementacion_Dual_Camera_Scan.md)
* Matemáticas de calibración de pose relativa $T_{2 \to 1} \in SE(3)$ con tablero ChArUco y OpenCV solvePnP.
* Pipeline de captura estática simultánea y fusión de nubes en Open3D (`pcd1 + pcd2.transform(T)`).
* Filtros de remoción de ruido estadístico y submuestreo por vóxeles.
* Guardado organizado en `Saves/<fecha>/dual_scan_<hora>/` con `nube_fusionada.ply` y metadatos.

### 5. [05_Modo_Renderizado_Dual_En_Vivo.md](file:///c:/Users/Usuario/Documents/RealSenseScans/documentacion/planes/05_Modo_Renderizado_Dual_En_Vivo.md)
* Transmisión y co-renderizado simultáneo de **ambas cámaras en el mismo visor 3D en vivo** a 30 FPS.
* 4 Modos de visualización: Dual Fusion (cobertura combinada 360°), Split-Screen (pantalla dividida), PiP y Cámara única.
* Multiplexación de paquetes binarios en el WebSocket `/ws` con tag de cámara.
* Matriz de cambio de base entre coordenadas RealSense y Three.js ($C \cdot T \cdot C^{-1}$).
* Panel de ajuste fino interactivo (sliders de traslación/rotación) para inspección de solape en vivo.

### 6. [06_Plan_Implementacion_Captura_Dual_Claude.md](file:///c:/Users/Usuario/Documents/RealSenseScans/documentacion/planes/06_Plan_Implementacion_Captura_Dual_Claude.md)
* Guía de ejecución paso a paso especialmente diseñada como contexto ejecutable para Claude / Pair Programming.
* Arquitectura de red, buffers de decodificación y shaders WebGL listos para desplegar.
* Gotchas de hardware (gotcha de reseteo USB, regla de emisor IR, modo de sincronización).
* Checklist de aceptación con 5 pruebas empíricas de validación.

### 7. [07_Plan_Calibracion_Interactiva_y_Fusion_Sin_Duplicados.md](file:///c:/Users/Usuario/Documents/RealSenseScans/documentacion/planes/07_Plan_Calibracion_Interactiva_y_Fusion_Sin_Duplicados.md)
* Diagnóstico de duplicación espacial ("efecto fantasma"): discrepancia entre la matriz por defecto (25 cm) y la distancia real física (~10.4 cm con ~25° de convergencia).
* **Calibración Automática sin Tablero (ICP Multiescala):** Endpoint `/api/dual/auto_align` para calcular la pose $T_{2 \to 1}$ en $<1.5$ s a partir de la escena activa.
* **Ajuste Manual Interactivo 60 Hz en el Visor WebGL:** Sliders de traslación ($X, Y, Z$) y rotación (Yaw, Pitch, Roll) con feedback instantáneo en Three.js bajo modo de color "Solape" (Cian y Magenta).
* **Fusión y Desduplicación en Captura Estática (`capture_dual_scan`):** Voxel Grid Downsampling (2.5 mm) para eliminar duplicados en la zona compartida, estimación de normales y generación de mallas Poisson.

### 8. [08_Fotogrametria_RGBD_y_Calibracion_Automatica_Tiempo_Real.md](file:///c:/Users/Usuario/Documents/RealSenseScans/documentacion/planes/08_Fotogrametria_RGBD_y_Calibracion_Automatica_Tiempo_Real.md)
* Fundamentos de fotogrametría clásica (SfM + MVS) y sus limitaciones en tiempo real (ambigüedad de escala, costo computacional).
* Análisis de las técnicas fotogramétricas ya presentes en el proyecto (`multiscan.py` con descriptores FPFH 3D, RANSAC, Colored ICP y Poisson Surface Reconstruction).
* Arquitectura de **Fotogrametría RGB-D en Tiempo Real**: elevación 2D $\to$ 3D (RGB-D Feature Lifting) usando ORB/SIFT y profundidad métrica directa sin ambigüedad de escala.
* Algoritmo de resolución cerrada de Kabsch/Umeyama + RANSAC para match automático sin tablero en ~15-30 ms.
* Diseño del nuevo módulo `live3d/feature_matcher.py` y modo de alineación continua en caliente.

### 9. [09_Documentacion_Cambios_Alineacion_Virtual_y_Estudio_Capturas.md](file:///c:/Users/Usuario/Documents/RealSenseScans/documentacion/planes/09_Documentacion_Cambios_Alineacion_Virtual_y_Estudio_Capturas.md)
* **Registro Técnico y Changelog:** Detalle de hitos y soluciones implementadas en el entorno desacoplado (`test_fotogrametria/`, puerto 5005).
* **Estudio de Capturas Multi-Toma:** Receptor continuo a 60 FPS, disparador instantáneo a demanda con efecto de obturador y carrusel de tomas en memoria.
* **Reflejo del Alineado en el Entorno de Cámaras:** Fusión 2D en perspectiva (reproyección 3D de Cam 2 sobre Cam 1 con control de opacidad 0–100%), Anaglifo 3D Estéreo (Rojo/Cian) y Nivel Epipolar Horizontal (líneas láser guía de altura).
* **Control en Caliente del Visor 3D:** Solución de endpoints estáticos `/assets/` y proxy `/viewer3d` para manipular Three.js en vivo a 60 FPS sin bloqueos de origen cruzado.

### 10. [10_Plan_Optimizacion_Pre_Produccion.md](10_Plan_Optimizacion_Pre_Produccion.md)
* Diagnóstico de ineficiencias (captura dual, estudio, servidor, calibración) y fases F0–F6.
* Iteración 2 (§6): motor único de alineación, estado por cámara, búsqueda de montaje, previsualización de la toma.

### 11. [11_Plan_Ecosistema_Unificado_Estudio_Live3D.md](11_Plan_Ecosistema_Unificado_Estudio_Live3D.md)
* Un solo proceso y puerto con el estudio como interfaz principal y el visor 3D como panel.
* Fases U0–U5: núcleo del estudio extraído, backend único con datos nativos, frontend único, calibración persistente y multi-toma, trabajos en cola y Blender en vivo.
* Prerrequisitos de hardware: montaje fijo, metadatos UVC de la Cam 2, firmware y emisores.
