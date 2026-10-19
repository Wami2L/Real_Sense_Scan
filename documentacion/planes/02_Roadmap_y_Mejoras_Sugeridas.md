# Roadmap y Sugerencias de Mejora del Proyecto RealSenseScans

Este documento recopila las oportunidades de optimización y evolución para el sistema de escaneo y visualización 3D.

---

## 1. Visualización Web Integrada (Visor 3D en el Dashboard)
* **Situación actual:** La interfaz web de `app.py` muestra streams de video 2D (color y mapa de calor). Para ver la geometría 3D se depende de Blender o scripts externos.
* **Sugerencia:**
  - Integrar un visor WebGL interactivo (Three.js / Canvas 3D) en la página principal (`templates/index.html`), permitiendo inspeccionar la nube de puntos o malla capturada directamente en el navegador antes de enviarla a Blender.
  - Conectar el visor de `live3d` directamente en una pestaña de la interfaz principal para tener todo centralizado en un único puerto web.

---

## 2. Optimización de la Calidad de Nube y Malla 3D
* **Filtrado Avanzado de Ruido:**
  - Añadir *Statistical Outlier Removal* (SOR) y *Radius Outlier Removal* (ROR) de Open3D al pipeline de captura para eliminar partículas flotantes ("flying pixels") y ruido en los bordes de profundidad.
  - Aplicar filtrado bilateral sobre la imagen de profundidad para suavizar planos sin perder bordes nítidos.
* **Texturizado con Mapeo UV:**
  - El sistema actual asigna color por vértice (`vertex colors`).
  - Para modelos de alta fidelidad destinados a Blender, Unreal Engine o Unity, generar mapas de textura UV (`.obj` + `.mtl` + `texture.png`) mediante proyección de las imágenes RGB capturadas.

---

## 3. Arquitectura Multi-Cámara y Captura Simultánea
* **Gestor de Dispositivos (`MultiCameraManager`):**
  - Desacoplar el pipeline único de `camera_manager.py` para permitir 1, 2 o más cámaras simultáneas con descubrimiento automático por número de serie.
  - Habilitar captura sincrónica de 360° o cobertura frontal+lateral sin necesidad de mover la cámara o el objeto.
* **Calibración Guiada:**
  - Añadir una herramienta web interactiva para calibración extrínseca rápida con patrones ChArUco o AprilTags.

---

## 4. Integración Bidireccional con Blender
* **Addon Live Streaming en Blender:**
  - Crear un operador en Python para Blender que se conecte por WebSocket directamente a `live3d`, actualizando un objeto `Mesh` de Blender a 30 FPS en el viewport en tiempo real.
* **Script de Automatización Mejorado:**
  - Ya existen `script_blender.py` y `RealSense_Importer.py`. Se puede añadir soporte para remeshing cuadrilátero automático (*QuadriFlow*) y corrección de normales con un solo clic.

---

## 5. Medición y Metrología 3D
* **Herramienta de Medición en Web / Blender:**
  - Aprovechar la escala métrica real de RealSense (Z en milímetros/metros) para permitir medir distancias entre puntos, volúmenes de objetos o secciones transversales directamente desde la interfaz web o generar informes técnicos en PDF.
