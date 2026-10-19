# 01. Arquitectura del Sistema y Aplicación Web

Este documento detalla la arquitectura general del proyecto de Escáner 3D y el diseño de la interfaz web que sirve como punto de control para la captura de datos.

## 1. Arquitectura General
El ecosistema de `RealSenseScans` está diseñado en capas para separar la captura de datos de hardware (pesada) del procesamiento matemático y del renderizado final.
- **Capa de Frontend:** Interfaz de usuario construida en HTML/CSS/JS moderno ("Glassmorphism").
- **Capa de Backend (Flask):** Servidor local que se encarga de extraer los feeds del sensor D455 y exponerlos como streams HTTP (MJPEG).
- **Capa de Procesamiento Offline:** Scripts independientes de Python que realizan cálculos geométricos complejos (Delaunay) sin bloquear la captura de la cámara.
- **Capa de Presentación 3D:** Integración nativa en Blender a través de la API `bpy`.

## 2. El Frontend Web (`templates/index.html` y estáticos)
La aplicación web provee una interfaz amigable (UI) para que el usuario pueda visualizar lo que "ve" el escáner y emitir la orden de captura.

### Características Visuales:
- **Glassmorphism:** Uso intensivo de paneles con desenfoque de fondo (`backdrop-filter: blur`), generando una estética premium.
- **Diseño Dual Feed:** La pantalla se divide en dos visualizadores sincronizados:
  1. **Visión RGB:** Stream de video a color estándar.
  2. **Mapa de Calor:** Representación térmica o graduada de la información de profundidad (Depth), permitiendo al usuario saber qué tan lejos o cerca están los objetos físicos.
- **Interactividad:** Iconos vectoriales renderizados con `Lucide` y respuestas táctiles/visuales implementadas en `static/js/main.js`. Al hacer clic en "Capturar Ambos", el Frontend envía una petición REST (usualmente `/capture` o similar) al backend de Flask.

### Integración Backend:
En el código HTML, los elementos `<img src="{{ url_for('video_feed') }}">` y `<img src="{{ url_for('depth_feed') }}">` permiten a Flask inyectar un flujo de fotogramas directamente al navegador, logrando un monitor de cámara en vivo sin requerir WebRTC o sockets complejos, optimizando el ancho de banda en la red local.
