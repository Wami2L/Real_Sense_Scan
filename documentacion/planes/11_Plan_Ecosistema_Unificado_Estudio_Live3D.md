# Plan 11: Ecosistema unificado — Estudio y Live 3D en un solo servidor

> **Rama:** `Pre_Produccion`
> **Fecha:** 2026-10-19
> **Estado:** Propuesta. Sustituye y concreta la Fase 3 del plan 10. Absorbe parte de sus fases 2, 4, 5 y 6.
> **Punto de partida:** iteración 2 del plan 10 (motor de alineación único, estado por cámara, búsqueda de montaje, previsualización de la toma).

---

## 1. Objetivo

Un solo proceso y un solo puerto, con **el estudio como interfaz principal**: es la que tiene todas las herramientas de calibración, y el usuario ya no quiere trabajar con el visor Live 3D por separado. El visor 3D pasa a ser un panel dentro del estudio.

Además, todas las decisiones de calibración se toman con **datos nativos** (snapshot emparejado de las cámaras, profundidad sin comprimir), no con los cuadros JPEG del stream.

Resultado esperado:
- `iniciar_sistema.bat` lanza **un** proceso y abre **una** página (`http://localhost:5002/`).
- Cámaras, previsualización de la toma, tomas, alineación, verificación (fusión 2D, anaglifo, nivel, rayos), visor 3D, escaneo simple, dual y multicaptura, guardados y ajustes: todo en la misma página.
- Desaparecen el puerto 5005, el cliente WebSocket duplicado, el parche de `main.js` por reemplazo de texto (E3), el CORS entre puertos y la recodificación MJPEG (E2).

---

## 2. Situación actual

| Aspecto | Hoy |
|---|---|
| Procesos | 5002 abre las cámaras y sirve el visor y la API. 5005 (estudio) es cliente del WebSocket de 5002. Hay que arrancar los dos. |
| Datos del estudio | Cuadros del **stream**: color JPEG y profundidad zlib, alineada a color en CPU. El snapshot nativo emparejado solo existe en 5002 (`/api/dual/auto_align`, captura dual). |
| Tomas del estudio | En RAM, máximo 15. Se pierden al reiniciar (E4). |
| Visor 3D en el estudio | `iframe` a `/viewer3d` con `main.js` parcheado con `str.replace` para apuntar a 5002 (E3). |
| Previsualización de la toma | Hecha (19-10): `live3d/alignment.preflight` se ejecuta en 5005 sobre los cuadros del stream, a ~3 Hz y solo mientras alguien mira. |
| Calibración | `src.set_extrinsics()` con escritura atómica, sin historial ni deshacer. Las herramientas están repartidas: ChArUco y auto-alinear en 5002; alineación con confirmación, sliders con guardado explícito y verificación en 5005. |
| Escaneo y guardados | Solo en la interfaz de 5002 (simple, dual 360°, multicaptura, galería de `Saves/`). |

### Inventario de funciones y destino

| Función | Hoy | En el ecosistema |
|---|---|---|
| Visor 3D en vivo (modos, solape, sensor único) | 5002 `/` | Panel **3D** del estudio (componente, sin iframe) |
| Vista de cámaras en vivo + guías + previsualización | 5005 | Panel **Cámaras**, calculado en el servidor sobre datos nativos |
| Tomas emparejadas y galería | 5005 (RAM) | Panel **Tomas**, persistidas en disco |
| Alineación (motor único, búsqueda de montaje, confirmación) | 5005 `/api/align` y 5002 `/api/dual/auto_align` | Un solo endpoint sobre la toma nativa |
| Fusión 2D, anaglifo, nivel, rayos | 5005 | Panel **Tomas** (verificación) |
| Sliders con guardar explícito | 5002 (guardan) y 5005 (previsualizan) | Uno solo: previsualizar con encaje en vivo y guardar explícito |
| ChArUco | 5002 `/api/dual/calibrate` | Asistente **Calibración inicial** del estudio |
| Escaneo simple, dual 360° y multicaptura | 5002 | Panel **Escaneo** |
| Galería de `Saves/` | 5002 | Panel **Escaneo** (con caché, S2) |
| Ajustes de cámara (perfil, filtros, emisor) | 5002 (sliders) | Panel **Ajustes** |
| Estado por cámara | 5002 `status` + 5005 `/api/status` | Un único mensaje `status` por WebSocket |

---

## 3. Arquitectura objetivo

```
                       ┌────────────── proceso único · aiohttp :5002 ──────────────┐
 Cam 1 ── worker ─────►│ capture: fuentes duales (estado por cámara, reconexión)    │
 Cam 2 ── worker ─────►│   ├─ stream  ─────────► Hub /ws (RSF1)  ──► panel 3D        │
                       │   ├─ ring buffer N cuadros + timestamps                      │
                       │   │     ├─ snapshot emparejado (min |Δt|) ─► tomas, escaneo  │
                       │   │     └─ cuadro diezmado ─► preflight (~3 Hz, con demanda) │
                       │   │                              └─► guías + métricas por WS │
                       │ studio: tomas en disco · alineación · verificación           │
                       │ calibration_store: lock · atómico · historial · deshacer     │
                       │ jobs: cola (captura dual, multi, alineación) · progreso WS   │
                       └──────────────────────────────────────────────────────────────┘
                         web/ (ES modules, sin build): estudio = shell + paneles
                         Cámaras · 3D · Tomas · Calibración · Escaneo · Ajustes
```

Principios:
- **Un dueño por cámara**: solo los workers de `capture` tocan el dispositivo. El resto pide cuadros al ring buffer.
- **Lo pesado fuera del event loop**, en el pool de trabajos. Si Open3D degrada el stream por el GIL, se pasa a un `ProcessPoolExecutor`, como ya advierte el plan 10.
- **Empujar, no sondear**: el estado, la previsualización y el progreso de los trabajos llegan por el WebSocket de estado. El sondeo HTTP queda como respaldo.
- **El motor manda**: `live3d/alignment.py` (alineación y previsualización) es la única fuente de verdad para la calidad. La interfaz no calcula métricas propias.

---

## 4. Decisiones (con recomendación)

| # | Decisión | Recomendación | Alternativa |
|---|---|---|---|
| D1 | Interfaz principal | **Estudio** como página raíz; el visor 3D actual pasa a ser el panel 3D | Mantener las dos páginas en el mismo servidor (más barato, pero sigue duplicando herramientas) |
| D2 | Dónde se calcula la previsualización | **Servidor**, sobre la profundidad nativa diezmada del ring buffer, publicada por WS solo con demanda | En el navegador (tendría que enviar la profundidad completa) |
| D3 | Guías sobre la vista en vivo | **Capas en el navegador** (canvas sobre la imagen) a partir de las máscaras diezmadas que manda el servidor (~25 k etiquetas, ~3 KB con RLE/zlib) | Pintarlas en el servidor y recodificar JPEG (lo de hoy: CPU y latencia) |
| D4 | Tomas del estudio | **En disco**: `Saves/calibracion/AAAA-MM-DD/toma_HHMMSS/` con `camN_color.png`, `camN_depth.png` (uint16) y `meta.json` | En RAM (lo de hoy) |
| D5 | Frontend | **ES modules sin build**, reutilizando `viewer.js` y `stream.js` como componentes | Framework con build (más dependencias para un proyecto de un solo puesto) |
| D6 | Calibración | **`CalibrationStore`** con historial de las últimas N, deshacer y metadatos de origen (motor, ChArUco, manual) | Escribir directo al JSON (lo de hoy) |

---

## 5. Fases

Cada fase debe funcionar con `--synthetic --dual` **y** con las dos D455 antes de pasar a la siguiente. 5005 sigue disponible hasta cerrar U2.

### U0 — Extraer el núcleo del estudio (bajo esfuerzo, bajo riesgo)
- Crear el paquete `live3d/studio/`:
  - `captures.py`: tomas y su persistencia.
  - `views.py`: fusión 2D, anaglifo, nivel y rayos.
  - `preflight_service.py`: bucle con demanda y caché de máscaras.
  - `guides.py`: dibujo de guías, mientras se pinten en el servidor.
- Interfaz `PairSource` con dos implementaciones: `StreamPairSource` (el receptor WS actual de 5005) y `NativePairSource` (en proceso, ver U1). `servidor_test.py` queda como una capa fina sobre el paquete.
- **Criterio:** los tests pasan, 5005 funciona igual y `servidor_test.py` baja de ~800 a menos de 250 líneas.

### U1 — Backend único (medio)
- **Ring buffer** por cámara (4–6 cuadros con timestamp) y **snapshot emparejado** por mínimo |Δt| (Fase 2 del plan 10). Usar `global_time` cuando las dos cámaras lo den; hoy la Cam 2 no lo da (ver §8).
- Montar las rutas del estudio en 5002 bajo `/api/studio/*` con `NativePairSource`:
  - La toma es un snapshot nativo emparejado.
  - La previsualización usa la profundidad nativa diezmada (sin JPEG ni zlib).
  - La vista en vivo reutiliza los JPEG que ya codifica el Hub.
- Unificar la alineación: `/api/dual/auto_align` y `/api/studio/align` llaman al mismo servicio, con semillas extra y confirmación de montaje movido.
- Previsualización por WS (`{"type": "preflight", ...}` + máscaras comprimidas) con demanda: solo si hay un cliente con el panel Cámaras abierto.
- **Criterio:**
  - Con 5005 apagado, toda la API del estudio responde en 5002.
  - Previsualización en menos de 60 ms por análisis y sin caída de FPS del stream (benchmark de la Fase 0).
  - Δt de las tomas medido y en `meta.json`.

### U2 — Frontend único (medio-alto)
- `/` sirve el estudio. Shell con paneles **Cámaras · 3D · Tomas · Calibración · Escaneo · Ajustes**, sin iframe ni parche de `main.js`: `viewer.js` y `stream.js` se importan como módulos.
- Panel **Cámaras**: vista en vivo de las dos cámaras con guías en `<canvas>` (zona común, menos de 0,52 m, conflicto, sin profundidad, contorno de la zona visible por ambas, cuadrícula) y el panel de previsualización. El botón de captura refleja el veredicto.
- Panel **3D**: el visor actual con sus modos (dual, solo, solape, sensor único) y los sliders de pose. Los sliders previsualizan y muestran el encaje en vivo, y se guardan con una acción explícita.
- Paneles **Escaneo** y **Ajustes**: portar desde `live3d/web/js/main.js` (1342 líneas), partiéndolo en módulos (F6 del plan 10).
- **Criterio:** checklist de paridad del §2 completa en hardware; se retiran `live3d/web/index.html` (como página aparte) y `test_fotogrametria/web/`.

### U3 — Calibración robusta y persistente (medio)
- `CalibrationStore`:
  - Lock y escritura atómica.
  - Historial de las últimas 20 con origen, calidad, Δt y escena (miniatura).
  - **Deshacer** en un clic.
  - La simulación nunca escribe en disco (ya es así).
- **Alineación multi-toma**: acumular K tomas emparejadas de una escena quieta y resolver una sola pose (ICP conjunto). Objetivo: dispersión de yaw de ±1,5° a menos de ±0,5° en la escena de trabajo.
- Asistente **Calibración inicial**: ChArUco con verificación inmediata con el motor y la previsualización. Es el camino recomendado para montajes muy convergentes o con poca zona común.
- **Detector de montaje movido**: si la previsualización muestra «la pose no encaja» de forma sostenida (p. ej. más de 10 s con zona común suficiente), aviso proactivo de recalibrar.
- **Criterio:** reiniciar no pierde tomas ni historial; deshacer funciona; multi-toma cumple el objetivo de dispersión.

### U4 — Trabajos y operación (bajo-medio)
- `JobQueue` (captura dual, multicaptura, alineación, reproceso) con progreso y cancelación por WS. La petición de captura responde en menos de 300 ms (criterio de la Fase 2 del plan 10).
- `iniciar_sistema.bat`/`.sh`: un proceso y abrir el navegador cuando `/api/status` responda.
- Retirar 5005, el CORS y `ejecutar_servidor_test.py`. Mover `test_fotogrametria/test_cli.py` a `live3d/tools/`. Actualizar CLAUDE.md, el README y los documentos 01–05.
- **Criterio:** un solo proceso, un solo puerto y documentación al día.

### U5 — Blender en vivo (opcional)
- Cliente RSF1 en el addon (operador modal con temporizador) que reciba las dos cámaras y la extrínseca. `live3d/protocol.py` ya se puede importar desde Blender.
- **Criterio:** nube dual en vivo en el visor de Blender a 10 FPS o más, con el mismo marco que el importador.

### Relación con el plan 10

| Plan 10 | En este plan |
|---|---|
| F1 Captura dual eficiente | Independiente; conviene antes o en paralelo a U1 |
| F2 Snapshot sincronizado + captura asíncrona | U1 (ring buffer y snapshot) + U4 (JobQueue) |
| F3 Unificar el estudio | U0–U2 |
| F4 Consolidar la geometría | U0 y U3 (`CalibrationStore`) |
| F5 Calidad de calibración | U3 (multi-toma, ChArUco guiado) |
| F6 Operación | U2 (modularizar `main.js`) y U4 |

---

## 6. Orden y esfuerzo

| Orden | Fase | Esfuerzo | Riesgo | Por qué en este orden |
|---|---|---|---|---|
| 1 | U0 | Bajo | Bajo | Deja el estudio listo para montarse en cualquier servidor sin romper nada |
| 2 | U1 | Medio | **Medio** (toca los workers de captura) | Datos nativos y emparejados: la calidad de calibración depende de ello |
| 3 | U3 | Medio | Bajo | La calibración persistente y multi-toma tiene más impacto que la interfaz |
| 4 | U2 | Medio-alto | Medio (paridad de funciones) | Con el backend estable, la migración de la interfaz es mecánica |
| 5 | U4 | Bajo-medio | Bajo | Cierre: un proceso y retirada de 5005 |
| 6 | U5 | Medio | Bajo | Opcional |

---

## 7. Riesgos

- **CPU en un solo proceso.** Previsualización, alineación y fusión comparten proceso con los workers de captura. Mitigación: la previsualización solo corre con demanda y a ~3 Hz sobre ~25 k píxeles (hoy unos 60 ms por análisis), y lo pesado va a la cola de trabajos. Si el benchmark muestra caída de FPS, se pasa a `ProcessPoolExecutor`.
- **Paridad de la interfaz.** `main.js` tiene muchas funciones sin tests. Mitigación: checklist de paridad en hardware y mantener 5005 hasta cerrar U2.
- **Montaje no rígido.** Ninguna mejora de software compensa unas cámaras que se mueven (medido el 19-10: 3,6° y 7° en 20 minutos). Mitigación: fijar el montaje (§8) y el detector de montaje movido (U3).
- **Windows y USB.** `hardware_reset` y la regla de un proceso por cámara. Mitigación: es justo lo que resuelve tener un único proceso dueño de las cámaras.

---

## 8. Prerrequisitos de hardware (antes de U1)

1. **Fijar el montaje** de las dos cámaras (trípode o soporte rígido). Hoy cambia entre sesiones y dentro de la misma sesión.
2. **Metadatos de cuadro de la Cam 2 en Windows** (`realsense_metadata_win10.ps1` de librealsense, como administrador). Sin ellos la Cam 2 no da timestamps `global_time` y el emparejamiento solo puede usar la hora de llegada.
3. **Firmware** de la Cam 2: de 5.12 a 5.17.
4. **Emisor IR**: probar ambos encendidos y comparar la previsualización y la consistencia en la misma escena (plan 10 §6.4).

---

## 9. Criterio global de éxito

- Un comando, un proceso, una página.
- Antes de capturar, la página dice qué calidad cabe esperar y dónde colocar el sujeto. Después, el resultado de la alineación coincide con esa previsión: misma escena y pose, misma calidad o mejor.
- Las tomas y calibraciones sobreviven a un reinicio y se pueden deshacer.
- Calibración repetible en la escena de trabajo: consistencia de al menos el 85 % y dispersión de yaw por debajo de ±0,5° con multi-toma.
