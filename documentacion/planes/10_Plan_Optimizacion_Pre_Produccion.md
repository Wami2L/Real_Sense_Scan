# Plan 10: Optimización y Consolidación para Pre-Producción

> **Rama:** `Pre_Produccion`
> **Fecha:** 2026-10-19
> **Estado:** Propuesta — diagnóstico hecho leyendo el código; ninguna cifra de rendimiento se ha medido todavía en hardware (la Fase 0 existe justamente para medirlas).

---

## 1. Resumen

Lo que el sistema hace está bien documentado (planes 01–09), pero el camino que siguen los datos tiene ineficiencias.
Hay cuatro problemas de fondo:

1. **El modo dual no aprovecha las optimizaciones del modo de una cámara.** `RealSenseSource` se diseñó para tener latencia mínima: no alinea en CPU, decima, aplica umbral y solo procesa si hay demanda. `DualRealSenseSource` no hace nada de eso.
2. **La calibración trabaja con datos degradados.** El estudio del puerto 5005 calibra con los cuadros del stream: JPEG con pérdida, profundidad sin filtrar y un par de cuadros no emparejado por timestamp.
3. **Las capturas son síncronas y pesadas.** Una captura dual hace SOR + normales + TSDF + Poisson + diagnóstico antes de responder, y aun así no guarda la profundidad cruda, así que no se puede volver a procesar.
4. **Hay dos procesos y lógica duplicada.** Hay dos servidores, dos clientes WebSocket, tres implementaciones de "snapshot → nube", dos de Euler↔matriz y dos de RANSAC/ICP.

---

## 2. Diagnóstico detallado

### 2.1 Captura dual — `live3d/capture.py`

| # | Problema | Ubicación | Impacto |
|---|---|---|---|
| C1 | `rs.align` en CPU en **cada cuadro de cada cámara**, antes de comprobar si hay clientes (`has_demand()` se evalúa después). | `_cam_worker`, L1075 vs L1142 | La operación más cara de librealsense, ×2 cámaras ×30 FPS, aunque nadie esté mirando. |
| C2 | Perfil fijo 848×480@30 y **se ignoran los `Settings`**: no hay decimación, ni umbral near/far, ni filtros, ni preset. | L1015-1016 | Se envían ~400 k puntos por cámara (unos 4× más que el modo simple con decimación 2): más CPU para zlib/JPEG, más ancho de banda y más carga en la GPU. Los sliders de la UI no tienen efecto en modo dual. |
| C3 | Sin watchdog ni reconexión: si falla `wait_for_frames` se hace `continue` para siempre, y si falla `pipeline.start` el worker muere en silencio. El estado se marca `streaming` antes de que arranque ninguna cámara. | L993, L1067-1072, L1153 | La UI dice "streaming" con una cámara caída. Hace falta reiniciar a mano. |
| C4 | El snapshot dual **no se empareja por timestamp**: cada worker guarda "su" cuadro cuando lo ve. | L1088-1129 | Puede haber 0–33 ms o más entre cámaras: si el sujeto se mueve, aparece efecto fantasma en la fusión y el ICP se sesga. |
| C5 | Mientras espera a la otra cámara, cada worker **reprocesa** spatial+temporal+colorize en cada cuadro y sobrescribe su snap. El `temporal_filter` procesa un solo cuadro por captura, así que no aporta nada. La lectura de `_capture_req` se hace sin lock. | L1088-1131 | CPU desperdiciada y un filtro sin efecto. Posible carrera entre los dos workers. |
| C6 | `_cam_data` se rellena en cada cuadro y nunca se lee. | L1138-1139 | Retención de memoria inútil. |
| C7 | `capture_dual_scan` hace todo el post-proceso de forma síncrona en el executor por defecto: 3 PLY + SOR + normales + TSDF + Poisson de respaldo + diagnóstico. | L1163-1279 | La petición HTTP tarda segundos. Se ocupa el pool compartido con calibración y multiscan. |
| C8 | No se guarda la **profundidad cruda** (solo un JPEG del mapa de calor y los PLY). | L1237-1245, L161-162 | No se puede volver a procesar una captura con mejor calibración o mejores parámetros. |
| C9 | `intrinsic_matrix` en `metadata_camara.json` está **transpuesta** (`ppx, ppy` en la última fila). | L169-173 | Cualquier consumidor externo que lea la matriz tal cual la interpreta mal. |
| C10 | `depth_trunc` incoherente entre módulos: 3.0 (simple), 4.0 (dual) y 2.5 (registro). | L147, L1190, `registration.py` L12 | Resultados distintos según la ruta de código. |
| C11 | `SyntheticDualSource` y `DualRealSenseSource` duplican la lógica de snapshot, meta y modo de vista. | L751-903 | Los cambios hay que hacerlos dos veces y el simulador ya diverge del real (meta con depth→color ≠ identidad). |

### 2.2 Servidor — `live3d/server.py`

| # | Problema | Ubicación |
|---|---|---|
| S1 | Las extrínsecas se escriben en `config/dual_camera_config.json` desde 3 endpoints, sin lock ni escritura atómica, y solo si el archivo ya existe. | L561, L610-612, L660-662 |
| S2 | `api_list_saves` recorre todo `Saves/` (más de 800 MB, cientos de carpetas) en cada llamada. | L444-486 |
| S3 | `api_dual_status` expone atributos privados (`_cam_stats`, `_active_cam_idx`): la API está acoplada a la implementación. | L490-501 |
| S4 | Los trabajos pesados (captura, auto-alineación, multiscan) van al `run_in_executor(None, …)` por defecto, sin cola, sin progreso y sin cancelación. | L326, L527, L599 |

### 2.3 Estudio de calibración — `test_fotogrametria/servidor_test.py` (5005)

| # | Problema | Ubicación |
|---|---|---|
| E1 | Calibra con cuadros del **stream**: color JPEG (pérdida), profundidad sin filtrar y alineada a color, y el último cuadro de cada cámara sin emparejar por timestamp. | `LiveCameraReceiver`, L59-165 |
| E2 | `_update_preview` decodifica 2 JPEG, redimensiona y vuelve a codificar **en cada cuadro dentro del event loop**, aunque nadie esté viendo el MJPEG. | L88-133 |
| E3 | Sirve `live3d/web/js/main.js` **parcheado con `str.replace`** para apuntar a 5002. Se rompe en cuanto cambie una línea de `main.js`. | L281-312 |
| E4 | Las tomas viven en RAM (`captures_store` con `pair_data`) y se pierden al reiniciar. Máximo 15. | L402-470 |
| E5 | Duplica `matrix_to_euler_translation` / `euler_translation_to_matrix` de `live3d/registration.py`. | `calibrador.py` L19-66 |

### 2.4 Calidad de la calibración actual

`config/dual_camera_config.json`: **fitness 0.31, RMSE 5.21 mm** (método "calibración actual + ICP"). Un solape del 31 % es bajo y deja la calibración sensible a la escena. Ya existe un calibrador ChArUco (`tools/calibrate_dual.py`, expuesto en `/api/dual/calibrate`), pero no es el flujo principal. Además, la Cam 2 tiene firmware 5.12 frente a 5.17 de la Cam 1.

### 2.5 Repositorio y operación

- No hay tests ni benchmark: ninguna optimización se puede verificar con números.
- Hay código legado mezclado en la raíz: `app_legacy_flask.py`, `camera_manager.py`, `templates/`, `static/`, `escanear.py`. Flask no está en `requirements.txt`.
- Los PDF regenerables están versionados (19 binarios). `markdown`/`xhtml2pdf` están en las dependencias de ejecución.
- `iniciar_sistema.bat` usa `timeout` fijos en lugar de comprobar que el puerto 5002 responde.
- `live3d/web/js/main.js` tiene 1329 líneas en un solo archivo (no se revisó a fondo en este diagnóstico).

---

## 3. Arquitectura objetivo

```
                 ┌──────────────── proceso único (aiohttp :5002) ────────────────┐
 Cam1 ──worker──►│ CameraWorker (por cámara)                                     │
 Cam2 ──worker──►│   · filtros según Settings · ring buffer N cuadros + hw ts    │
                 │   · watchdog / reconexión independiente                       │
                 │            │                         │                        │
                 │     stream (sin align)        snapshot emparejado (align)     │
                 │            ▼                         ▼                        │
                 │   Hub WS /ws (sin cambios)    JobQueue (1 worker, progreso)   │
                 │            │                    · guardar crudo inmediato     │
                 │            │                    · fusión/TSDF/malla en fondo  │
                 │            ▼                         ▼                        │
                 │   /  visor   /studio  estudio calibración   /api/jobs/<id>    │
                 └───────────────────────────────────────────────────────────────┘
                   live3d/geometry.py  ← única implementación snap→pcd, Euler, ICP
```

---

## 4. Fases

Cada fase debe quedar funcionando en `--synthetic --dual` **y** en hardware antes de pasar a la siguiente.

### Fase 0 — Línea base medible (prerrequisito)
- `tools/bench_live3d.py`: se conecta a `/ws` durante N segundos y registra FPS por cámara, `proc_ms`, `encode_ms`, Mb/s y cuadros descartados. Lo expuesto en `status` ya da casi todo.
- Tests de humo con `pytest` sobre la fuente sintética: arranque del servidor, ida y vuelta de `protocol.pack/decode`, `matrix_to_euler ↔ euler_to_matrix`, y que `capture_dual_scan` genere los archivos esperados.
- Mover el código legado a `legacy/`: `app_legacy_flask.py`, `camera_manager.py`, `templates/`, `static/`, `escanear.py`.
- **Criterio:** tabla de referencia "antes" con hardware real, guardada en este documento.

### Fase 1 — Captura dual eficiente (mayor ganancia)
- Extraer un `CameraWorker` reutilizable a partir de `RealSenseSource` (apertura con perfiles de respaldo, watchdog, `hardware_reset`, filtros según `Settings`). `DualRealSenseSource` pasa a ser N workers más la coordinación.
- Stream **sin `rs.align`**: depth decimada + meta con extrínsecas depth→color reales (como en el modo simple; el shader ya lo soporta).
- Comprobar `has_demand()` **antes** de cualquier procesamiento.
- Estado por cámara real (`starting/streaming/error/no_device`) y agregado en `status.camera.cams[]`.
- Eliminar `_cam_data`. Unificar `SyntheticDualSource` sobre la misma interfaz.
- **Criterio:** con 2 cámaras a 30 FPS, `proc_ms` por cámara ≤ el del modo simple, los sliders afectan al modo dual y desconectar el USB de una cámara se recupera solo.

### Fase 2 — Snapshot sincronizado y captura asíncrona
- Ring buffer de 4 a 6 framesets por cámara con `frame.get_timestamp()` (dominio de reloj global, `global_time_enabled`). El snapshot elige el par con menor |Δt| y reporta `dt_ms` en los metadatos. Se rechaza si supera un umbral configurable (p. ej. 20 ms).
- Alinear y filtrar **solo** los cuadros elegidos. Para el filtro temporal, alimentarlo con los cuadros del buffer o quitarlo.
- `JobQueue` (un hilo, o un proceso para Open3D) con `POST /api/dual/capture` → `{job_id}` y `GET /api/jobs/<id>` → progreso y resultado. La UI hace polling o recibe eventos por el WS de status.
- Formato de guardado reprocesable: `camN_color.png`, `camN_depth.png` (uint16), `meta.json` (intrínsecas, depth_scale, extrínsecas, `dt_ms`, ajustes). Los PLY y la malla son derivados.
- Arreglar la matriz intrínseca transpuesta (C9) y unificar `depth_trunc` en una constante (C10).
- **Criterio:** la petición de captura responde en < 300 ms. Una captura se puede reprocesar con `python -m live3d.reprocess <carpeta>`.

### Fase 3 — Unificar el estudio de calibración en 5002
- Mover `test_fotogrametria/` a `live3d/studio/` y servirlo en `/studio` del mismo servidor. Así no hacen falta el proxy, el parche de `main.js` (E3), el segundo cliente WS ni el MJPEG recodificado (E2).
- Las tomas del estudio usan el snapshot emparejado de la Fase 2 (datos nativos, no JPEG del stream) y se guardan en disco (`Saves/calibracion/…`) con su resultado.
- Vista previa en vivo: reutilizar el visor y los cuadros JPEG ya codificados por el Hub, sin recodificar.
- `iniciar_sistema.bat`/`.sh` lanzan un solo proceso y abren el navegador cuando `/api/status` responde.
- **Criterio:** un solo proceso, un solo puerto, sin E1–E4.

### Fase 4 — Consolidar la geometría
- Crear `live3d/geometry.py` con una única implementación de: snapshot→RGBD/PCD, Euler↔matriz, `_prep`, RANSAC/ICP en cascada, TSDF dual y diagnóstico de solape.
- `registration.py`, `multiscan.py`, `capture.py` y `studio/calibrador.py` lo importan.
- Escritura de extrínsecas centralizada en un `CalibrationStore` (lock + escritura atómica con archivo temporal + `os.replace`), con historial de las últimas N calibraciones (S1).
- **Criterio:** los tests de la Fase 0 siguen pasando y desaparecen las funciones duplicadas listadas en C11/E5.

### Fase 5 — Calidad de calibración
- Flujo recomendado en la UI: **ChArUco** (`tools/calibrate_dual.py`) como estimación inicial, refinada con ICP de color sobre varias tomas emparejadas (no una sola).
- Métricas por calibración: fitness, RMSE, `dt_ms` del par y número de tomas. Se rechazan las que no superen el umbral.
- Actualizar el firmware de la Cam 2 (5.12 → 5.17) para igualar comportamiento y timestamps.
- **Criterio:** fitness ≥ 0.5 y RMSE ≤ 5 mm de forma repetible en la escena de trabajo.

### Fase 6 — Operación y mantenimiento
- `api_list_saves` con caché e índice incremental, o paginado (S2). API de estado dual con un esquema público (S3).
- Separar `requirements.txt` (ejecución) de `requirements-docs.txt` (markdown/xhtml2pdf). Fijar las versiones probadas.
- Sacar los PDF de git y generarlos bajo demanda o en CI.
- Partir `main.js` en módulos (UI de calibración, panel de capturas, ajustes).

---

## 5. Orden recomendado y riesgos

| Orden | Fase | Esfuerzo | Riesgo | Por qué en este orden |
|---|---|---|---|---|
| 1 | F0 | Bajo | Bajo | Sin medir no se puede demostrar ninguna mejora. |
| 2 | F1 | Medio | **Medio**: toca la captura en hardware | Mayor ahorro de CPU/ancho de banda. Desbloquea F2. |
| 3 | F2 | Medio | Medio | Arregla la causa raíz de los "fantasmas" y la lentitud de captura. |
| 4 | F3 | Medio | Bajo | Simplifica la operación diaria. Depende de F2. |
| 5 | F4 | Bajo | Bajo | Limpieza, mejor con los tests de F0 ya en marcha. |
| 6 | F5 | Medio | Bajo | Mejora de calidad sobre una base ya estable. |
| 7 | F6 | Bajo | Bajo | Se puede ir haciendo en paralelo. |

**Riesgos:**
- El shader del visor espera depth→color de cada cámara. Al quitar `align` en dual (F1) hay que verificar la reproyección del color en las dos cámaras.
- La regla de "un proceso por cámara" hace que F3 sea obligatoria si el estudio necesita snapshots nativos.
- Open3D libera el GIL solo en parte: si la `JobQueue` en hilo degrada el stream, pasarla a `ProcessPoolExecutor`.
