# Plan 10: Optimización y Consolidación para Pre-Producción

> **Rama:** `Pre_Produccion`
> **Fecha:** 2026-10-19
> **Estado:** En curso — Fase 0 hecha salvo mover el código legado (línea base sintética y con hardware medidas); iteración 2 (§6) implementada y probada con las dos D455. Fases 1–6 pendientes.

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
| C12 | **El TSDF dual no funciona**: con Open3D 0.20.0 + numpy 2.5, `ScalableTSDFVolume` devuelve 0 vértices incluso con un plano trivial (`UniformTSDFVolume` sí funciona con los mismos datos). `capture_dual_scan` cae siempre, sin avisar, en el respaldo de Poisson. *Verificado el 2026-10-19; cubierto por un test `xfail` en `tests/test_geometria.py`.* | `registration.py` L247, `capture.py` L1214-1230 | La "malla TSDF sin duplicados" del plan 07 nunca se genera: sale Poisson sobre la nube voxelizada. |
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

**Estado (2026-10-19):** benchmark y tests de humo hechos. Falta la línea base con hardware y mover el código legado.

- `python tools/bench_live3d.py [--spawn] [--capture] [--json archivo]`
- `python -m pytest`: hay que instalar `requirements-dev.txt`. Recuento actual en §6.2.

Línea base **sintética** (`--spawn`, 10 s, zlib+JPEG, misma máquina). Sirve para medir regresiones de código, no el rendimiento del hardware:

| Métrica | Cam 1 | Cam 2 |
|---|---|---|
| FPS recibidos | 21.7 | 21.7 |
| Mb/s | 22.1 | 21.8 |
| KB/cuadro (depth / color) | 102.6 / 21.0 | 102.6 / 19.1 |
| Latencia p50 / p95 | 4.5 / 6.6 ms | 4.3 / 7.0 ms |
| Servidor | `proc_ms` 31.1 · `encode_ms` 4.2 | |
| Captura dual | 2.69 s (malla vía Poisson, por C12) | |

Notas: la fuente sintética está limitada por el trazado de rayos en numpy (no llega a 30 FPS). `camera_fps` del status (36.5) suma las dos cámaras porque `SyntheticDualSource` llama a `_tick_fps` en cada `_emit`. zlib nivel 1 solo comprime la profundidad unas 2:1.

Línea base **con hardware** (2026-10-19, `python tools/bench_live3d.py --seconds 20 --capture`, 2× D455 por USB 3.2, 848×480@30, código de la iteración 2, un solo cliente WS):

| Métrica | Cam 1 (master) | Cam 2 (slave) |
|---|---|---|
| FPS recibidos | 30.0 | 30.0 |
| Mb/s | 52.6 | 45.0 |
| KB/cuadro (depth / color) | 160.1 / 53.6 | 133.2 / 49.7 |
| Latencia p50 / p95 | 8.0 / 11.3 ms | 7.4 / 10.3 ms |
| Servidor | `proc_ms` 13.5 · `encode_ms` 7.4 · 0 descartes/s | |
| Captura dual | 2.21 s (164 407 puntos; malla vía Poisson, por C12) | |

Frente al modo sintético, cada cuadro real pesa ~2× (848×480 sin decimar, C2): ~98 Mb/s en total con un solo cliente. Esta es la referencia "antes" de la Fase 1.

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

---

## 6. Iteración 2 — Estados veraces y alineación coherente con el sensor (2026-10-19)

### 6.1 Hallazgos nuevos

| # | Problema | Estado |
|---|---|---|
| N1 | El alineador del estudio (`calibrador.py`) tenía **la pose del montaje cableada** (Tx ∈ [-115, -15] cm, Yaw ∈ [15°, 80°] y una pose "base" fija). Cuando no encontraba nada válido devolvía esa pose fija con `success=True`. Cualquier otro montaje, incluida la simulación, "se alineaba" mal sin avisar. | Resuelto: motor nuevo |
| N2 | Kabsch sobre **todos** los pares SIFT sin RANSAC (un solo outlier basta para arruinarlo). La profundidad se muestreaba con intrínsecas de color aunque no estuviera alineada al color. | Resuelto |
| N3 | Escena sintética **incoherente entre cámaras**: la ondulación de la pared dependía del rayo de cada cámara, no del mundo, así que dos cámaras simuladas nunca podían alinearse. El snapshot sintético etiquetaba el color con intrínsecas que no le correspondían. | Resuelto |
| N4 | Los sliders del estudio **guardaban en disco** en cada movimiento. Además, una fuente simulada podía sobrescribir `config/dual_camera_config.json` (incluidas las cámaras y sus seriales). | Resuelto |
| N5 | El indicador del estudio decía siempre "Entorno 3D Activo", hubiera cámaras o no. El estado dual se marcaba `streaming` antes de que arrancara ninguna cámara y un worker caído no se reintentaba. | Resuelto |
| N6 | El RANSAC de Open3D no tenía semilla: el mismo par daba poses distintas en cada ejecución. | Resuelto |
| N7 | `test_cli.py` no arrancaba (constructor con argumentos inexistentes). `/api/align` bloqueaba el event loop y la vista previa MJPEG se recodificaba en cada cuadro (E2). | Resuelto |

### 6.2 Qué se implementó

- **`live3d/alignment.py`**: motor único para 5002 (`/api/dual/auto_align`) y 5005 (`/api/align`).
  - Modelo de ruido del D455: σz = z²·σd / (f·b), con b = 95 mm, Min-Z 0,52 m y rango de calibración hasta 3 m. Todos los umbrales salen de σz.
  - SIFT con CLAHE, ratio de Lowe y verificación mutua. La elevación a 3D respeta las extrínsecas depth→color y descarta los keypoints en bordes de profundidad. Después, RANSAC rígido y reajuste ponderado por 1/σ².
  - Tres hipótesis: SIFT, calibración vigente y FPFH global. Cada una se refina con ICP point-to-plane con kernel de Tukey.
  - Evaluación por **consistencia proyectiva**: cada punto se proyecta en la imagen de profundidad de la otra cámara y se comprueban la distancia (3σ), la violación de espacio libre y la orientación de la superficie (descarta soluciones espejo).
  - Veredicto `buena` / `aceptable` / `insuficiente`. Si es insuficiente, **no se devuelve matriz** y se conserva la calibración vigente.
  - Una pose que se aleja más de 5 cm o 5° de la calibración vigente solo se acepta con evidencia fuerte (calidad buena y fotogrametría coherente). Si no, pide confirmación.
- **Estado por cámara** (`capture.py`): `_DualMixin` común a la fuente real y a la simulada.
  - Estados `starting`, `streaming`, `stalled`, `error` y `no_device` por cámara, y agregado `degraded` cuando solo una está en vivo.
  - Reconexión por cámara con `hardware_reset` tras fallos repetidos.
  - Snapshot dual con Δt real (timestamp de hardware si ambos están en dominio global).
  - No se alinea ni se copia un cuadro que nadie necesita.
  - Guardado atómico de extrínsecas con validación de rotación y metadatos de cómo se obtuvieron.
- **Servidor 5002**: selección de fuente centralizada (`choose_source`). **Vigilante de dispositivos**: si arrancó en simulación por falta de cámaras, cambia solo a las reales al conectarlas. CORS para el estudio. Mensaje `source_changed` al cliente.
- **Estudio 5005**: estado real en la cabecera y en el aviso de sistema. Toma rechazada si una cámara está congelada o desincronizada. Alineación fuera del event loop. Sliders solo de previsualización, con Guardar y Restaurar explícitos.
- **Tests**: 41 (39 pasan, 1 `xfail` por C12 y 1 que se omite si hay RealSense conectadas). Incluyen la recuperación de la verdad sintética con y sin semilla, el determinismo, los casos sin profundidad, desincronizados y sin relación (sin falsos positivos), el estado `no_device` sin hardware y el guardado atómico.

### 6.3 Resultados con datos reales (escaneos duales del 2026-10-16)

`python test_fotogrametria/test_cli.py --all-saves`, con la calibración usada en cada toma como semilla:

- En 3 de 8 tomas el motor **confirma** la calibración vigente, con una diferencia de 1,0–1,5 cm y 0,9–1,8°.
- Las otras 5 se **rechazan** con su motivo: sin solape, o pose contradictoria sin evidencia. Antes, una de ellas se habría aceptado con una pose a 2 m y 79° de la real.
- **Sin semilla, ninguna toma real basta para calibrar desde cero.** Con el montaje a ~60–70° de convergencia hay muy pocos pares SIFT con profundidad (0–12 por toma), porque SIFT pierde invariancia a partir de ~30–40° de cambio de vista. Con una persona como escena, la geometría sola es ambigua.

### 6.3 bis Prueba en vivo con las dos D455 (2026-10-19)

- 5002 arranca en modo dual real, con las dos cámaras en `streaming` a 30 FPS y la calibración vigente (2026-10-16) cargada.
- Estudio 5005: la cabecera muestra "2 cámaras en vivo" y la toma y la galería funcionan. La alineación (escena de aula, profundidad mediana 2,5 m) se **rechaza** con motivo: la calibración vigente solo es consistente al 12–40 % en esa escena y FPFH propone una pose a 1,7–3,2 m / ~176° de ella. `/api/dual/auto_align` (snapshot nativo) da el mismo veredicto. No se tocó `config/dual_camera_config.json`.
- **N8 — Δt del snapshot dual falso (resuelto).** Con la Cam 2 fuera del dominio de tiempo global, el respaldo usaba `time.time()` tomado después de filtrar y copiar. En Python 3.12 sobre Windows ese reloj va a saltos de ~15,6 ms, así que `dt_ms` salía 0,0 / 15,6 aunque los cuadros estuvieran separados 30–60 ms. Ahora se usa `perf_counter()` justo después de `wait_for_frames`, y el resultado indica `dt_source` (`hardware (global_time)` / `llegada al host`), que también se guarda en `metadata_dual.json`. Medido después del arreglo: 15–41 ms.
- La fuente dual real publica `profile` en el status (antes salía vacío en el visor y en el benchmark).

### 6.3 ter Montaje cambiado de lado: búsqueda de montaje y confirmación (2026-10-19)

- **N9 — La calibración guardada estaba estropeada.** `config/dual_camera_config.json` tenía Tx = -95 cm (con fecha y métricas de otra calibración). Lo más probable es que lo dejaran así los sliders antiguos, que guardaban en cada movimiento (N4). La buena del 16-10 era (-55, +4, +20) cm con yaw +63,5°.
- **N10 — Las cámaras cambiaron de lado.** Con el montaje del 19-10, la Cam 2 está a la **derecha** de la Cam 1 (en el sujeto, los brazos van a la derecha de la imagen en la Cam 1 y a la izquierda en la Cam 2). Pose medida en tres pares en vivo: t ≈ (+44,4, +6,1, +41,6) cm, yaw -64,5° (±1,5°), pitch +7,4°, roll +1,7°, con consistencia del 87–89 % y RMSE de 4–10 mm. El motor no podía llegar ahí: SIFT da 0–1 inliers a ~65°, la semilla guardada estaba en el lado contrario y FPFH daba poses absurdas. Y aunque llegara, la habría rechazado por alejarse de la calibración vigente sin evidencia fotogramétrica, sin que el estudio ofreciera forma de confirmarlo.
- **Cambios:**
  - Búsqueda de montaje (`mount_search`): 125 poses genéricas (paralelas o convergentes, a ambos lados) cribadas con ICP grueso y consistencia proyectiva, con tope de 8 s. Solo se lanza si ninguna otra hipótesis es buena.
  - Semillas extra: la pose de los sliders se envía al alinear si se movieron.
  - El estudio muestra **«Moví las cámaras: aplicar esta pose»** cuando la geometría es buena pero contradice la calibración guardada (`needs_confirmation` + `quality_if_confirmed`). Al pulsarlo se vuelve a alinear sin la calibración vieja como referencia.
  - Los sliders arrancan en 0, sin una pose de montaje cableada.
- **N11 — Falso positivo detectado en hardware y corregido.** Con el sujeto tapando una cámara a menos de 0,5 m, la búsqueda aceptó como «buena» una pose con Ty = -3 m y roll -171°. Tenía dos causas:
  - La σ del umbral se calculaba con la profundidad *predicha* sin acotar: a 300 m, σ ≈ 200 m y todo «encajaba». Ahora se acota a `z_max`.
  - Comparaciones múltiples: entre 125 poses, alguna encaja por azar en poca superficie. Ahora las hipótesis a ciegas (FPFH y búsqueda) exigen ≥ 1500 puntos comunes (~0,15 m²) y se rechazan como **ambiguas** si otra pose distinta (> 5 cm / 5°) explica la escena con menos de 10 puntos de diferencia. También se descartan poses no rígidas o con |t| > 2·`z_max`.
- **Montaje no rígido.** Entre dos grabaciones separadas 20 minutos, el fondo se desplazó 27 px en la Cam 1 (~3,6°) y 54 px en la Cam 2 (~7°). Cualquier calibración caduca en cuanto se toca una cámara: **fijar el montaje es requisito previo**.

### 6.4 Recomendaciones de operación (según el sensor)

1. **Calibración inicial con ChArUco** (ya existe en 5002) y la alineación automática para verificar y refinar. Es lo único fiable para un montaje tan convergente.
2. **Emisor IR de la Cam 2**: está apagado en la config (`emitter_enabled: false`). Así su profundidad depende de la textura ambiente o del patrón de la Cam 1. Intel documenta que varias D4xx no se degradan de forma apreciable entre sí. **Probar ambos encendidos** y comparar consistencia y RMSE con la misma escena antes de decidir.
3. **Firmware**: igualar la Cam 2 (5.12) con la Cam 1 (5.17).
4. Escena de calibración: objetos con volumen y textura única en la zona común, a 0,8–2,5 m, y quietos.
5. **Metadatos de cuadro en Windows (Cam 2)**: la Cam 2 no entrega metadatos (`frame_counter` y `sensor_timestamp` no disponibles), así que sus cuadros llevan timestamps `system_time` en vez de `global_time`, aunque `global_time_enabled = 1`. Hay que instalar el registro de metadatos UVC para ese dispositivo con `scripts/realsense_metadata_win10.ps1` de librealsense (como administrador y con las cámaras conectadas). Mientras tanto, el Δt del snapshot se mide por la hora de llegada al host. Esto es un requisito previo de la Fase 2 (emparejamiento por timestamp de hardware).

### 6.5 Pendiente

- Reconexión de los workers (C3): con las dos D455 (2026-10-19) se verificaron el arranque, el estado por cámara (`streaming`, 30 FPS, USB 3.2, emisor) y el snapshot emparejado. **Falta desenchufar y volver a enchufar** una cámara con el sistema en marcha para comprobar `degraded` → recuperación (solo se puede hacer físicamente).
- Emparejamiento del snapshot: hoy se toma el primer cuadro de cada cámara tras la petición, con Δt medido de 15–41 ms. Elegir el par con menor |Δt| de un buffer es la Fase 2.
- Alineación multi-toma: acumular varias tomas emparejadas de una escena quieta para ganar solape.
- El estudio sigue alineando sobre cuadros del stream (JPEG). El snapshot nativo emparejado ya existe en 5002 (`/api/dual/auto_align`), y la Fase 3 unificará ambos.
