# CLAUDE.md — RealSenseScans

Contexto para agentes que trabajen en este repo. Idioma del proyecto: **español** (código, comentarios, docs y UI).

## Flujo de ramas
- `main` ← `Pre_Produccion` ← `testing` / `Current_Changes`.
- El trabajo de mejora se hace en **`Pre_Produccion`**. No hacer push ni merge a `main` sin confirmación.

## Hardware real
- 2× Intel RealSense **D455** por USB 3.2 (seriales en `config/dual_camera_config.json`):
  Cam 1 master `207322250777` (FW 5.17), Cam 2 slave `203522251709` (FW 5.12 — desactualizado).
- **Sin cable de sincronía**: `inter_cam_sync_mode = 0` siempre. El modo 2 sin cable congela el sensor.
- `sync_mode` no se puede cambiar con el pipeline activo.
- Emisor IR: hoy master on, slave off (configurable por cámara en el JSON); pendiente probar ambos on (plan 10 §6.4).
- Una RealSense solo puede abrirla **un proceso**. En Windows, si el proceso muere con el pipeline abierto,
  hace falta `hardware_reset()` + esperar ~5 s.
- La Cam 2 no entrega metadatos de cuadro en Windows (registro UVC sin instalar) → timestamps `system_time`, no
  `global_time`; el Δt del snapshot dual se mide entonces por llegada al host (`dt_source` en el resultado).
- Rango útil del D455 para calibrar: 0.52–3.0 m; ruido σz ≈ z²·0.1/(447·0.095) m (crece con z²).
- Montaje actual muy convergente (~60–70° de yaw): SIFT entre vistas casi no funciona → la calibración inicial
  se hace con ChArUco; la alineación automática verifica y refina.
- Sin cámaras: `python app.py --synthetic [--dual]` (escena trazada con numpy). En dual, Cam 2 está desplazada
  +0.25 m en x sin rotación: es la verdad de referencia de los tests de alineación.
- Si 5002 arranca en simulación por falta de cámaras, cambia solo a las reales al conectarlas (`device_watch_task`).

## Arquitectura actual (dos procesos)
| Proceso | Puerto | Entrada | Rol |
|---|---|---|---|
| Live3D (aiohttp) | 5002 | `app.py` → `live3d/server.py` | Captura, WebSocket binario `/ws`, API REST `/api/*`, visor Three.js |
| Estudio de calibración (aiohttp) | 5005 | `test_fotogrametria/servidor_test.py` | Cliente WS de 5002, galería de tomas, alineación SIFT+ICP, overlays |

- `live3d/capture.py`: fuentes `RealSenseSource`, `DualRealSenseSource`, `SyntheticSource`, `SyntheticDualSource`.
  Llaman a `on_frame(CapturedFrame)` desde su hilo; snapshots vía `get_snapshot()` (petición al hilo de captura).
- `live3d/protocol.py`: protocolo binario `RSF1` (cabecera 32 B, profundidad uint16 zlib + color JPEG/raw, meta JSON).
  `flags` bit 0 = índice de cámara. Solo depende de stdlib+numpy (debe seguir importable desde Blender).
- `live3d/alignment.py`: **motor único** de alineación Cam2→Cam1 (SIFT 3D + RANSAC, FPFH, ICP robusto y
  consistencia proyectiva con el modelo de ruido del D455). Lo usan `/api/dual/auto_align` (5002) y `/api/align` (5005).
  Nunca devuelve matriz si el resultado no es fiable. `test_fotogrametria/calibrador.py` es solo un alias.
- `live3d/registration.py`: convención de Euler (única), snapshot→nube, TSDF dual y nube de diagnóstico.
- Estado por cámara: `_DualMixin.cameras_status()` → `status.camera.cams[]`, con agregado
  `streaming|degraded|no_device|error|starting` y `simulated: true/false`.
- Extrínsecas: escribirlas siempre con `src.set_extrinsics(T, **metadatos)` (atómico; la simulación no toca disco).
- `multiscan.py`: sesión multi-vista (pose graph + Poisson). `generar_malla_offline.py`: Delaunay 2.5D.
- `RealSense_Importer.py` / `script_blender.py`: lado Blender (usan `bpy`, no importar fuera de Blender).
- Convención extrínseca: `T_cam2_to_cam1` (4×4, metros), `X_cam1 = T · X_cam2`. Euler: `R = Ry(yaw)·Rx(pitch)·Rz(roll)`,
  idéntica en `registration.py`, `test_fotogrametria/calibrador.py` y `live3d/web/js/main.js`.
- Coordenadas RealSense: x derecha, y abajo, z adelante. Three.js: (x,-y,-z). Blender: (x,z,-y).

## Código legado / no usar como referencia
- `app_legacy_flask.py`, `camera_manager.py`, `templates/`, `static/` — interfaz Flask antigua.
- `escanear.py` — script suelto.
- `test_fotogrametria/servidor_test.py` reescribe `main.js` por reemplazo de strings para apuntar a 5002 (frágil;
  5002 envía CORS para `localhost:5005` mientras no se unifiquen).

## Datos
- `Saves/AAAA-MM-DD/<tipo>_HHMMSS/` (scan_, dual_scan_, multi_) — ignorado por git, puede pesar >800 MB.
- `test_fotogrametria/web/captures/` — capturas del estudio, ignoradas.
- Los PDF de `documentacion/` se regeneran desde los `.md` (`documentacion/convertir_todo.py`); editar el `.md`.

## Comandos
```bash
instalar_dependencias.bat          # crea .venv e instala requirements.txt
iniciar_sistema.bat                # lanza 5002 + 5005 y abre el navegador
python app.py --synthetic --dual   # desarrollo sin cámaras
python -m live3d.tools.probe_client  # cliente WS de prueba
python -m pip install -r requirements-dev.txt
python -m pytest                   # tests de humo (lanzan su propio servidor sintético en un puerto libre)
python tools/bench_live3d.py --spawn --capture   # benchmark sintético; sin --spawn mide el servidor en 5002
python test_fotogrametria/test_cli.py --all-saves # motor de alineación sobre los escaneos duales reales guardados
```
- Los tests y el benchmark con `--capture` escriben en `Saves/` (los tests limpian lo que crean).
- Open3D 0.20: `ScalableTSDFVolume` devuelve vacío (C12 del plan 10). `UniformTSDFVolume` sí funciona.

## Plan vigente
`documentacion/planes/10_Plan_Optimizacion_Pre_Produccion.md` — diagnóstico de ineficiencias y fases de mejora.
