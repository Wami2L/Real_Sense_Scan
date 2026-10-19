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
- Emisor IR: master on, slave off (configurable por cámara en el JSON).
- Una RealSense solo puede abrirla **un proceso**. En Windows, si el proceso muere con el pipeline abierto,
  hace falta `hardware_reset()` + esperar ~5 s.
- Sin cámaras: `python app.py --synthetic [--dual]` (escena trazada con numpy).

## Arquitectura actual (dos procesos)
| Proceso | Puerto | Entrada | Rol |
|---|---|---|---|
| Live3D (aiohttp) | 5002 | `app.py` → `live3d/server.py` | Captura, WebSocket binario `/ws`, API REST `/api/*`, visor Three.js |
| Estudio de calibración (aiohttp) | 5005 | `test_fotogrametria/servidor_test.py` | Cliente WS de 5002, galería de tomas, alineación SIFT+ICP, overlays |

- `live3d/capture.py`: fuentes `RealSenseSource`, `DualRealSenseSource`, `SyntheticSource`, `SyntheticDualSource`.
  Llaman a `on_frame(CapturedFrame)` desde su hilo; snapshots vía `get_snapshot()` (petición al hilo de captura).
- `live3d/protocol.py`: protocolo binario `RSF1` (cabecera 32 B, profundidad uint16 zlib + color JPEG/raw, meta JSON).
  `flags` bit 0 = índice de cámara. Solo depende de stdlib+numpy (debe seguir importable desde Blender).
- `live3d/registration.py`: registro Cam2→Cam1 (ICP en cascada, barrido yaw/pitch, FPFH+RANSAC), TSDF dual.
- `multiscan.py`: sesión multi-vista (pose graph + Poisson). `generar_malla_offline.py`: Delaunay 2.5D.
- `RealSense_Importer.py` / `script_blender.py`: lado Blender (usan `bpy`, no importar fuera de Blender).
- Convención extrínseca: `T_cam2_to_cam1` (4×4, metros), `X_cam1 = T · X_cam2`. Euler: `R = Ry(yaw)·Rx(pitch)·Rz(roll)`,
  idéntica en `registration.py`, `test_fotogrametria/calibrador.py` y `live3d/web/js/main.js`.
- Coordenadas RealSense: x derecha, y abajo, z adelante. Three.js: (x,-y,-z). Blender: (x,z,-y).

## Código legado / no usar como referencia
- `app_legacy_flask.py`, `camera_manager.py`, `templates/`, `static/` — interfaz Flask antigua.
- `escanear.py` — script suelto.
- `test_fotogrametria/servidor_test.py` reescribe `main.js` por reemplazo de strings para apuntar a 5002 (frágil).

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
```
- Los tests y el benchmark con `--capture` escriben en `Saves/` (los tests limpian lo que crean).
- Open3D 0.20: `ScalableTSDFVolume` devuelve vacío (C12 del plan 10). `UniformTSDFVolume` sí funciona.

## Plan vigente
`documentacion/planes/10_Plan_Optimizacion_Pre_Produccion.md` — diagnóstico de ineficiencias y fases de mejora.
