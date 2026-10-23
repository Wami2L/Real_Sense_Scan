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
- Montaje muy convergente (~60–70° de yaw) y **no rígido**: cambia a menudo (el 16-10 Cam 2 estaba a la izquierda,
  el 19-10 a la derecha, t ≈ (+44, +6, +42) cm, yaw ≈ -64°). SIFT entre vistas casi no funciona: la alineación se
  apoya en la geometría (búsqueda de montaje + ICP) o en ChArUco. Nunca cablear una pose de montaje en el código.
- Sin cámaras: `python app.py --synthetic [--dual]` (escena trazada con numpy). En dual, Cam 2 está desplazada
  +0.25 m en x sin rotación: es la verdad de referencia de los tests de alineación.
- Si 5002 arranca en simulación por falta de cámaras, cambia solo a las reales al conectarlas (`device_watch_task`).

## Arquitectura actual (tres procesos)
| Proceso | Puerto | Entrada | Rol |
|---|---|---|---|
| Live3D (aiohttp) | 5002 | `app.py` → `live3d/server.py` | Captura, `/ws`, API `/api/*`, visor Three.js y **escáner de objetos `/objetos`** |
| Estudio de calibración (aiohttp) | 5005 | `test_fotogrametria/servidor_test.py` | Cliente WS de 5002, galería de tomas, alineación SIFT+ICP, overlays |
| Enlace Unity (aiohttp) | 5010 | `python -m live3d.unity_link` | Sirve la biblioteca de objetos a Unity; solo 127.0.0.1 o red con token |

### Escáner de objetos (prioridad actual, plan 13) — `live3d/objscan/`
- Ruta: escanear en `/objetos` → (Blender opcional) → publicar → Unity (VR ScanWorld) lo construye con partículas.
- `base.py`: hoja de marcadores ArUco **ids 10-17** (los 0-3 son de calibración). Pose: PnP y luego
  `refine_with_plane` (plano de la hoja por profundidad ∩ rayos de las esquinas + Umeyama = pose y escala real de
  la impresión; el PnP solo, con marcadores pequeños, oscila cm). Varias copias de la hoja: `group_sheets` (usa
  la más horizontal). PDFs en `imprimibles/` y en `/objetos-ui/base_escaneo_A3.pdf`.
- Marco "base" del objeto: metros, **Y arriba**, diestro, pivote en el centro de la base. Blender: forward -Z / up Y.
  Unity (`.rsm`): z → -z y orden de triángulos invertido (ya hecho al escribir el archivo).
- `optica.py`: rectifica color+profundidad (coeficientes de la lente en las intrínsecas del snapshot) ANTES de nada.
- `segment.py`: objeto = volumen de trabajo (cilindro sobre la base) o, sin base, plano de la mesa + grupo central.
  Usa `D455_OBJ` (Min-Z 0,30 m: a 848×480 la D455 mide desde ~0,34 m; el 0,52 de `alignment.D455` es de 1280×720).
- `rafaga.py`: cada toma = mediana de 6 cuadros de profundidad (descarta los movidos: umbral relativo a la mediana
  de la ráfaga, la profundidad alineada a HD cambia 2-3 % en bordes aun quieta). `api.tomar` usa `capture_burst`.
- `api.encuadre`: las dos cámaras (imagen y origen de la pose: hoja / calibración / mesa) y, si ambas ven la hoja,
  la diferencia con la calibración dual; `POST /api/objscan/calibrar_con_hoja` la guarda.
- `session.py`: poses base > calibración (Cam 2) > icp > referencia. ICP **point-to-plane** (el ICP con color sesga
  con fija y objeto que gira). Dual: UNA corrección común de la calibración con todas las tomas (consenso) y la
  Cam 2 se descarta si no encaja. TSDF **uniforme** solo del objeto, sin la "falda" de papel (`_skirt_mask`).
  Rechaza una toma desde la misma pose que otra (objeto girado sobre la hoja quieta). `from_folder` reprocesa
  una sesión guardada (`tools/reprocesar_objeto.py`). Resultado a `Saves/objetos/<id>/`.
- `textura.py`: atlas UV + fotos de las tomas → textura 2048² (**una vista por isla**: mezclar vistas duplica
  detalles en superficies negras/brillantes). `split_by_uv` para el RSM.
- `library.py` (manifiesto `objeto.json`, `objeto.obj`+`textura.jpg`, `objeto_limpio.ply` de Blender gana y se
  re-texturiza con las fotos de la sesión), `rsm.py` (formato Unity; v2 = UV + textura JPEG),
  `sintetico.py` + `fuente.py` (escena con verdad de referencia; `python app.py --synthetic-object`).

- `live3d/capture.py`: fuentes `RealSenseSource`, `DualRealSenseSource`, `SyntheticSource`, `SyntheticDualSource`.
  Dual: color a 1280×800 (`COLOR_PROFILE` o `color_profile` por cámara) con la profundidad alineada en los
  snapshots; el stream en vivo se reduce a 848 de ancho. `set_depth_options(preset, laser_power, emitter)` en
  caliente (`POST /api/objscan/sensor`). Medido: High Accuracy y láser al máximo empeoran en metal brillante.
  Llaman a `on_frame(CapturedFrame)` desde su hilo; snapshots vía `get_snapshot()` (petición al hilo de captura).
- `live3d/protocol.py`: protocolo binario `RSF1` (cabecera 32 B, profundidad uint16 zlib + color JPEG/raw, meta JSON).
  `flags` bit 0 = índice de cámara. Solo depende de stdlib+numpy (debe seguir importable desde Blender).
- `live3d/alignment.py`: **motor único** de alineación Cam2→Cam1 (SIFT 3D + RANSAC, FPFH, ICP robusto y
  consistencia proyectiva con el modelo de ruido del D455). Lo usan `/api/dual/auto_align` (5002) y `/api/align` (5005).
  Nunca devuelve matriz si el resultado no es fiable. `test_fotogrametria/calibrador.py` es solo un alias.
  `preflight()` predice la calidad antes de capturar (zona común, < Min-Z, movimiento, encaje con la pose) y da
  las máscaras de las guías que el estudio pinta sobre la vista en vivo (`/api/preflight`, `/api/guides`).
  Evidencia inequívoca: marcadores ArUco `DICT_4X4_50` (esquinas 3D por plano del papel; hoja en
  `tools/generar_marcadores.py`) y puntos a mano (`manual_pairs`, Kabsch robusto). `alignment_error_map` y
  `combine_poses` (multi-toma). Historial de calibraciones con deshacer: `extrinsics_history` en el JSON.
- `live3d/registration.py`: convención de Euler (única), snapshot→nube, TSDF dual y nube de diagnóstico.
- Estado por cámara: `_DualMixin.cameras_status()` → `status.camera.cams[]`, con agregado
  `streaming|degraded|no_device|error|starting` y `simulated: true/false`.
- Extrínsecas: escribirlas siempre con `src.set_extrinsics(T, **metadatos)` (atómico; la simulación no toca disco).
- `multiscan.py`: sesión multi-vista (pose graph + Poisson). `generar_malla_offline.py`: Delaunay 2.5D.
- `RealSense_Importer.py`: addon de Blender ≥ 4.0 (probado en 4.5 y 5.2) que importa scan_/dual_scan_/multi_ con ejes,
  marcos (Cam 2 y vistas con su pose), cámaras y color, y panel **Objetos escaneados** (abrir, limpieza rápida que cierra
  la base, «Publicar en Unity» = `objeto_limpio.ply` triangulado + versión +1). `script_blender.py -- --objeto <id>`
  lo usa el botón «Abrir en Blender» de /objetos.
  Usan `bpy`: no importarlos fuera de Blender. Probar con `blender -b --factory-startup --python <script>`.
- Convención extrínseca: `T_cam2_to_cam1` (4×4, metros), `X_cam1 = T · X_cam2`. Euler: `R = Ry(yaw)·Rx(pitch)·Rz(roll)`,
  idéntica en `registration.py`, `test_fotogrametria/calibrador.py` y `live3d/web/js/main.js`.
- Coordenadas RealSense: x derecha, y abajo, z adelante. Three.js: (x,-y,-z). Blender: (x,z,-y).

## Proyecto Unity asociado
- `C:\Users\Usuario\Documents\Proyectos Unity\VR ScanWorld` — Unity **6000.3.8f1**, URP 17.3, solo Input System nuevo,
  espacio lineal. Git propio: https://github.com/Wami2L/Unity_Realsense_Reciever (rama `main`; G0 del plan 15 en la
  rama `gsplat`). XRI 3.6.1 + OpenXR 1.16.1 (+ Oculus 4.5.5 instalado pero sin loader).
- VR (G0, plan 15): **OpenXR en las dos plataformas**. Windows = D3D12 (+Vulkan), SPI, perfiles Vive/Oculus Touch/Index
  (Vive Pro por SteamVR, Quest 2 por Link). Android = solo Vulkan, multiview, Meta Quest Support + foveación SRP,
  IL2CPP ARM64, minSdk 32, id `com.wami2l.vrscanworld`. Build profiles `Assets/Settings/BuildProfiles/{Quest2,PCVR}.asset`
  (defines `SCANWORLD_QUEST2` / `SCANWORLD_PCVR`). Todo lo aplica `ScanWorldXRSetup.Configure` (idempotente; menú
  ScanWorld › Configurar VR o `-executeMethod ScanWorld.EditorTools.ScanWorldXRSetup.BatchConfigure`); APK con
  `...BatchBuildQuest2` (→ `Builds/Quest2/`), PC con `...BatchBuildPcVr`. El editor en batch arranca en Vulkan aunque
  D3D12 sea el primero de la lista: para probar D3D12 usar `-force-d3d12`.
- **gsplat** (wuyize, v1.4.0, commit `a2bf458`) vendorizado en `Packages/wu.yize.gsplat/`, con `Gsplat URP Feature`
  en `PC_Renderer` y `Mobile_Renderer`. Cambios propios en commits aparte con prefijo `gsplat:`. **No lee PLY de más
  de 2 GB**: partir (`tools/splats.py limpiar` lo hace solo) y usar la ordenación global.
- Banco de splats: `-executeMethod ScanWorld.EditorTools.GsplatBench.Run -benchPly "etiqueta=a.ply|b.ply;..."
  -benchCams camaras.json -benchOut <dir>` (sin `-quit`; un render por tick del editor porque
  `Graphics.RenderMeshPrimitives` acumula dibujos si no termina el frame).
- Escenas 3DGS: `live3d/splats/` (PLY por bloques, análisis, limpieza, poda por estación, LOD por fusión) + `tools/splats.py`;
  datos en `Saves/entornos/<nombre>/` (ignorado). La escena de prueba es CC BY-NC-ND: no distribuir sus variantes.
- `.venv-splats/` (ignorado): Python 3.10 + torch 2.4.1 cu124 + gsplat 1.5.3 precompilado, solo para `tools/destilar.py`
  (en Windows no hay ruedas de gsplat para 3.12 y no hay MSVC para compilarlo). Instalación en el plan 15 §12.6.
- Paquete `Assets/ScanWorld/` (ver su `LEEME.md`): `ScanLinkClient` (consulta 5010), `RsmLoader` (RSM v1/v2 con
  textura → `RsmModel`), `ScanGallery`
  (pedestales), `ScanBuildEffect` + shaders URP `ScanWorld/BuildParticles` y `ScanWorld/BuildSolid` (sin VFX Graph ni
  geometry shaders), importador `.rsm`, menú ScanWorld. Escena `Assets/ScanWorld/Scenes/ScanWorld.unity`.
- Verificar sin abrir el editor (Unity cerrado): `Unity.exe -batchmode -projectPath <proyecto> -executeMethod
  ScanWorld.EditorTools.ScanWorldSetup.BatchCapture` (PNG en `ScanWorldCapturas/`), `...BatchTestLink`, y
  `-runTests -testPlatform PlayMode` (el test del enlace se ignora si 5010 no responde).
- Ejes RealSense (cámara) → Unity: `(x, -y, z)`. Objetos escaneados: ya vienen en Unity dentro del `.rsm`.

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
iniciar_sistema.bat                # lanza 5002 + 5005 + 5010 y abre /objetos
python app.py --synthetic --dual   # desarrollo sin cámaras
python app.py --synthetic-object   # escáner de objetos sin cámaras (plato giratorio simulado)
python -m live3d.unity_link        # enlace con Unity (5010); --red para otros equipos (token)
python tools/generar_base_escaneo.py --hoja A3   # hoja imprimible de la base de escaneo
python tools/reprocesar_objeto.py Saves/<fecha>/objeto_HHMMSS [--reemplazar <id>]  # rehacer un objeto sin cámara
python tools/splats.py info|camaras|limpiar|estacion|lod|comparar ...  # escenas 3DGS (plan 15 §7, §12)
.venv-splats\Scripts\python tools/destilar.py <lod.ply> --maestro <limpio.ply>  # destilación (PyTorch+gsplat, plan 15 §12.6)
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
`documentacion/planes/11_Plan_Ecosistema_Unificado_Estudio_Live3D.md` — siguiente paso: estudio y Live 3D en un solo
servidor con el estudio como interfaz principal (fases U0–U5).
`documentacion/planes/12_Plan_Alineacion_Guiada_Marcadores_y_Puntos.md` — alineación guiada (implementada; §6 = qué refinar).
`documentacion/planes/13_Plan_Escaner_de_Objetos_a_Unity.md` — **prioridad**: escáner de objetos → Blender → Unity.
`documentacion/planes/14_Plan_Escaneo_Texturizado_RealSense.md` — geometría RealSense + textura fotográfica (vigente).
`documentacion/planes/15_Plan_Biblioteca_VR_y_Entornos_Gaussian_Splatting.md` — **fin del proyecto** (biblioteca infinita en
VR dentro de entornos 3DGS) y plan G0–G6 para 3DGS en Unity; base elegida: gsplat (wuyize) vendorizado.
Guía de uso: `documentacion/GUIA_ESCANER_OBJETOS.md`.
