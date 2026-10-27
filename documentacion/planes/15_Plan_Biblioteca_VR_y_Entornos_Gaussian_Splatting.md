# Plan 15: Biblioteca infinita en VR y entornos Gaussian Splatting en Unity

> **Rama:** `Pre_Produccion` · **Proyecto Unity:** `VR ScanWorld` (repo `Unity_Realsense_Reciever`, rama `main`)
> **Fecha:** 2026-10-22 (§11 Quest 2 + Vive Pro: 2026-10-23) · **Estado:** bases del proyecto + investigación;
> implementación por fases (G0–G6)
> **Sigue a:** [plan 13](13_Plan_Escaner_de_Objetos_a_Unity.md) y [plan 14](14_Plan_Escaneo_Texturizado_RealSense.md)
> (escáner de objetos → Unity)

---

## 1. Para qué existe este proyecto

### 1.1 Visión

**RealSenseScans + VR ScanWorld es una forma de interactuar con la realidad desde la realidad virtual.**
Los objetos del mundo real se escanean (RealSense D455, plan 13-14) y llegan a un entorno VR donde el usuario
**los recibe, los toma, los examina y los ordena** dentro de una **biblioteca infinita**: un espacio que crece con
cada objeto nuevo que se escanea. Nada en la biblioteca está modelado a mano: cada pieza es una copia de algo
que existe.

La biblioteca no flota en el vacío: se construye **dentro de lugares reales escaneados previamente con Gaussian
Splatting (3DGS)**, que dan la ambientación (una sala, un taller, un exterior). El 3DGS reproduce la luz, los
reflejos y los detalles finos de un lugar con un realismo que la malla texturizada no alcanza, y es la piel del
mundo; los objetos escaneados son el contenido con el que se interactúa.

### 1.2 Las dos capas

| Capa | Qué es | Tecnología | Requisito clave |
|---|---|---|---|
| **Objetos** (contenido) | Lo que el usuario toma con la mano | Malla + textura fotográfica (`.rsm` v2), aparece con el efecto de partículas `ScanBuildEffect` | Interactuar: colisión, física, agarrar (XRI) |
| **Entornos** (ambientación) | El lugar donde vive la biblioteca | Gaussian Splatting (`.ply`/`.spz` entrenados aparte) | Verse bien y **a la tasa del visor** (72–90 Hz), sin errores visuales |

### 1.3 Flujo completo

```
 Mundo real
   │  objeto en la hoja de marcadores           lugar recorrido con cámara/móvil
   ▼                                                  ▼
 RealSenseScans /objetos (5002)              entrenamiento 3DGS (Postshot, gsplat, Luma…)
   │  TSDF + textura (plan 14)                        │  .ply con SH
   ▼                                                  ▼
 Saves/objetos/<id>/ → (Blender opcional)     tools/splats/ (este plan): limpiar, podar,
   │  publicar                                  │  reducir SH, trocear, LOD → paquete de entorno
   ▼                                                  ▼
 enlace Unity 5010 ──────────────► VR ScanWorld ◄──── entornos importados (gsplat)
                                     │
                                     ▼
                     Biblioteca infinita en VR: cada objeto nuevo se construye con
                     partículas, se coloca en su estante y queda para siempre
```

### 1.4 Objetivos y no-objetivos

- **Objetivos:** (1) entorno 3DGS "pesado" en VR a la tasa del visor y sin errores visuales; (2) objetos escaneados
  que llegan en vivo desde 5010 y se pueden agarrar; (3) biblioteca que crece sin límite práctico (organización,
  persistencia, carga por zonas); (4) cadena reproducible para convertir un escaneo 3DGS en un entorno optimizado.
- **No-objetivos (por ahora):** entrenar 3DGS dentro de este repo; iluminar los objetos con la luz del splat
  (relighting); objetos interactivos en 3DGS (los objetos siguen siendo malla: se agarran y colisionan).

---

## 2. Estado actual del proyecto Unity (revisado hoy)

| Aspecto | Valor | Consecuencia para 3DGS |
|---|---|---|
| Unity | 6000.3.8f1, URP 17.3, Render Graph (modo compatibilidad **apagado**) | Bien para gsplat (lo exige apagado). aras-p tiene informes de VR roto en 6.3 (§4). |
| Espacio de color | **Lineal** | gsplat (wuyize) pide Gamma o reentrenar; ver §5.2 |
| API gráfica Windows | **Solo Direct3D11** | **Ningún** renderizador 3DGS de los evaluados funciona en DX11 (ordenan con operaciones de *wave*). Hay que pasar a **D3D12 o Vulkan**. |
| API gráfica Android | Vulkan + GLES3 | Vulkan sí; GLES3 no sirve para 3DGS |
| XR | XRI 3.6.1, OpenXR 1.16.1, Oculus 4.5.5 — **instalado pero sin commitear** | Modo estéreo: escritorio = Single Pass Instanced, Android = Multiview |
| URP | PC: render scale 1, HDR, MSAA 1. Móvil: render scale 0.8 | |
| GPU de desarrollo | **RTX 3050 Laptop, 4 GB VRAM** | Presupuesto de memoria y relleno muy justo (§6) |
| Visores disponibles | **Meta Quest 2** (objetivo principal, autónomo) y **HTC Vive Pro** (PC VR, SteamVR) | Dos versiones de cada entorno (§11) |
| Herramientas en el PC | CUDA 13.3 instalado; **sin** PyTorch, Node.js ni WSL | Instalarlas en G0 para la cadena offline (§7, §11) |

> Nota: CLAUDE.md dice "Sin OpenXR/Meta todavía": ya no es cierto (paquetes añadidos en el árbol de trabajo).

---

## 3. Qué hace pesada una escena de Gaussian Splatting

### 3.1 Qué hace la GPU en cada cuadro

Un splat es una **gaussiana 3D semitransparente** (posición, escala, rotación, opacidad, color base + armónicos
esféricos, SH). Para dibujar N splats cada cuadro:

1. **Profundidad de cada splat** respecto a la cámara (compute, O(N)).
2. **Ordenar** los N splats de atrás hacia delante (radix sort en GPU u ordenación por conteo en CPU).
3. **Proyectar** cada splat: covarianza 3D → elipse 2D en pantalla, color dependiente de la vista (evaluar SH).
4. **Dibujar un quad por splat** con mezcla alfa (sin escritura de profundidad), y en algunos renderizadores
   **componer** el resultado sobre la imagen de la cámara.

No hay "malla" que el hardware pueda descartar con el *z-buffer*: **cada splat visible se mezcla en cada píxel que
cubre**. Por eso el coste no depende solo de cuántos splats hay, sino de **cuántas capas semitransparentes se apilan
en cada píxel** (overdraw).

### 3.2 Factores de coste

| # | Factor | Crece con | Síntoma | Remedio |
|---|---|---|---|---|
| F1 | **Overdraw / tasa de relleno** | píxeles × capas por píxel × tamaño de los splats en pantalla | GPU al 100 % aunque N sea moderado; peor al acercarse a una pared | Recortar el quad por alfa, descartar splats < 2 px y de contribución mínima, podar splats enormes/flotantes, **renderizar el 3DGS a menor resolución** |
| F2 | **Número de splats N** | N | ordenación, proyección y memoria lineales en N | Podar opacidad baja, decimar, **LOD por distancia con presupuesto** de splats |
| F3 | **Ordenación** | N × frecuencia × bits de la clave | picos de 2-5 ms; en VR, si se ordena por ojo, el doble | Ordenar solo cuando la cabeza se mueve, claves de 16-20 bits, ordenación **radial** (no cambia al girar la cabeza), asíncrona |
| F4 | **Armónicos esféricos** | grado SH: SH0 = 3 floats, SH3 = 48 floats por splat | memoria ×4, ancho de banda del vertex shader | SH1 o SH0 en entornos lejanos; SH por distancia |
| F5 | **Formato en memoria** | bytes/splat | VRAM agotada, cargas lentas | PLY 3DGS con SH3 = **248 B/splat**; "Spark" empaquetado = 16 B + SH (8/16/16 B por banda) |
| F6 | **Buffers de trabajo** | N | aras-p: ~48 B/splat extra; gsplat: ~16 B/splat | elegir renderizador; no instanciar el mismo asset dos veces |
| F7 | **Pase extra de composición** | píxeles | aras-p dibuja en un RT RGBA16F y luego lo compone: un pase de pantalla completa por ojo y ancho de banda doble | dibujar en la cola transparente (gsplat) o en un RT **reducido** |
| F8 | **Sin culling** | splats fuera de vista | se procesan splats detrás o fuera del frustum | frustum culling por splat y por trozo (*tile*) |
| F9 | **Cámaras extra** | nº de cámaras | cada cámara (Scene view, UI, reflexiones) vuelve a ordenar y dibujar | excluir cámaras; medir en build, no en el editor |

### 3.3 Por qué la VR lo multiplica

- **Píxeles:** el Quest 3 renderiza ~2064×2208 por ojo → **~9 Mpx por cuadro**. La cifra pública de aras-p
  (escena *bicycle*, 6,1 M splats, 6,8 ms en RTX 3080 Ti) es a 1200×797 = **0,96 Mpx**: la VR pide ~9× más píxeles.
- **Tiempo:** 90 Hz = 11,1 ms por cuadro; 72 Hz = 13,9 ms, y el compositor del visor se queda una parte.
- **Dos vistas:** proyección y dibujo por ojo; si además se ordena por ojo, el coste de ordenar se duplica.
- **Distancias cortas:** en VR uno se acerca a las paredes; los splats cercanos ocupan media pantalla → overdraw
  enorme (F1). En una pantalla plana se mira de lejos.
- **Estimación de orden de magnitud** (a medir en G1): la RTX 3050 Laptop tiene ~1/4–1/5 del ancho de banda de la
  3080 Ti. Una escena tipo *bicycle* sin optimizar a resolución de visor estaría en **>100 ms/cuadro**. Para llegar a
  ~11 ms hace falta combinar todas las palancas: menos splats visibles (LOD + presupuesto ~1 M), menos píxeles
  (pase 3DGS a 0,6–0,75 de resolución) y menos overdraw (poda y culling).

### 3.4 Por qué PlayCanvas/SuperSplat va tan rápido en la web

Revisado en el código de `playcanvas/engine` (motor de SuperSplat, `src/scene/gsplat-unified/`). No hay magia:
**dibuja muchos menos splats y trabaja menos por splat**.

| Técnica PlayCanvas | Dónde | ¿En Unity hoy? |
|---|---|---|
| **Presupuesto de splats** por defecto **1 M** (`SPLAT_BUDGET_DEFAULT`) con **LOD por octree y streaming** (`gsplat-octree`, `gsplat-budget-balancer`) | `constants.js`, `gsplat-params.js` | **No** (ninguno de los tres) |
| **Ordenación por conteo en un *worker*** (CPU, asíncrona), claves de **10-20 bits** según N, con más precisión cerca de la cámara (`compareBits = clamp(log2(N/4), 10, 20)`, 32 *bins* ponderados) | `gsplat-unified-sort-worker.js` | No: aras-p y gsplat usan radix sort GPU de 32 bits |
| **Solo reordena si la cámara se movió**; con **ordenación radial**, solo si cambió la *posición* (girar la cabeza no reordena) | `gsplat-manager.js#testCameraMovedForSort` | gsplat: umbral de movimiento, sin radial. aras-p: cada N cuadros |
| **Descartar splats < 2 px** (`minPixelSize = 2`), por **contribución mínima** (`minContribution = 3`), *alpha clip* | `gsplat-params.js` | gsplat: culling pequeño/frustum en el vertex shader. aras-p: solo detrás de cámara |
| **Quad recortado por alfa** (el quad acaba donde α < 1/255) | shader gsplat | gsplat **sí** (`ClipCorner`); aras-p no |
| **Buffer de trabajo compacto** de 20 B/splat y formatos comprimidos (**SOG**: texturas WebP) | `gsplat-params.js#_createFormat` | gsplat "Spark" 16 B (parecido) |
| **Foveación** (`foveationStrength`) | `gsplat-params.js` | No |
| Herramientas offline: `splat-transform` (filtrar NaN/flotantes/opacidad, quitar bandas SH, decimar, LOD) | `@playcanvas/splat-transform` | Usable desde nuestro pipeline (§7) |

**Conclusión:** Unity no es "lento" por ser Unity. Los paquetes de Unity dibujan **todos** los splats, **siempre**,
a resolución completa y reordenando a 32 bits. PlayCanvas limita el trabajo por cuadro. Las mismas técnicas se
pueden llevar a Unity, y ese es el núcleo de este plan.

### 3.5 Errores visuales: de dónde salen

| Error | Causa | Dónde aparece |
|---|---|---|
| Splats mezclados en orden incorrecto (todo "lavado", objetos que se transparentan) | **Sin ordenación** | dylanebert: el VFX Graph tiene `sort: 0` |
| Un ojo negro, splats a mitad de tamaño, estéreo plano | Single Pass Instanced mal soportado (posiciones de recorte precalculadas para una sola vista, `_ScreenParams` por ojo) | aras-p en SPI (PR #173 cerrado, #225 abierto) |
| Pantalla negra en VR | SPI + URP sin HDR | aras-p (guía de World Labs: usar Multiview + HDR) |
| Bordes oscuros / halos, colores apagados | Mezcla en espacio **lineal** de un splat entrenado en **gamma** | gsplat en proyecto lineal con `Gamma To Linear` |
| "Popping" al girar la cabeza | Orden recalculado con poca frecuencia u orden por eje de vista | cualquiera; se mitiga con orden radial |
| Temblor / escalones en escenas grandes | Posiciones en **float16** absolutas: a 8-16 m del origen el paso es ~8 mm, a 16-32 m ~16 mm | gsplat modo Spark |
| Nubes, agujas y "flotantes" | Defectos del entrenamiento (no del render) | se limpian offline (§7) |
| Parpadeo/aliasing en splats finos lejanos | Splats < 1 px sin filtrar | `minPixelSize`, modelos *antialiased* (Mip-Splatting) |

---

## 4. Compatibilidad de los tres proyectos (revisado en su código)

| | **aras-p/UnityGaussianSplatting** | **dylanebert/UnityGaussianSplatting** | **wuyize25/gsplat-unity** |
|---|---|---|---|
| Qué es | El renderizador "de referencia" en Unity | Reimplementación **educativa** de SplatVFX con VFX Graph | Paquete inspirado en PlayCanvas |
| Último cambio | 2025-10 (autor: "no planeo más desarrollo" desde 2023-12) | 2025-02 (3 commits) | **2026-08** (v1.4.0, activo) |
| Pipelines | BiRP, URP, HDRP | URP (VFX Graph) | BiRP, URP, HDRP |
| Formatos | PLY, SPZ → asset propio con compresión por *chunks* | solo `.splat` (32 B, **sin SH**) | PLY (SH 0-3), SPZ v1-4 (SH 0-4); "Spark" 16 B o sin comprimir; carga en tiempo de ejecución |
| Ordenación | Radix GPU 32 bits, **cada N cuadros**, por cámara | **Ninguna** (`sort: 0`) → errores de mezcla | Radix GPU 32 bits, cada N cuadros **+ al moverse la cámara**; orden global entre renderers |
| Culling | Solo splats detrás de la cámara | Ninguno (`frustumCulling: 0`) | Frustum + splats pequeños en el vertex shader; quad recortado por alfa |
| Cómo dibuja | Compute "view data" (48 B/splat) → RT RGBA16F → **composición** | Partículas VFX (capacidad 5 M) | Directo en la cola transparente, 1 draw call instanciado |
| Espacio de color | **Correcto en lineal** (convierte al componer) | — | Pide **Gamma** (o `Gamma To Linear`, con halos) |
| VR | Multi-pass oficial. SPI/Multiview **solo con PR #225 sin fusionar** (Quest 3: 300 k splats a 16-18 fps). World Labs: **VR no funciona en Unity 6.3** | VFX Graph soporta XR, pero sin orden es inservible | **Multi-pass y SPI en URP** (macros estéreo + `UNITY_MATRIX_V` por ojo) |
| API | D3D12/Vulkan/Metal (no DX11, no GLES) | cualquiera | D3D12/Vulkan/Metal (no DX11, no GLES) |
| Extras | Editor de splats (seleccionar/borrar), recortes | — | Recortes, poda por opacidad al importar, subida asíncrona, generador de malla (marching cubes) |
| LOD / streaming | No | No | No |

**Veredicto:**
- **dylanebert:** descartado como renderizador (sin orden, sin SH, sin culling). Útil solo como ejemplo de enlazar
  buffers a un VFX Graph.
- **aras-p:** el más probado y con color lineal correcto, pero **sin mantenimiento**, VR estéreo de un ojo solo en
  un PR abierto y con informes de rotura en Unity 6.3 (nuestra versión). El pase de composición a resolución
  completa (F7) y la falta de culling (F8) penalizan justo en VR. Lo usamos como **referencia de comparación**, no
  como base.
- **gsplat (wuyize): base elegida.** Activo, diseño de PlayCanvas (sin RT intermedio), XR SPI en URP, formato
  compacto, culling, ordenación con umbral de movimiento y orden global (útil para trocear entornos). Sus dos
  problemas (Gamma y float16) tienen solución en nuestro fork (§5.2).

> Alternativas vistas pero no elegidas: **SplatBridge** (Asset Store, de pago, Unity 6, streaming con presupuesto,
> PC VR por OpenXR SPI/Multiview, Quest "experimental", sin cifras publicadas); forks VR de aras-p (ninjamode,
> Prempi1364 para Quest BiRP, ptc-lexvandersluijs BiRP). Si nuestro fork se atasca, SplatBridge es el plan B.

---

## 5. Decisiones de arquitectura

### 5.1 Base técnica
- **gsplat vendorizado** como paquete embebido en `Packages/com.wuyize.gsplat/` del proyecto Unity, fijado al commit
  `a2bf458` (2026-08-02). Nuestros cambios van en commits propios con prefijo `gsplat:` para poder rebasar contra
  versiones nuevas.
- **D3D12** en Windows (Vulkan como alternativa si D3D12 da problemas con OpenXR/Link). Vulkan en Android.
- **Entornos troceados** (*tiles*) con origen local por trozo: arregla la precisión float16, permite culling por
  trozo, LOD por distancia y carga por zonas de la biblioteca.

### 5.2 Espacio de color (el problema Gamma vs. lineal)
El proyecto es lineal (los objetos `.rsm`, la iluminación URP y XR lo esperan así). Opciones:

| Opción | Calidad | Coste | Decisión |
|---|---|---|---|
| Pasar el proyecto a Gamma | Splats correctos; objetos e iluminación cambian | ninguno | No: rompe lo demás, y XR/URP van mejor en lineal |
| `Gamma To Linear` de gsplat | Halos oscuros en bordes semitransparentes | ninguno | Solo para la primera medida (G1) |
| Reentrenar los splats en lineal | Correcto | reentrenar cada entorno | Si el entrenador lo permite, ideal |
| **Pase 3DGS propio en un RT a resolución reducida (mezcla en gamma) + composición a lineal** | Correcto | un pase barato de composición, pero **menos píxeles** que dibujar | **Elegida** para G3: resuelve el color y además ataca F1 (es la mejora de mayor impacto según el autor del PR #225) |

El entorno es fondo: se compone **después de los opacos** con prueba de profundidad contra la escena (los objetos
tapan al entorno) y **antes de los transparentes** (las partículas `ScanBuildEffect` quedan encima).

### 5.3 Ordenación en VR
Ordenar **una vez por cuadro para el ojo central** (no por ojo), con **orden radial** desde la cabeza: girar la
cabeza, lo más frecuente en VR, no exige reordenar; solo la traslación. Cadencia por movimiento (umbral de
posición), no cada cuadro.

---

## 6. Presupuestos de rendimiento (objetivos; se fijan con las medidas de G1)

| Plataforma | Objetivo | Splats visibles | Resolución del pase 3DGS | VRAM para splats |
|---|---|---|---|---|
| **PC VR**: HTC Vive Pro (1440×1600/ojo, 90 Hz) o Quest 2 por Link, con RTX 3050 Laptop 4 GB | 90 Hz (Vive Pro), 72 Hz mínimo | ≤ 1–1,5 M tras LOD | 0,6–0,75 | ≤ 1,2 GB |
| **Quest 2 autónomo** (1440×1584/ojo, Adreno 650) | 72 Hz | **≤ 100–150 k** splats cercanos + malla + cubemap (§11) | eye buffer 0,9–1,0 + foveación fija | < 50 MB |

Referencias públicas (todas en **Quest 3**, que tiene ~2,5× la GPU del Quest 2): 500 k splats ≈ 12 fps en Unity
(aras-p) frente a ≈ 19 fps en PlayCanvas en el navegador del visor; 2 M splats cierran la app (World Labs); el fork
del PR #225 logra 300 k a 16-18 fps; GSVision recomienda 700 k y se nota el retraso hacia 1,2 M. Las apps
comerciales de splats que analiza UploadVR (2026-09) solo se prueban en Quest 3, y Hyperscape exige Quest 3/3S.
**En Quest 2 autónomo una escena "pesada" no se puede dibujar tal cual: hay que transformarla** (§11).

Cuentas de memoria (sin SH / con SH3, solo datos del asset):

| Splats | PLY original (SH3) | gsplat Spark SH0 | gsplat Spark SH3 |
|---|---|---|---|
| 1 M | 248 MB | 16 MB | 56 MB |
| 3 M | 744 MB | 48 MB | 168 MB |
| 6 M | 1,49 GB | 96 MB | 336 MB |

(+ ~16 B/splat de buffers de ordenación en gsplat.) La memoria cabe; **el límite real es el tiempo de GPU (F1-F3)**.

---

## 7. Cadena de optimización de splats (offline)

Implementado (2026-10-23) como `tools/splats.py` (CLI: `info`, `camaras`, `limpiar`, `estacion`, `comparar`) sobre
la biblioteca `live3d/splats/` (Python + numpy, sin GPU, lee PLY de varios GB por bloques). Las imágenes y tiempos
salen del banco de Unity `ScanWorld.EditorTools.GsplatBench` (gsplat real). Resultados en §12. Pasos, en orden:

1. **Inspeccionar** (`--info`): N, bandas SH, caja envolvente, histogramas de opacidad y escala, densidad por zona,
   NaN/Inf. Informe en JSON + resumen en consola.
2. **Limpiar:** NaN/Inf; opacidad < umbral (0,01–0,05; en gsplat 0,05 quitó ~40 % con poco cambio visible);
   splats gigantes (escala > percentil 99,9 o > X m); flotantes (`--filter-floaters` de splat-transform); recorte
   a la zona útil (caja/esfera).
3. **Reducir SH:** SH3 → SH1 (o SH0 para trozos lejanos). Es la mayor reducción de memoria con menor pérdida en
   entornos difusos.
4. **Orientar y escalar:** ejes del entrenador → Unity (Y arriba, metros), suelo en y = 0. Se guarda la transformación.
5. **Trocear:** rejilla 2D (p. ej. 4×4 m) o octree; cada trozo con origen local y su caja.
6. **LOD:** por trozo, niveles al 100/50/25/12 % con `splat-transform --decimate` (o `--decimate-adaptive`).
7. **Exportar** un **paquete de entorno**: `entorno.json` (trozos, cajas, niveles, transformación, estadísticas)
   + los PLY/SPZ por trozo y nivel. Unity lo importa con un `ScanWorldEnvironmentImporter`.
8. **Informe antes/después:** N, MB, y capturas desde cámaras fijas para comparar visualmente.

---

## 8. Fases

| Fase | Qué | Hecho cuando |
|---|---|---|
| **G0** Preparación | Commitear los paquetes XR; dos *build profiles* (Quest 2 / PC VR, §11.6); D3D12 en Windows; rama `gsplat` en el repo Unity; en el PC: Node.js (splat-transform) y PyTorch + gsplat de nerfstudio con CUDA | El proyecto abre en D3D12 con XR, compila un APK de Quest 2 y las escenas actuales siguen funcionando (tests PlayMode verdes). **Hecho en Unity el 2026-10-23** (§12); faltan las herramientas del PC |
| **G1** Banco de pruebas | gsplat vendorizado; escena `GsplatBench` con trayectoria de cámara fija, `FrameTimingManager` (CPU/GPU), tiempos de ordenación y VRAM; medición **en build** y en el visor; aras-p en un proyecto aparte solo para comparar | Tabla de tiempos de la escena pesada **sin optimizar**: quién gana, dónde se va el tiempo (ordenar / proyectar / dibujar) |
| **G2** Optimización offline | `tools/splats/` (§7) aplicado a la escena del usuario | La escena pesa ≥ 50 % menos sin pérdida visible en las capturas fijas; tiempos de G1 repetidos |
| **G2Q** Versión Quest 2 | Estaciones, renders virtuales, poda por importancia, destilación, malla y cubemap (§11.3-11.5) | Una estación a ≥ 72 Hz en el Quest 2 con foveación, comparada en capturas con la versión PC |
| **G3** Optimización del render (fork) | Pase 3DGS en RT reducido + composición lineal (§5.2); orden radial y por movimiento; `minPixelSize`/`minContribution`; foveación fija; SH por distancia | Escena pesada a ≥ 72 Hz estable en PC VR, sin halos ni errores estéreo |
| **G4** LOD y streaming | Trozos + niveles + presupuesto global de splats (como `splatBudget` de PlayCanvas); subida asíncrona; orden global entre trozos | Recorrer el entorno entero sin tirones; splats visibles ≤ presupuesto en todo momento |
| **G5** Integración con la biblioteca | `EnvironmentManager` (cargar/cambiar entorno), suelo y colisiones (plano o malla de gsplat), estantes de la biblioteca colocados en el entorno, objetos de 5010 encima | Un objeto escaneado en vivo aparece en el estante del entorno 3DGS, se puede agarrar y la tasa se mantiene |
| **G6** Validación en el visor | Pruebas de confort (tasa, latencia, sin parpadeo), capturas `ScanWorldCapturas/` | Sesión de 10 min sin caídas bajo el objetivo |

---

## 9. Riesgos

| Riesgo | Mitigación |
|---|---|
| OpenXR + D3D12 en Unity 6.3 con Link da problemas | Probar Vulkan en G0; si ambos fallan, D3D11 queda descartado para 3DGS → reconsiderar SplatBridge |
| SPI de gsplat con `_ScreenParams` por ojo (splats a mitad de tamaño como en aras-p) | Verificar en G1 con un splat de prueba de tamaño conocido; Multiview/multipass como reserva |
| 4 GB de VRAM con Unity + XR + texturas de objetos | Spark + SH1 + LOD; medir VRAM en G1 |
| El presupuesto de la 3050 no alcanza 72 Hz ni con todo | Bajar la resolución del pase 3DGS, presupuesto 750 k, *reprojection* del runtime (ASW) como red de seguridad |
| Licencia de los splats (entrenador INRIA no comercial) | Entrenar con herramientas de licencia clara (gsplat/nerfstudio Apache-2.0, Postshot, etc.) |
| gsplat cambia aguas arriba | Fork fijado + commits propios pequeños |
| La ordenación GPU de gsplat (operaciones de *wave*) falla en el Adreno 650 | Orden en CPU con Burst y orden precalculado por estación (§11.4): en Quest no hace falta compute |
| gsplat (Python) no compila con CUDA 13.3 en Windows | Ruedas precompiladas de gsplat o CUDA 12.x en paralelo; WSL2 como reserva |
| Meta retira el soporte de Quest 2 en nuevas versiones del SDK | Fijar versiones de OpenXR/Meta que lo soporten; el Quest 2 por Link (PC VR) sigue funcionando |
| El splat de internet no permite uso derivado | Solo uso local de desarrollo hasta conocer la licencia; no publicar ni subir la versión destilada |

---

## 10. Qué necesito para la siguiente iteración (los splats)

- **Formato preferido: `.ply` 3DGS original sin comprimir**, con todas las bandas SH (en entrenamientos tipo INRIA:
  `point_cloud/iteration_30000/point_cloud.ply`). Es sin pérdidas y lo leen todos (gsplat, aras-p, splat-transform).
- Aceptables si no hay PLY: `.spz`, `.sog`, `.compressed.ply`, `.splat` (perdemos precisión o SH; avisar).
- El splat es **de internet, sin imágenes de entrenamiento**: no se puede reentrenar con fotos, pero sí **destilar**
  desde renders del propio splat (§11.3). No hace falta `cameras.json`: las cámaras virtuales salen de la zona por
  donde caminará el usuario.
- Datos útiles: **enlace de origen y licencia**, nº de splats, tamaño del lugar (m) y, si se sabe, con qué se
  entrenó (casi todos se entrenan en sRGB/gamma).
- Ubicación sugerida: `Saves/entornos/<nombre>/original/` (ignorado por git).

---

## 11. Estrategia para Quest 2 autónomo (objetivo principal) y PC VR con Vive Pro

### 11.1 El problema en números

| | Quest 2 | Quest 3 (referencias de §6) | Vive Pro (PC) |
|---|---|---|---|
| GPU | Adreno 650 (XR2 Gen 1), de tipo *tile-based* | XR2 Gen 2, ~2,5× Quest 2 | RTX 3050 Laptop |
| Píxeles por cuadro | 2 × 1440×1584 ≈ **4,6 Mpx** | ≈ 9,1 Mpx | 2 × 1440×1600 ≈ 4,6 Mpx (SteamVR al 100 %) |
| Frecuencia | 72 Hz → 13,9 ms | 72-90 Hz | 90 Hz → 11,1 ms |
| Splats que mueve un visor optimizado | **~150–300 k** (estimado: Quest 3 / 2,5) | 400–700 k | 1–1,5 M con LOD |

Una escena "pesada" de internet suele tener 1–6 M splats. En Quest 2 hay que quedarse con **~5–10 %** de los splats
**sin que se note desde donde está el usuario**. Para conseguirlo hay que explotar lo que sabemos de nuestra app.

### 11.2 Lo que juega a nuestro favor

1. **El entorno es estático.** Todo lo que se mueve (objetos, manos, partículas) es malla.
2. **El usuario no va a cualquier sitio.** La biblioteca se organiza en **estaciones**: zonas de ~2×2 m (escala
   de habitación) entre las que el usuario se teletransporta. Desde una estación, el 90 % del entorno está a más de
   3 m y solo se ve desde un volumen pequeño.
3. **Lo lejano no necesita paralaje.** A más de ~10 m, mover la cabeza ±0,5 m apenas cambia la imagen.
4. **Ya sabemos fundir RGB-D en mallas texturizadas** (TSDF + atlas de `live3d/objscan/`, plan 14): un splat se
   puede tratar como una **cámara RGB-D virtual perfecta**.

### 11.3 Opciones evaluadas

| Opción | Idea | Calidad | Quest 2 | Decisión |
|---|---|---|---|---|
| A. Splats tal cual + LOD | La versión de PC con presupuesto | — | 1–6 M splats: inviable | Solo PC VR |
| B. Poda/decimado "ciego" | Opacidad baja + `splat-transform --decimate` hasta 150 k | Agujeros y borrón en todo | ✓ | Línea base para comparar |
| C. **Poda por importancia desde la estación** | Renderizar el splat desde cientos de **cámaras virtuales** dentro del volumen de la estación y acumular cuánto aporta cada splat a los píxeles (opacidad × transmitancia, como LightGaussian). Se quedan los K que más aportan | Buena cerca, pierde lo que no se ve desde ahí (que no importa) | ✓ | **Primer paso**: no entrena nada, son minutos |
| D. **Destilación** (alumno compacto) | El splat original es el "maestro": sus renders son las imágenes de entrenamiento de un modelo nuevo con tope de K splats (gsplat, estrategia MCMC con `cap_max`), SH0-SH1, *antialiasing* | La mejor por splat (Distilled-3DGS: 0,49 M igualan a 3,4 M) | ✓ | **Segundo paso** si C no basta. Resuelve además el color: se entrena con renders en **lineal**, así se mezcla bien en Unity sin pase extra |
| E. **Malla texturizada horneada** | Renders RGB-D virtuales → TSDF (Open3D `UniformTSDFVolume`, ya en uso) → malla diezmada + atlas (`textura.py`) | Pierde reflejos, pelo, hojas y transparencias | Muy barata (y sirve de **colisión**) | Para el **campo medio** (3–10 m) y el suelo |
| F. Cubemap por estación | 6 caras de 1024-2048² renderizadas desde el centro de la estación, solo con lo que está a > 10 m | Sin paralaje (bien a > 10 m) | Coste casi nulo | Para el **fondo** |
| G. PC → Quest 2 por Link/Air Link | El PC renderiza la versión completa | Completa | — | Es el modo PC VR, no autónomo; útil igualmente |
| H. Render en la nube (estilo Hyperscape) | Streaming desde servidor | Completa | — | Fuera de alcance |

### 11.4 Arquitectura elegida para Quest 2: **estación = 3 capas**

```
            fondo  (> 10 m)   cubemap horneado, sin paralaje            ~0 ms
          campo medio (3–10 m)  malla texturizada horneada del splat    ~1 ms, también colisión
        cerca  (< 3 m)         splats reales: poda por importancia     100–150 k splats
                               (o alumno destilado), SH0, orden precalculado
   [usuario + estantes + objetos escaneados (.rsm)]
```

- **Ordenación sin compute:** cada estación trae un **orden precalculado** (radial desde su centro). Si la cabeza se
  aleja > 25 cm del último punto de orden, se reordena en **CPU con un Job de Burst** (ordenación por conteo de
  16 bits, < 1 ms para 150 k) y se sube el buffer de índices (600 KB). Así no dependemos de la ordenación GPU con
  operaciones de *wave*, que es el punto frágil en Adreno, ni bloqueamos la GPU *tile-based* con compute.
- **Dibujo directo en el eye buffer** (nada de RT intermedio en Quest: en GPU *tile-based* resolver y recomponer
  cuesta ancho de banda). Multiview + Vulkan + **foveación fija** (FFR) alta: el coste de los splats es casi todo
  de fragmentos, que es justo lo que la foveación recorta en la periferia.
- **Color:** con C se usa `Gamma To Linear` (halos leves). Con D, el alumno se entrena en lineal y se ve correcto.
- **Transiciones:** al teletransportarse se carga la siguiente estación en segundo plano (subida asíncrona de
  gsplat) y se funde en negro ~0,3 s. La "biblioteca infinita" es una red de estaciones que se cargan por demanda.
- **Presupuesto por cuadro (72 Hz):** splats ≤ 150 k (empezar en 100 k) · malla media ≤ 100 k triángulos ·
  1 cubemap · objetos `.rsm` visibles ≤ 50 k triángulos · eye buffer 0,9–1,0 · FFR alta. **AppSpaceWarp** (36 → 72 Hz)
  queda como reserva: pide vectores de movimiento y profundidad, que los splats no escriben.

### 11.5 Cadena offline para Quest (en `tools/splats/`, sobre el PC con la RTX 3050)

1. **Suelo y escala:** plano del suelo por RANSAC sobre los centros de los splats (misma idea que `segment.py`),
   Y arriba, metros. Para un splat de internet la escala es arbitraria: se ajusta con una medida conocida
   (altura de una puerta, una mesa).
2. **Estaciones:** se marcan a mano (posición + radio) en un visor sencillo o en `entorno.json`; más adelante, sugeridas
   automáticamente en zonas libres del suelo.
3. **Cámaras virtuales:** por estación, 300–600 poses dentro de un cilindro (radio 1 m, altura 1,1–1,8 m), mirando
   en todas direcciones; FOV y resolución del Quest 2.
4. **Renders del maestro** con gsplat (CUDA): color, profundidad esperada y alfa.
5. **Poda por importancia** → conjunto cercano de K splats (C). Opcional: **destilación** del conjunto (D), unos
   30-60 min por estación en la 3050 (a medir).
6. **Malla media:** TSDF con las profundidades del maestro (solo 3–10 m), diezmado y atlas.
7. **Cubemap:** render del maestro con plano cercano a 10 m.
8. **Paquete** `entorno_quest/estacion_XX/` (splats `.ply`/`.spz` SH0, orden precalculado, malla, cubemap) +
   capturas comparativas maestro ↔ versión Quest desde las mismas cámaras virtuales (PSNR/SSIM por estación).

### 11.6 Un proyecto Unity, dos versiones

| | Perfil **Quest 2** (Android) | Perfil **PC VR** (Windows) |
|---|---|---|
| Visores | Quest 2 autónomo | HTC Vive Pro (SteamVR) y Quest 2 por Link |
| API / estéreo | Vulkan · Multiview | D3D12 (o Vulkan) · Single Pass Instanced |
| Runtime | OpenXR + función Meta Quest | OpenXR de SteamVR (perfil de mandos HTC Vive) / Meta por Link |
| URP | `Mobile_RPAsset`, FFR, HDR apagado si se puede, MSAA 2× | `PC_RPAsset`, pase 3DGS en RT reducido (§5.2) |
| Entorno | `entorno_quest/` (estaciones de 3 capas) | `entorno_pc/` (trozos + LOD, §7) |
| Objetos `.rsm` | iguales (textura 1024²) | iguales (textura 2048²) |

Un `EnvironmentProfile` elige el paquete según la plataforma; la lógica de la biblioteca (estaciones, estantes,
objetos de 5010) es común. La versión PC también usa las estaciones, de modo que el diseño de la biblioteca es el
mismo en ambos visores. Con la Vive Pro hay que comprobar que la salida DisplayPort del portátil sale de la RTX 3050.

### 11.7 Orden de trabajo recomendado

1. **G0** con los dos perfiles y un APK vacío en el Quest 2 (asegura SDK, Vulkan y Multiview en el visor).
2. **G1 en Quest 2 primero:** gsplat con escenas de 50/100/200/400 k splats → curva real de ms por splat en el
   Adreno 650 (y si su ordenación GPU funciona). Esa curva fija K.
3. **Con tu splat:** limpieza (§7) → una estación → poda por importancia (C) → medir en el visor.
4. Malla media + cubemap → estación completa a 72 Hz.
5. Solo si C se ve pobre: destilación (D).
6. Versión PC/Vive Pro con LOD (G3-G4), que reutiliza la limpieza y las estaciones.

---

## 12. Registro: G0 y primera escena (2026-10-23)

### 12.1 G0 en Unity (rama `gsplat` del repo Unity)
OpenXR en Windows (D3D12 + Vulkan, SPI, mandos Vive / Oculus Touch / Index) y en Android (Vulkan, multiview, Meta
Quest Support, foveación SRP, IL2CPP ARM64, minSdk 32); build profiles `Quest2` / `PCVR`; gsplat vendorizado con su
renderer feature. El APK de Quest 2 compila (55 MB); tests PlayMode en verde en D3D12. Sin probar aún en el visor.

### 12.2 La escena: "Parish Church of Saint Maurice"
SuperSplat, autor Pavel Tanhäuser, **CC BY-NC-ND 4.0** (citar al autor, sin uso comercial, **no distribuir versiones
modificadas**: las variantes optimizadas son solo para uso local y no van en ningún APK público).

| Dato | Valor |
|---|---|
| Splats / formato | 9.975.598 · SH3 · 236 B/splat · 2,35 GB |
| Origen | Exportado de SOG: color, opacidad y escala en 8 bits; SH en paleta de ~64 k filas |
| Tamaño | 47 × 27 × 21 m (unidades = metros por las proporciones; confirmado al verla en Unity) |
| Ejes | RDF (arriba = −y); suelo nivelado (0,33°): no hace falta rotar |
| Relleno | el 1 % de splats más grandes aporta el 55 % del área×alfa; el 5 %, el 78 % |
| SH | RMS mediano 0,055 (≈14/255); el 86 % de la energía está en las bandas 2-3 |
| Flotantes | casi ninguno (0,3 % aislados) |

### 12.3 Hallazgos
1. **gsplat no lee archivos de más de 2 GB** (`NotSupportedException`): la escena original ni siquiera entra en
   Unity. `limpiar` parte la salida en trozos de < 1,8 GB y el banco los une con la **ordenación global** de gsplat.
2. **SH0/SH1 basta**: frente a SH3, PSNR 27,9 / 29,2 dB y SSIM 0,95 / 0,96; a la vista, casi indistinguible
   (cambian matices de brillo). Memoria: 2,2 GB → 859 MB (SH1) → 523 MB (SH0).
3. **El coste es overdraw sin oclusión**: desde la nave se dibujan ~2,9 M splats con **~600 capas de quad por
   píxel** (1,3 G fragmentos por ojo); la mitad viene de lo que está a más de 8 m (bóveda, paredes tapadas por
   columnas y bancos). Quitar SH apenas cambia el tiempo.
4. **Tiempos** (un ojo 1440×1584, RTX 3050 Laptop, D3D12, quitando ~10-13 ms fijos del render vacío): escena limpia
   completa **~120-260 ms**; 150 k splats **~10 ms**; 600 k **~21 ms**. Ni el PC llega a 90 Hz con la escena entera:
   **también la versión PC necesita LOD** (G4).
5. **La poda por importancia sola no basta** (estación "nave", 21 puntos de vista): con 150 k el campo cercano se
   conserva, pero lo lejano desaparece (PSNR 14,8, SSIM 0,66); con 600 k quedan agujeros en las paredes lisas
   (PSNR 20,4). La métrica alfa × ángulo sólido premia lo cercano y una pared lejana está hecha de muchos splats
   pequeños que, uno a uno, puntúan poco. Confirma la arquitectura de capas (§11.4): el campo lejano necesita otra
   representación.
6. **Banco**: dentro de `-executeMethod` síncrono, `Graphics.RenderMeshPrimitives` acumula dibujos (no hay fin de
   frame) y los tiempos crecen sin parar. El banco avanza un render por tick del editor y vuelve a medir la primera
   vista al final como control.

### 12.4 Siguiente paso propuesto (G2Q) — hecho, ver §12.5
**LOD por distancia desde la estación, fusionando splats** (en lugar de borrarlos): rejilla adaptativa alrededor de la
estación con celda ∝ distancia (≈0,4-0,6° de ángulo por celda); dentro de cada celda, una sola gaussiana por
ajuste de momentos (media y covarianza ponderadas por alfa×área, color/SH ponderados, alfa que conserva la "tinta").
Cubre toda la esfera con ~150-250 k splats y conserva el paralaje (es 3D, a diferencia del cubemap). Cerca
(< 3-4 m) se queda la poda por importancia actual a resolución completa. Medir con el mismo banco y, si el lejano
se ve demasiado blando, pasar a la malla horneada (§11.3-E) o a la destilación (§11.3-D).

### 12.5 LOD por fusión desde la estación (implementado, `tools/splats.py lod`, `live3d/splats/lod.py`)
Rejilla esférica alrededor de la estación (celdas de θ grados vistas desde el centro, profundidad ∝ distancia al
borde de la zona de movimiento); por celda, una gaussiana por ajuste de momentos (media/covarianza ponderadas por
tinta α·πσ1σ2, color y SH promediados, α = 1 − exp(−tinta/área)); las celdas de un solo splat se copian intactas.

**Fallo encontrado y corregido en la oclusión**: las paredes altas de la nave, vistas muy de canto, se ocultaban a
sí mismas (agujeros negros en la poda y en el primer LOD). Dos causas: (1) un splat plano visto de canto proyecta casi
una línea, pero se dibujaba en el mapa como un disco de 2σ → ahora el disco se escorza con la normal del splat
(eje de σ mínima, `load_core` la calcula) y fuera de su celda escribe d + 2σ1; (2) vista de canto, una celda del mapa
abarca metros de la misma pared → tolerancia proporcional al mayor salto de profundidad con las celdas vecinas.
Test sintético: pared de 40 m vista de canto, visible > 97 % (antes 16 %). Splats visibles: 6,4 M → 8,3 M.

Resultados (estación "nave", 9 cámaras: 6 direcciones, bóveda y dos bordes a 0,75 m; referencia SH3 completa
~111-165 ms; tiempos = coste de los splats en un ojo 1440×1584, RTX 3050, sin los ~9 ms fijos):

| Variante | Splats | ms | PSNR (peor vista) | SSIM |
|---|---|---|---|---|
| Poda por importancia 150 k | 150.000 | 5,9 | 14,8 (13,4) | 0,66 |
| LOD 2,5° | 62.720 | 3,7 | 20,3 (19,8) | 0,71 |
| **LOD 2°** | 99.681 | 6,7 | 20,8 (20,3) | 0,72 |
| LOD 1,2° radial ×3 | 138.118 | 8,1 | 21,1 (20,5) | 0,73 |
| **LOD 1,5°** | 175.636 | 11,0 | 21,5 (20,9) | 0,73 |
| LOD 1° | 362.610 | 18,6 | 22,5 (21,9) | 0,75 |
| **LOD 0,6°** | 805.853 | 27,9 | 23,9 (23,2) | 0,80 |
| LOD 0,4° | 1.398.356 | 34,6 | 24,9 (24,2) | 0,84 |

Lectura:
- **Sin agujeros y con paralaje**: a igual o menor número de splats que la poda, +6 dB y la escena completa.
- La calidad sigue una curva suave (~+1 dB por cada ×2 de splats); alargar las celdas en profundidad (radial ×3) cae
  en la misma curva, no la mejora.
- **Quest 2 (100-150 k)**: escena completa pero blanda (todo, también el suelo cercano, queda a ~2° por splat).
  **PC VR (0,6°, ~800 k)**: casi fiel (se ablandan el retablo y los relieves finos), ~28 ms por ojo en la 3050 a
  resolución de Quest; con el pase 3DGS a 0,7 de resolución y la Vive Pro (90 Hz) aún hay que bajar a ~1°.
- El límite de la fusión geométrica es que nadie corrige los errores: el siguiente salto de calidad es la
  **destilación** (§11.3-D) partiendo de este LOD como inicialización: optimizar posiciones, formas, colores y
  opacidades de los 100-150 k splats contra renders del original desde la estación (PyTorch + gsplat con CUDA).

### 12.6 Destilación desde el LOD (implementado, `tools/destilar.py`)
Entorno aparte `.venv-splats` (Python 3.10 portátil con `uv`, PyTorch 2.4.1 + CUDA 12.4 y la rueda precompilada
`gsplat 1.5.3+pt24cu124`: en Windows no hay ruedas de gsplat para Python 3.12 y sin MSVC no se puede compilar).
El maestro (escena limpia SH1, 9,8 M) renderiza 424 vistas de 512×512 desde dentro de la estación (FOV 90°,
alturas 1,2-1,9 m, 65 % horizonte, 25 % bóveda, 10 % suelo) y se libera la GPU; el alumno (el LOD) se optimiza
4000 iteraciones con L1 + 0,2·(1 − SSIM), sin cambiar el número de splats. ~2,5-3 min por estación en la 3050.

| Variante | Splats | ms (3050, 1 ojo) | PSNR Python (512², validación) | PSNR / SSIM en Unity (1440×1584) |
|---|---|---|---|---|
| LOD 2,5° → destilado | 62.720 | 3,7 → 10,1 | 20,4 → 28,8 | 20,3 / 0,71 → **23,7 / 0,79** |
| LOD 2° → destilado | 99.681 | 6,7 → 9,4 | 21,3 → 30,0 | 20,8 / 0,72 → **24,2 / 0,80** |
| LOD 1,5° → destilado | 175.636 | 11,0 → 13,8 | 22,4 → 31,5 | 21,5 / 0,73 → **24,8 / 0,82** |
| LOD 0,6° (sin destilar) | 805.853 | 27,9 | — | 23,9 / 0,80 |
| LOD 0,4° (sin destilar) | 1.398.356 | 34,6 | — | 24,9 / 0,84 |

Lectura:
- **100 k splats destilados igualan a 806 k sin destilar** (8× menos), y 176 k igualan a 1,4 M. A la vista: suelo,
  bancos, pilares y nervios de la bóveda nítidos; se ablanda el detalle fino lejano (retablo, cuadros).
- **Para Quest 2: LOD 2° destilado (100 k)** es el candidato; falta medirlo en el visor.
- La distancia entre el PSNR de Python (30 dB) y el de Unity (24 dB) viene de entrenar a 512² y medir a 1440×1584,
  y de las diferencias entre los dos renderizadores (gsplat de nerfstudio frente a gsplat de wuyize con
  `Gamma To Linear`).
- La destilación **agranda los splats** (más overdraw: 2° pasa de 6,7 a 9,4 ms; 2,5° llega a 24 ms en una vista).

Siguientes mejoras: (1) entrenar a más resolución (768-1024) o con recortes de 1440; (2) limitar el tamaño en pantalla
de los splats durante el entrenamiento (regularizador de escala) para que no crezca el overdraw; (3) entrenar en
espacio lineal para quitar `Gamma To Linear` en Unity (§5.2); (4) varias estaciones y transición entre ellas;
(5) **medir en el Quest 2** (APK con la estación destilada). (1) y (2) hechas en §12.7.

### 12.7 Destilación afinada: más resolución y límite de tamaño
`--res 1024` (el maestro de 9,8 M cabe: 2,1 GB de pico) y `--cap-escala k`: tras cada paso, ningún eje de un splat
puede pasar de k × su eje mayor inicial (restricción dura: el overdraw queda acotado por construcción). Indicador
rápido en Python: pares splat-tesela 16×16 que rasteriza gsplat (`tiles_per_gauss`). LOD 2° (99.681 splats):

| Variante | Entrenamiento | pares ×inicial | ms (Unity, 3050) | PSNR / SSIM (Unity) |
|---|---|---|---|---|
| LOD sin destilar | — | 1,00 | 6,7 | 20,8 / 0,720 |
| 512 px, sin límite (§12.6) | 2,5 min | — | 9,4 | 24,2 / 0,801 |
| 512 px, límite ×1,5 | 2,7 min | 1,07 | 6,1 | 23,7 / 0,795 |
| **1024 px, sin límite** | 7,8 min | 1,32 | 6,9 | **24,6 / 0,828** |
| **1024 px, límite ×1,5** | 6,8 min | 1,09 | **5,6** | 24,0 / 0,817 |
| 1024 px, límite ×1,2 | 6,6 min | 1,00 | 5,2 | 23,8 / 0,813 |
| LOD 0,6° sin destilar (806 k) | — | — | 27,9 | 23,9 / 0,795 |

Lectura:
- **Entrenar a 1024 px mejora la calidad a la resolución del visor** (SSIM 0,801 → 0,828) y, de paso, frena el
  crecimiento de los splats (a 512 px se agrandaban porque a esa resolución un splat grande y borroso cuesta poco).
- **El límite de tamaño deja el coste por debajo del LOD sin destilar** (5,2-5,6 ms frente a 6,7) perdiendo
  0,6-0,8 dB: los splats no crecen y la calidad se gana moviendo y recoloreando, no engordando.
- **Recomendado para Quest 2: 1024 px + límite ×1,5** (`lod_nave_2_d1024_c1p5.ply`, 100 k splats): +3,2 dB y
  −16 % de coste frente al LOD sin destilar; mejor SSIM que 806 k sin destilar con 1/5 del coste.
  Para PC VR (o si el Quest aguanta): 1024 px sin límite.
- Lo que sigue blando es el detalle lejano y fino (retablo, cuadros): con 100 k splats para toda la esfera no hay
  presupuesto para él; la mejora natural es repartir más splats hacia donde mira la gente (altar) o usar 150 k.

Reproducir (desde RealSenseScans):
```
python -m pip install --user uv
python -m uv venv .venv-splats --python 3.10
python -m uv pip install --python .venv-splats/Scripts/python.exe "torch==2.4.1" --index-url https://download.pytorch.org/whl/cu124
python -m uv pip install --python .venv-splats/Scripts/python.exe "numpy<2" pillow scipy packaging setuptools
python -m uv pip install --python .venv-splats/Scripts/python.exe "gsplat==1.5.3+pt24cu124" --find-links https://docs.gsplat.studio/whl/gsplat/
.venv-splats/Scripts/python.exe tools/destilar.py Saves/entornos/escena1/lod_nave_2.ply --maestro Saves/entornos/escena1/limpio_sh1.ply --res 1024 --cap-escala 1.5 --sufijo d1024_c1p5
python tools/splats.py info Saves/entornos/escena1/original/scene.ply
python tools/splats.py camaras Saves/entornos/escena1/original/scene.ply
python tools/splats.py limpiar Saves/entornos/escena1/original/scene.ply --variantes-sh 3,1,0
python tools/splats.py estacion Saves/entornos/escena1/limpio_sh1.ply --nombre nave --vista nave --k 100,150,300,600 --sh 1
python tools/splats.py lod Saves/entornos/escena1/limpio_sh1.ply --nombre nave --vista nave --angulos 2,1.5,0.6
Unity.exe -batchmode -force-d3d12 -projectPath "<VR ScanWorld>" -executeMethod ScanWorld.EditorTools.GsplatBench.Run
  -benchPly "ref=<...>/limpio_sh3_p0.ply|<...>/limpio_sh3_p1.ply;e150=<...>/estacion_nave_150k.ply"
  -benchCams <...>/camaras_estacion_nave.json -benchOut <...>/bench_estacion
python tools/splats.py comparar Saves/entornos/escena1/bench_estacion --referencia ref --variantes e150 --hojas
```

## 13. LOD dinámico de toda la escena (2026-10-23, noche)

Para poder recorrer el entorno entero (las estaciones solo valen cerca de su punto), como SuperSplat/PlayCanvas:
- **Pirámide por trozos** (`tools/splats.py piramide`, `live3d/splats/piramide.py`): cubos de 3 m; por cubo, nivel 0
  (originales) y fusiones por momentos en vóxeles de 4-128 cm. Un solo PLY ordenado por (trozo, nivel) + manifiesto con
  cajas (ejes de Unity) y rangos. Iglesia: 809 trozos, 15,8 M splats (1,38 GB); versión Quest sin nivel 0: 6,0 M (524 MB).
- **Fork de gsplat**: `GsplatRenderer.SetSelection` (dibuja solo los índices elegidos; la profundidad y la ordenación
  solo sobre ellos) y **pre-pasada de profundidad** opcional (`DepthPrepass`, shader `Gsplat/Depth`): los núcleos casi
  opacos escriben profundidad empujada hacia atrás y el pase de color descarta por early-Z lo oculto.
- **`ScanWorld.SplatLod`**: nivel por trozo según la distancia (tamaño angular constante), recorte por frustum con
  margen (los trozos del margen ×3 más gruesos), periferia más gruesa y presupuesto de splats por bisección del ángulo.

Medidas (16 cámaras repartidas por toda la iglesia, un ojo, RTX 3050; escena completa ~105 ms):

| Configuración | 800 k | 400 k | Calidad (PSNR / SSIM, 800 k) |
|---|---|---|---|
| Sin pre-pasada | 38,0 ms | 25,4 ms | 24,8 / 0,832 |
| Pre-pasada 0,5 / 2 % / 2 cm (por defecto) | 18,2 ms | 11,9 ms | 24,8 / 0,830 (sin pérdida) |
| + escala 0,7 (1008×1109) | 13,1 ms | 8,4 ms | — |

Perfil Quest (pirámide sin originales, 1152×1267, 150 k): 11 ms, PSNR 22,2 / SSIM 0,74 (borroso: falta destilar los
niveles de la pirámide). Escena de Unity: ScanWorld › Recrear escena de la iglesia (variantes 1 = LOD PC 600 k, 2 = perfil
Quest 150 k). Escenas nuevas: `python tools/splats.py preparar <escena.ply> --nombre X --unity <proyecto>`.

Pendiente: menú genérico de Unity para las escenas exportadas por `preparar`, destilar la pirámide, probar en el visor.

## 14. Escenas propias: del vídeo a Unity con un comando (2026-10-27)

Primera escena propia: el lavadero de azulejos de `vid_20260321_134921` (vídeo → RealityScan → COLMAP → LichtFeld
Studio, MCMC, 1 M de splats, SH3, 248 MB; flujo en `Documents/Photogrametry Workflow`).

**Problema**: COLMAP no fija gravedad ni escala. El detector de suelo (busca un plano alineado con un eje) fallaba:
se ve poco suelo, y el "arriba" medio de las cámaras sale inclinado ~20° porque el móvil apuntaba hacia abajo.
**Solución** (`live3d/splats/colmap.py`, `preparar --colmap <sparse/0>`): RANSAC del suelo entre los splats bajos,
guiado por el arriba de las cámaras (±35°); la escala sale de la altura de las cámaras sobre ese suelo
(`--altura-camara`, 1,5 m por defecto; `--escala` la fija a mano). Aquí: suelo = −y (0,1° de inclinación),
cámaras a 11,57 unidades → **0,13 m/unidad** (escena de ~3,8 × 3 × 5,6 m). La nivelación va como rotación del
objeto en Unity y la cámara inicial es la cámara central del recorrido.

**Limpieza nueva** (por defecto en `preparar`): agujas fantasma (σ1/σ2 > 10, > 3 cm, alfa < 0,35; los barrotes
opacos se quedan) y niebla con menos de 16 vecinos (alfa mediana 0,08-0,10). En el lavadero quitaron 27 k y 38 k
splats y casi todas las manchas flotantes; en la iglesia solo el 0,6 % y el 0,3 %. Quedan algunas agujas opacas
sueltas en zonas que el vídeo no vio.

**Pirámide a la medida**: trozos de diagonal/20 (1-3 m) y un nivel de 2 cm si la escena tiene < 3 M splats; sin eso
el presupuesto de una escena pequeña saltaba de todo-original a todo-4 cm (90 k splats en vez de 600 k).

Comando completo (8 s para 1 M de splats) y escena en Unity:
```
python tools/splats.py preparar "<...>/resultados/<escena>.ply" --nombre <escena> --colmap "<...>/proyectos/<escena>/colmap/sparse/0" --unity "<VR ScanWorld>"
```
Unity › ScanWorld › Crear escenas de entornos exportados → `Scenes/Entorno_<escena>.unity` (1 = LOD PC 600 k,
2 = perfil Quest 150 k; cámara de vuelo). `preparar` deja también `colision.json` para una escena VR con el rig.

---

## Fuentes

- Código revisado (clonado el 2026-10-22): [aras-p/UnityGaussianSplatting](https://github.com/aras-p/UnityGaussianSplatting)
  (2c6fed3), [dylanebert/UnityGaussianSplatting](https://github.com/dylanebert/UnityGaussianSplatting) (4e85853),
  [wuyize25/gsplat-unity](https://github.com/wuyize25/gsplat-unity) (a2bf458), [playcanvas/supersplat](https://github.com/playcanvas/supersplat)
  (ebce8e6, PlayCanvas 2.23.3) y [playcanvas/engine](https://github.com/playcanvas/engine) `src/scene/gsplat-unified/`.
- VR en aras-p: [PR #173](https://github.com/aras-p/UnityGaussianSplatting/pull/173) (cerrado) y
  [PR #225](https://github.com/aras-p/UnityGaussianSplatting/pull/225) (abierto, Quest 3 Multiview, 300 k a 16-18 fps).
- [World Labs — exportar a Unity](https://docs.worldlabs.ai/marble/export/gaussian-splat/unity) (Multiview + HDR,
  500 k vs 2 M splats en Quest 3, VR roto en Unity 6.3 con aras-p).
- [SplatBridge](https://radiancefields.com/splatbridge-brings-streamed-gaussian-splats-and-splat-colliders-to-unity-6).
- [splat-transform (PlayCanvas)](https://developer.playcanvas.com/user-manual/splat-transform/) y
  [LOD streaming](https://developer.playcanvas.com/user-manual/gaussian-splatting/building/lod-streaming/).
- [Unity Discussions — 3DGS en Quest 3](https://discussions.unity.com/t/3d-gaussian-splats-on-quest-3/1542816).
- [UploadVR — formas de ver splats en Quest (2026-09)](https://www.uploadvr.com/the-best-ways-to-view-gaussian-splats-on-quest/)
  (GSVision: 700 k recomendados, retraso a 1,2 M; todo probado en Quest 3).
- [aras-p issue #171](https://github.com/aras-p/UnityGaussianSplatting/issues/171) (Quest 2: lento de cerca; respuesta: menos
  splats, más lejos, menos resolución).
- [Distilled-3DGS](https://arxiv.org/html/2508.14037v1) (destilación maestro → alumno compacto).
