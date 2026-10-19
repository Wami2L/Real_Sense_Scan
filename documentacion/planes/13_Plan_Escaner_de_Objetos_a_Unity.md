# Plan 13: Escáner de objetos → Blender → Unity (VR ScanWorld)

> **Rama:** `Pre_Produccion` · **Proyecto Unity:** `C:\Users\Usuario\Documents\Proyectos Unity\VR ScanWorld` (Unity 6000.3.8f1, URP 17.3)
> **Fecha:** 2026-10-19 · **Estado:** implementado y verificado sin hardware; prueba con el escáner el 2026-10-20

---

## 1. Objetivo

Una ruta completa y repetible para **objetos** del mundo real:

```
 1. ENCUADRAR        2. CAPTURAR VISTAS      3. PROCESAR            4. AJUSTAR (opcional)   5. VER EN UNITY
 objeto aislado  ->  multi-foto alrededor -> malla limpia,       -> Blender: limpieza    -> aparece "construyéndose"
 (resto atenuado)    con calidad por vista   orientada, a escala    mínima y "Publicar"     con partículas de luz y ¡paf!
```

Prioridad: **Unity y la calidad del objeto capturado**. El resto del sistema (Live 3D, estudio de calibración) se mantiene, pero no crece.

## 2. Diagnóstico del multi-foto actual (`multiscan.py`)

| # | Problema | Efecto |
|---|---|---|
| M1 | No aísla el objeto: registra y fusiona **toda la escena** (mesa, paredes, fondo). | Si se gira el objeto, el fondo inmóvil domina y el ICP devuelve "no se movió". Si se mueve la cámara, el modelo sale con la habitación pegada. |
| M2 | Registro encadenado sin referencia externa: solo ICP o FPFH entre vistas consecutivas. | El error se acumula, y con poco solape o simetrías la vuelta no cierra. |
| M3 | En dual, la Cam 2 se fusiona con la calibración tal cual (`voxel_down_sample`). | Si la calibración está mal (el problema recurrente), cada vista ya nace con fantasma. |
| M4 | La malla usa `ScalableTSDFVolume`, que está **vacío en Open3D 0.20** (C12). | Siempre cae en Poisson sobre toda la escena: burbujas y superficies inventadas. |
| M5 | Salida sin orientación ni pivote: ejes de la cámara 1, origen en el sensor y OBJ con colores no estándar. | En Blender o Unity el objeto aparece tumbado, lejos del origen y sin color. |
| M6 | La interfaz del multi-foto es un panel más dentro del visor Live 3D, sin encuadre ni vista previa del objeto. | Es difícil saber qué se va a capturar. |

## 3. Decisiones de diseño

1. **Base de escaneo con marcadores** (hoja A3/A4 impresa con 4–8 ArUco `DICT_4X4_50`, ids 10–17, distintos de los de calibración 0–3). El objeto va encima, en el centro.
   - Cada vista detecta los marcadores y obtiene la **pose exacta de la cámara respecto a la base**: un alineado concreto que no depende de la textura del objeto ni de la fotogrametría.
   - La base define **el suelo (plano) y el "arriba"**: el objeto sale de pie, con el pivote en el centro de su base.
   - Sirve igual girando el objeto (la base gira con él) que moviendo la cámara.
2. **Sin base también funciona**: plano dominante por RANSAC, objeto = cluster sobre el plano más cercano al centro de la imagen, y registro por ICP con color solo sobre el objeto. La calidad baja, pero no hay que imprimir nada.
3. **Aislar el objeto antes de registrar.** Un volumen de trabajo (cilindro sobre la base: radio y altura ajustables) recorta cada vista. La vista previa en vivo pinta el objeto en verde y atenúa el resto.
4. **Dual = dos vistas por toma**, unidas en el grafo de poses por la calibración como estimación inicial y **refinadas con ICP sobre el objeto**. Una calibración imperfecta deja de producir fantasmas.
5. **Malla**: `UniformTSDFVolume` dentro de la caja del objeto (sí funciona en 0.20), mayor componente conexa, recorte por debajo del plano, decimado a un presupuesto apto para Unity/VR (≤ 150 k triángulos) y colores por vértice.
6. **Formato de intercambio**:
   - `objeto.ply` (malla, metros, **Y arriba**, pivote en la base) para Blender.
   - `objeto.rsm` (binario propio, trivial de leer en C#) para Unity en tiempo de ejecución.
   - `objeto.json` (manifiesto: id, nombre, estado, tamaño, triángulos, calidad, origen).
   - Unity no necesita importadores de terceros.
7. **Biblioteca de objetos** en `Saves/objetos/<id>/`, con estados `crudo` → `limpio` (tras Blender) → `publicado`. La carpeta es la fuente de verdad: el servidor, Blender y Unity leen y escriben ahí.
8. **Enlace privado con Unity**: servidor ligero independiente de las cámaras (`python -m live3d.unity_link`, puerto 5010).
   - Por defecto solo `127.0.0.1`; en la red local hace falta un **token** que se comparte con Unity.
   - Unity consulta `GET /api/objetos?desde=<marca>` y descarga `GET /api/objetos/<id>/modelo.rsm`.
9. **Blender**: el addon existente gana tres operadores, *Abrir objeto escaneado*, *Limpieza rápida* (unir por distancia, quitar sueltos, decimar) y **Publicar en Unity**. Este último exporta a la biblioteca y cambia el estado.
10. **Unity** (`Assets/ScanWorld/`):
    - Cliente del enlace, cargador `.rsm` e importador de `.rsm` como asset (arrastrar y soltar).
    - Efecto de aparición en dos fases:
      - **Partículas de luz**: billboards en GPU, uno por punto muestreado de la superficie, que vuelan desde una nube dispersa hasta su sitio con brillo.
      - **Disolución**: la malla sólida "se revela" de abajo arriba con un borde emisivo y un destello final ("¡paf!").
    - Shaders URP escritos a mano: sin VFX Graph ni geometry shaders, así que funcionan en Quest y en PC.
    - Escena de demostración con un objeto de ejemplo para verlo sin escáner.
11. **Interfaz**: página propia **`/objetos`** en 5002, por pasos (Encuadre → Vistas → Procesar → Resultado → Publicar). El panel "Mapeo Multi-Foto" del visor se sustituye por un enlace a esa página, sin duplicar controles.

## 4. Fases

| Fase | Contenido | Verificación (sin escáner) |
|---|---|---|
| **O0** | Banco de pruebas sintético de objetos: escena (base con marcadores y objeto procedural) renderizada con *raycasting* de Open3D desde poses conocidas → vistas RGB-D como las de la D455, con ruido de σz. | Las vistas tienen la forma, las intrínsecas y el ruido esperados. |
| **O1** | `live3d/objscan/`: detección de la base (marcadores → pose cámara←base, plano y ejes), segmentación del objeto (volumen de trabajo o, sin base, plano + cluster) y máscara para la vista previa. Hoja imprimible de la base. | Pose de la base < 5 mm / 1°; la máscara contiene el objeto y nada de la base. |
| **O2** | Registro multi-vista: poses iniciales por base (o ICP encadenado sobre el objeto), dual como dos nodos, ICP con color sobre el objeto, grafo de poses y calidad por vista. | Con 8–12 vistas sintéticas: error de pose < 1 cm / 1°; sin base y con giros de 20–30°, < 2 cm. |
| **O3** | Malla: TSDF uniforme en la caja del objeto, limpieza, decimado y orientación con Y arriba y pivote en la base. Exporta PLY, RSM, manifiesto y miniatura. | Dimensiones de la malla = las del objeto ± 5 mm; ni rastro de la base. |
| **O4** | Biblioteca + servidor de enlace Unity (5010, token) + conversión perezosa PLY → RSM al publicar desde Blender. | Tests de API: listar, descargar, rechazar sin token desde la red. |
| **O5** | Sesión de escaneo de objetos en el servidor 5002 (API) + página `/objetos` + fuente sintética "objeto" para probar el flujo completo en el navegador. | Flujo completo en el navegador con la fuente sintética. |
| **O6** | Blender: abrir, limpiar y publicar (addon). | `blender -b` abre un objeto de la biblioteca, lo limpia y lo publica. |
| **O7** | Unity: enlace, cargador, importador, shaders, efecto de construcción y escena demo. | Compilación en *batchmode* sin errores y render a imagen de la escena demo en mitad del efecto. |
| **O8** | Documentación de uso (guía para mañana), `CLAUDE.md` y tests. | — |

## 5. Qué NO se hace ahora

- Unificar 5002 y 5005 (plan 11) ni reescribir el estudio de calibración.
- VR (OpenXR/Meta): la escena funciona en escritorio y los shaders son aptos para Quest. Añadir el paquete XR es un paso aparte cuando se elija el casco.
- Texturas UV (se usa color por vértice, suficiente para el efecto y para VR).

## 6. Riesgos

| Riesgo | Mitigación |
|---|---|
| Superficies negras, brillantes o transparentes sin profundidad en la D455. | Avisos en el encuadre cuando el objeto tiene poca profundidad válida. Recomendación de spray mate en la guía. |
| Objetos pequeños (< 8 cm) cerca del Min-Z (0,52 m) con resolución insuficiente. | Mostrar el tamaño estimado en el encuadre y la distancia recomendada. |
| Los marcadores no se ven en algunas vistas (tapados por el objeto). | 6–8 marcadores alrededor; basta con 1 visible. Si no hay ninguno, cae a ICP desde la vista anterior. |
| Shaders URP escritos sin abrir el editor. | Verificación en *batchmode* (compilación y render a PNG). |

---

## 7. Estado (2026-10-19)

Todas las fases (O0-O8) están implementadas. Se verificaron sin escáner con la escena sintética (verdad de
referencia), en el navegador, en Blender 4.5/5.2 en segundo plano y en Unity 6 en modo batch.

### Resultados medidos (escena sintética con ruido de la D455)

| Prueba | Resultado |
|---|---|
| Pose de la cámara por la base (PnP) | 1,4-3,3 mm · 0,2° desde varios ángulos; escala de la hoja medida al 0,1 % |
| Hoja impresa al 90 % | detectada (relación 0,90) y corregida |
| Superficie reconstruida, con base (10 tomas) | **0,9 mm** de error medio frente al objeto real; medidas ±2 mm |
| Superficie reconstruida, sin base (14 tomas) | **1,0 mm** |
| Dual con calibración desajustada 3-6 cm / 4-8° (Cam 2 sin ver la base) | corregida con todas las tomas: **0,9 mm**; el escaneo mide el desajuste |
| Lente con distorsión (brown e inverse brown) | rectificada; la pose vuelve a la precisión sin distorsión |
| Ida y vuelta Blender (abrir → limpiar → publicar) | ejes y medidas idénticos; RSM regenerado desde `objeto_limpio.ply` |
| Unity: efecto (8 fotogramas a PNG) y tests en modo juego | partículas → revelado → destello; el enlace descarga la versión de Blender y la galería la construye |

### Decisiones tomadas con datos

- **ICP point-to-plane, no con color.** Con luz fija y objeto girando, el sombreado cambia entre tomas. El ICP
  con color se desviaba 33 mm desde la pose correcta y el point-to-plane 0,5 mm. El color solo desempata
  hipótesis, como cromaticidad (sin intensidad).
- **El refinado nunca empeora una pose por base**: solo acepta retoques pequeños que mejoren la métrica fina.
- **Calibración dual**: se estima UNA corrección común para todas las tomas. Exige consenso entre tomas, y si
  no lo hay, las vistas de la Cam 2 que no encajan se excluyen en lugar de dejar fantasmas.

### Para la prueba con hardware (2026-10-20)

Seguir `documentacion/GUIA_ESCANER_OBJETOS.md`. Qué observar y anotar:
1. Chip "Base": cuántos marcadores se ven a 0,6-0,9 m y si la escala de la hoja sale ≈ 100 %.
2. Encaje por toma (miniaturas) y origen de la pose (`base` en casi todas).
3. Con las dos cámaras: la línea "Calibración dual" del resultado (desajuste medido). Si es grande y el
   objeto sale limpio, probar «Actualizar la calibración dual».
4. Objetos oscuros o brillantes: aviso de silueta sin profundidad.

### Pendiente / siguientes pasos

- Validar con la D455 real el rendimiento del encuadre (~3 fps con snapshot dual completo) y la
  rectificación con los coeficientes reales.
- VR: XR Plugin Management + OpenXR (o Meta XR) y un XR Origin en la escena ScanWorld.
- Texturas UV (ahora color por vértice) si se quiere más detalle de color con mallas ligeras.
