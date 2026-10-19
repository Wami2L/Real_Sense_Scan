# Plan 12: Alineación guiada — marcadores, puntos a mano y una interfaz que explica

> **Rama:** `Pre_Produccion`
> **Fecha:** 2026-10-19
> **Estado:** Primera versión implementada y probada (tests sintéticos + estudio con las D455). Queda refinar con uso real (§6).
> **Relación:** complementa el plan 10 §6 (motor de alineación) y prepara el panel *Calibración* del plan 11 (U3).

---

## 1. Cómo funciona la alineación

Alinear es calcular la pose de la Cam 2 respecto a la Cam 1 (`T_cam2_to_cam1`: desplazamiento y giro) para que las dos nubes encajen. El motor (`live3d/alignment.py: align_pair`) sigue estos pasos:

1. **Comprobaciones previas.** Que los dos cuadros sean del mismo instante (Δt) y que haya profundidad útil entre 0,52 y 3 m.
2. **Evidencia (correspondencias inequívocas)**, de más a menos fiable:
   - **Puntos a mano.** El usuario marca el mismo punto físico en las dos imágenes. Se eleva cada punto a 3D con la profundidad y se resuelve con Kabsch robusto: tríos de parejas, la pose que más parejas explica y aviso de las que se descartan.
   - **Marcadores ArUco** (`DICT_4X4_50`). Cada marcador da 4 esquinas con un número único. Las esquinas 3D salen de cortar sus rayos con el **plano ajustado a la profundidad del papel**, porque la profundidad justo en el borde es mala. La pose se obtiene con Kabsch por marcador y consenso entre marcadores.
   - **Fotogrametría (SIFT)**: el mismo detalle de color en las dos fotos, elevado a 3D, con RANSAC.
3. **Hipótesis.** Cada evidencia da una pose de partida. Se añaden la calibración guardada, la pose de los sliders, FPFH (forma 3D) y, si no hay evidencia, la búsqueda de montaje.
4. **ICP.** Cada hipótesis se refina con ICP point-to-plane robusto. Las de marcadores y puntos a mano se evalúan también sin ICP.
5. **Selección.** Gana la que tiene mejor **consistencia proyectiva** (los puntos que ven ambas cámaras coinciden al proyectarlos en la otra) combinada con el **acuerdo con la evidencia**.
6. **Veredicto.**
   - Calidad buena, aceptable o insuficiente.
   - Ambigüedad: dos poses distintas que encajan igual.
   - Contradicción con la calibración guardada: pide confirmar «Moví las cámaras», salvo que la respalden marcadores o puntos a mano coherentes.
   - Si la zona común es mínima pero hay marcadores o puntos coherentes, la pose se acepta como «aceptable» con aviso.

### Por qué SIFT casi no encuentra puntos en este montaje

Las cámaras están a unos 60–70° entre sí, y SIFT reconoce un mismo detalle solo hasta unos 30–40° de cambio de vista. A eso se suman:
- texturas repetidas (ladrillo, camisa a cuadros), que el filtro de ambigüedad descarta;
- profundidad incompleta, sobre todo en bordes;
- imágenes JPEG a 848×480.

Medido en tomas reales: 50–150 parejas, 4–12 con profundidad y 0–9 coherentes. Por eso la evidencia útil son los **marcadores** y los **puntos a mano**, que no dependen del ángulo.

---

## 2. Qué se implementó

### Motor (`live3d/alignment.py`)

| Función | Qué hace |
|---|---|
| `detect_markers`, `marker_correspondences`, `pose_from_markers` | Detección ArUco, esquinas 3D por plano del papel y pose robusta por marcador |
| `lift_manual_pairs`, `robust_kabsch` | Puntos a mano a 3D (con aviso de borde o falta de profundidad) y pose tolerante a parejas erróneas |
| `align_pair(..., manual_pairs=...)` | Evidencia unificada (`metrics.evidence`: `manual`, `marcadores`, `sift` o ninguna), candidatas con `agreements_pct`, `chosen` y `viable`, y lista `correspondences` (píxeles en ambas imágenes, residuo 3D en mm, ok) |
| `alignment_error_map` | Mapa de error píxel a píxel sobre la Cam 1 (verde < 5 mm … rojo > 20 mm) con estadísticas (mediana, p90, % < 1 cm, m² comunes) |
| `combine_poses` | Media robusta ponderada de las poses de varias tomas (descarta las que se apartan más de 3 cm / 3°) |
| `preflight` | Ahora también detecta marcadores en vivo y dice cuáles son comunes |
| `draw_matches` | Parejas numeradas por tipo: SIFT en verde o rojo, marcadores en amarillo, a mano en cian o magenta |

### Calibración (`live3d/capture.py`, `live3d/server.py`)

- `extrinsics_history`: las 20 calibraciones anteriores, guardadas en el JSON.
- `restore_extrinsics(i)` y los endpoints `GET /api/dual/extrinsics/history` y `POST /api/dual/extrinsics/restore`.
- Deshacer no pierde nada: la calibración actual pasa al historial, así que deshacer dos veces vuelve al punto de partida.

### Hoja de marcadores (`tools/generar_marcadores.py`)

- PDF A4 a 300 ppp con barra de 5 cm para comprobar la escala e instrucciones de colocación.
- `python tools/generar_marcadores.py [--ids 0-3] [--tamano-cm 16]`, o desde el estudio con `/api/markers.pdf?tam=16`.

### Estudio (5005)

- **Asistente de 6 pasos:** montaje, previsualizar, tomar, método, alinear y guardar, con la instrucción del paso actual.
- **Método:** *Automático* (marcadores, SIFT y geometría) o *Puntos a mano*. Botones para imprimir marcadores de 16 o 10 cm.
- **📍 Puntos:** las dos imágenes originales. Clic en la Cam 1 y luego en el mismo punto de la Cam 2, con lupa 4×. Las parejas van numeradas y se colorean según su residuo tras alinear. Se pueden deshacer y borrar, y hay pantalla completa.
- **📊 Error:** mapa de error de la pose nueva, de la candidata (si falta confirmar) o de la vigente. Con la pestaña abierta, se recalcula al mover los sliders.
- **🎯 Parejas:** la imagen de parejas por tipo.
- **Detalle de la alineación:**
  - resumen en lenguaje claro;
  - tabla de hipótesis (★ la elegida, en gris las descartadas);
  - tabla de parejas filtrable con su residuo;
  - texto que interpreta el mapa de error.
- **Glosario** «¿Qué significa cada cosa?».
- **Multi-toma:** casilla «multi» en cada toma de la galería y «Alinear con N tomas». Da la dispersión entre tomas y las tomas descartadas.
- **Historial de calibraciones** con «Deshacer último cambio» y «Restaurar» por entrada.
- **Previsualización:** marcadores vivos dibujados con su ID, y casillas *Marcadores* y *Textura común*.

### Tests

- `tests/test_evidencias.py`:
  - escena sintética de un plano con dos marcadores y la Cam 2 a 60 cm y 40°;
  - pose recuperada con marcadores a menos de 1,5 cm y 1°, aunque la calibración guardada esté del lado contrario;
  - puntos a mano con una pareja ocluida descartada;
  - aviso de parejas incoherentes;
  - mapa de error;
  - combinación multi-toma.
- `tests/test_estudio_api.py`: flujo completo del estudio con 5002 simulado (toma, alineación aplicada, multi-toma, mapa de error, PDF).
- `tests/test_estado_camaras.py`: historial y deshacer, en memoria y en disco.
- **Total:** 60 pasan, 1 se omite con cámaras conectadas y 1 `xfail` (C12).

---

## 3. Cómo calibrar (procedimiento recomendado)

1. **Fija las cámaras.** Se midió que el montaje se mueve entre tomas: 3,6° y 7° en 20 minutos, y otra vez a las 16:21. Marca «Montaje fijo».
2. **Imprime 2–4 marcadores** de 16 cm (o de 10 cm si estás a menos de 1 m) al 100 %. Pégalos planos sobre cartón.
3. **Colócalos** donde los vean **ambas** cámaras, separados entre sí, a 0,6–2 m y girados hacia el punto medio entre las cámaras. En la previsualización deben salir como «comunes» (recuadro amarillo con su ID en las dos vistas).
4. **Toma 2–3 capturas** sin moverte y alinea una («Automático»). Si sale bien, marca las tres con «multi» y pulsa «Alinear con 3 tomas».
5. **Verifica** en 📊 Error: mayoría verde o amarilla y mediana cercana al ruido esperado. Usa también el visor 3D en modo Solape.
6. Si no hay marcadores: método **Puntos a mano**, con 5–6 parejas en esquinas nítidas sobre superficies planas, repartidas y a distintas profundidades.
7. ¿Algo fue mal? Usa **Deshacer** en el historial.

---

## 4. Medidas de calidad (cómo leerlas)

| Medida | Bien | Revisar |
|---|---|---|
| Consistencia | ≥ 85 % | < 70 % |
| Mediana del mapa de error | ≤ 2× el ruido esperado (≈ 5 mm a 1 m, 18 mm a 2 m) | > 15 mm a 1–1,5 m |
| Residuo de las esquinas de marcador | < 10 mm | > 15 mm (¿marcador doblado o de canto?) |
| Dispersión multi-toma | ≤ 1 cm / 1° | > 2,5 cm / 2,5° (el montaje se movió entre tomas) |

---

## 5. Decisiones

- **La profundidad, no el tamaño del marcador.** Las esquinas 3D salen de la profundidad (plano del papel), así que no hace falta medir ni configurar el lado impreso. Una mejora futura es combinarlo con PnP si se conoce el lado (ver §6).
- **Diccionario `DICT_4X4_50`**, distinto del tablero ChArUco (`DICT_5X5_100`), para no confundirlos.
- **La geometría sigue siendo el juez.** La evidencia orienta y desempata; sin zona común suficiente, la pose se acepta solo como «aceptable» y con aviso.
- **El historial vive en el mismo JSON** (`extrinsics_history`) que la calibración: sin archivos nuevos y compatible con versiones anteriores.

---

## 6. Para refinar (siguiente sesión)

1. **Probar con marcadores impresos reales** a distintas distancias y ángulos. Ajustar el umbral de residuo (15 mm) y el margen de papel usado para el plano (×1,3).
2. **PnP con tamaño conocido** (opcional): combinar la pose del marcador por imagen con la de profundidad cuando el papel esté de canto o con poca profundidad (emisor apagado en la Cam 2).
3. **Selector de puntos:** zoom y desplazamiento dentro de la imagen, sugerencias automáticas de esquinas cercanas al clic (`cornerSubPix`) y resaltar la pareja al pasar el ratón por la tabla.
4. **Mapa de error en 3D:** colorear la nube del visor 3D por error, no solo la imagen.
5. **Multi-toma con ICP conjunto** (una sola optimización sobre todas las tomas) en lugar de promediar poses.
6. **Detector de montaje movido:** aviso proactivo si la previsualización muestra «la pose no encaja» durante más de 10 s (plan 11 U3).
7. **Emisor IR de la Cam 2:** encenderlo mejora la profundidad sobre el papel de los marcadores (plan 10 §6.4). Probarlo con esta herramienta.
