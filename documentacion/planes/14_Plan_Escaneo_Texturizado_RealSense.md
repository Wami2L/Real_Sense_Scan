# Plan 14: Escaneo de objetos con geometría RealSense y textura fotográfica

> **Rama:** `Pre_Produccion` · **Proyecto Unity:** `VR ScanWorld` (repo `Unity_Realsense_Reciever`, rama `main`)
> **Fecha:** 2026-10-21 · **Estado:** implementado y probado con la D455 real (caja RealSense, 22 tomas)
> **Sigue a:** [plan 13](13_Plan_Escaner_de_Objetos_a_Unity.md) (escáner de objetos → Blender → Unity)

---

## 1. Problema

El primer escaneo real del plan 13 salió **sin forma**: un cilindro borroso con color por vértice. Prioridad del
usuario: **velocidad de escaneo** (por eso las RealSense) con un resultado **bonito**.

## 2. Diagnóstico (con los datos guardados del escaneo real)

| # | Causa | Evidencia | Peso |
|---|---|---|---|
| D1 | El objeto se giró **sobre la hoja quieta**. Con la base, la pose es la de la hoja: 5 vistas distintas del objeto se fundieron como si fueran la misma. | Las 5 tomas: misma pose respecto a la hoja (az −3,5°, el 28,5°, ±4 mm). El consejo de la interfaz decía "Gira el objeto". | **Principal** |
| D2 | **Pose inestable a 0,9 m**: solo 2-3 marcadores de ~25 px. El PnP fija bien la posición en imagen pero mal la distancia y la inclinación. | Dos tomas sin mover nada: poses a 5 cm y 2° una de otra. | Alto |
| D3 | **Geometría gruesa**: 1 cuadro de profundidad por toma, vóxel automático de 6 mm, cámara a 0,95 m. | `voxel_mm: 5.0`, σz(0,95 m) ≈ 2 mm. | Alto |
| D4 | La profundidad **por debajo de 0,52 m se descartaba**. Ese Min-Z es el de 1280×720; a 848×480 la D455 mide desde ~0,34 m (Min-Z = fx·B/126 px = 447·0,095/126). | `segment.pixel_points` con `z_min_m = 0.52`. | Medio |
| D5 | **Color por vértice** a ~6 mm: borroso por definición. | — | Medio |

## 3. Alternativas investigadas

| Opción | Qué aporta | Coste | Decisión |
|---|---|---|---|
| **RealSense + textura UV propia** | Geometría métrica y rápida (TSDF) + nitidez de las fotos de cada toma con su pose exacta. Open3D 0.20 trae `compute_uvatlas`. | Segundos de proceso, sin dependencias nuevas. | **Elegida** (modo por defecto) |
| RealityScan 2.2 (instalado) | MVS + texturizado de calidad. Acepta imágenes con pose fija por XMP (`-addImageWithCalibration`, `PosePrior="locked"`) y CLI (`-headless -align -calculateNormalModel -unwrap -calculateTexture -exportModel`). | Minutos por objeto. Con las fotos de la D455 (848×480) la ganancia sobre la textura propia es pequeña. | Futuro modo "alta calidad", mejor con fotos de una cámara mejor (móvil/réflex) y las poses de la hoja. |
| COLMAP | Triangulación con poses conocidas + `patch_match_stereo`. | No instalado; MVS necesita CUDA; más lento que RealityScan. | Descartada |
| `color_map.run_rigid_optimizer` (Open3D) | Ajusta la pose de cada foto por consistencia de color. | 0,6 s, pero **divergió** (hasta 6 cm/6°) con una caja casi toda negra. | Descartada: las poses de la hoja son mejores |
| Fusión de vistas por texel con pesos (variantes 1-2) | Simple. | Letras **duplicadas**: la geometría de una superficie negra brillante se desvía 2-3 mm y las vistas no coinciden. | Sustituida por una vista por isla |
| Una vista por triángulo + ICM | Nítido en teoría. | Moteado: 80 k triángulos pequeños y 22 vistas parecidas. | Sustituida |

## 4. Qué se implementó

### Captura (`live3d/objscan/`)
- **Pose por plano de profundidad** (`base.refine_with_plane`): tras el PnP se ajusta el plano de la hoja con
  la profundidad (miles de puntos, ajuste robusto por MAD), se cortan con él los rayos de las esquinas de los
  marcadores y una **semejanza de Umeyama** entre la hoja nominal y esas esquinas 3D da la pose **y la escala
  real de la impresión** a la vez. Si no hay plano fiable, se queda el PnP.
- **Varias copias de la hoja** a la vista (p. ej. pegadas en la pared): las detecciones se agrupan por hoja y se
  usa la más horizontal (`base.group_sheets`).
- **Ráfaga de 6 cuadros** por toma (`rafaga.py`): mediana de profundidad; se descartan los cuadros en los que
  algo se movió (> 2 % de los píxeles cercanos con salto > max(1,5 cm, 2 %·z)). Medido en la D455: 0,1 % de
  cambio con la escena quieta, 6/6 cuadros usados.
- **Modelo del sensor para objetos** (`segment.D455_OBJ`, Min-Z 0,30 m) y avisos de distancia para 0,4-0,6 m.
- **Toma repetida** (misma pose respecto a la hoja, < 15 mm y < 4°): se rechaza con la explicación (si el objeto
  cambió, "gira la HOJA con el objeto encima").
- **Vóxel automático** según el ruido real (σz de la D455 reducido por la ráfaga): 1,5-4 mm.
- **Captura automática** en `/objetos`: dispara cuando la vista está a ≥ 22° de las ya tomadas y la escena lleva
  quieta ~0,35 s. Gráfico de cobertura (vista desde arriba: puntos = tomas, anillo = vista actual).

### Proceso
- **Falda de papel** (`session._skirt_mask`): lo pegado al suelo (< 14 mm) fuera de la huella del objeto
  (sección a 1,5-5 cm, rellenada y agrandada 12 mm) se elimina.
- **Textura** (`textura.py`): atlas UV (UVAtlas, 1 partición: islas grandes) → posición y normal 3D de cada
  texel → en cada vista: visibilidad por raycast contra la propia malla, peso = cos²·borde de silueta·(1/z²),
  brillos especulares casi sin peso → **una vista por isla del atlas** (la que mejor la ve en conjunto), las
  demás solo rellenan lo que esa no ve → ganancia de exposición por vista → relleno "push-pull" de lo no visto
  (la base apoyada). Colores por vértice sacados de la textura (miniatura, PLY, partículas).
- **Reprocesar sin cámara**: `ObjectScanSession.from_folder` y `tools/reprocesar_objeto.py` (las tomas se
  guardan con color, profundidad, máscara, pose y ráfaga).

### Formatos y biblioteca
- `objeto.obj` + `objeto.mtl` + `textura.jpg` (Blender y otros programas).
- **RSM v2** (`rsm.py`): flags bit2 = UV por vértice (v hacia arriba, como Unity), bit3 = textura JPEG al final.
  Las costuras UV duplican vértices (`textura.split_by_uv`). v1 sigue siendo válido.
- Una malla de Blender (`objeto_limpio.ply`) se **re-texturiza** con las fotos de la sesión al regenerar el RSM.
- Visor web (`rsmviewer.js`) lee v2 y muestra la textura.

### Unity (`Assets/ScanWorld/`)
- `RsmLoader.LoadModel` → `RsmModel { mesh, texture }` (textura legible, sRGB, mipmaps).
- `ScanBuildEffect.Build(mesh, texture)`: el sólido usa `_BaseMap`/`_UseMap` en `ScanWorld/BuildSolid`;
  las partículas toman su color de la textura (`GetPixelBilinear`).
- `ScanLinkClient.ModelReady(info, mesh, texture)`, `ScanGallery`, `ScanDemo.texture`, importador `.rsm` v2
  (versión 2: añade la textura y un material con ella). La figura de demostración es ahora texturizada.

## 5. Resultados

| Medida | Antes | Ahora |
|---|---|---|
| Dispersión de la pose (2 tomas sin mover nada, 0,9 m) | 5 cm, 2° | < 1 mm |
| Escala de la hoja medida | 86-92 % (ruidosa) | 90,9 % ± 0,1 (impresa con "ajustar a página") |
| Caja real: tomas útiles / encaje medio | 5 (misma vista) / 82 % | 22 / 95 % |
| Coherencia de la geometría con cada toma | — | 1-3 mm (sesgo < 1 mm por cámara) |
| Tamaño de la caja | 14,8 × 18 × 13,5 cm (borrón) | 12,4 × 16,6 × 8,3 cm (caja girada en su huella, sin falda) |
| Apariencia | Cilindro gris borroso | Caja reconocible, "REALSENSE" y código de barras legibles |
| Tiempo de proceso (22 tomas) | 0,4 s (5 tomas) | ~50 s (TSDF 2,5 mm + atlas + textura 2048²) |
| Tiempo por toma | ~1 s, 1 cuadro | ~2,4 s, 6 cuadros (ráfaga + pose + segmentación) |

Escena sintética (tests): textura con 98 % de texels vistos, ganancias ≈ 1, colores reproducidos desde
cualquier toma con diferencia media < 18/255.

## 6. Límites conocidos y siguientes pasos

1. **Superficies negras o brillantes**: la D455 desvía la geometría 2-3 mm; la textura las compensa pero los
   cantos salen algo redondeados. Spray mate si importa la forma exacta.
2. **Borde claro en la base** del objeto (píxeles de papel en el último centímetro). Mejora posible: excluir
   de la textura los texels a < 1 cm del suelo en vistas rasantes.
3. **Modo alta calidad con RealityScan**: exportar las fotos con XMP de pose fija (convención: `xcr:Rotation`
   = R mundo→cámara, `xcr:Position` = centro, `FocalLength35mm = f·36/max(w,h)`), máscaras y la caja del
   objeto; `-headless -align -calculateNormalModel -selectLargestModelComponent -simplify -unwrap
   -calculateTexture -exportModel`. Tiene sentido con fotos de más resolución que la D455.
4. **Blender**: el addon importa `objeto.ply` (color por vértice). Importar `objeto.obj` con su material para
   editar viendo la textura; al publicar, la biblioteca ya re-texturiza la malla limpia.
5. **Segunda vuelta** a otra altura: la captura automática exige ≥ 22° respecto a lo tomado, así que a una
   elevación cercana no dispara. Bajar el umbral cuando la elevación difiere, o pedir la segunda vuelta en
   la interfaz cuando la primera esté completa.
6. Hoja impresa al 91 %: se corrige sola, pero reimprimir al 100 % mejora el tamaño de los marcadores.
