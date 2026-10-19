# 04. Integración en Blender

Blender es el visor y editor final de los escaneos. El addon `RealSense_Importer.py` (v2.0) importa cualquier guardado de `Saves/` con los ejes de Blender, color por punto, la malla y las cámaras reales colocadas en su sitio.

Probado en **Blender 4.5 y 5.2** (EEVEE y Cycles). Requiere Blender 4.0 o superior.

## 1. Instalación y uso
1. *Edit > Preferences > Add-ons > Install from Disk…* y elige `RealSense_Importer.py`. Activa **RealSense Scan Importer**.
2. En el visor 3D, barra lateral (tecla `N`), pestaña **RealSense**.
3. Elige la carpeta del escaneo o pulsa **Último escaneo**. El panel muestra el tipo y los datos clave (puntos, calidad de la calibración, Δt entre cámaras, si es simulada).
4. Marca qué importar y pulsa **Importar escaneo**.

Sin instalar nada: `blender --python script_blender.py` (o *Scripting > Run Script*) importa el último escaneo de `Saves/` con el mismo addon.

**Preferencias del addon** (opcionales):
- *Carpeta del proyecto*: raíz de RealSenseScans. Si se deja vacía, se deduce subiendo desde la carpeta del escaneo.
- *Python con Open3D*: el Python que usa "Generar malla". Si se deja vacío, se usa el `.venv` del proyecto y, si no existe, el `python` del sistema.

## 2. Qué importa de cada guardado

| Tipo | Por defecto | Opcional |
|---|---|---|
| `scan_` | `nube_puntos.ply`, cámara con `captura_color.jpg`, `malla_texturizada.obj` si existe | nube de calor; **Generar malla (offline)** con `generar_malla_offline.py` |
| `dual_scan_` | `nube_fusionada.ply`, `malla_fusionada.obj`, Cam 1 y Cam 2 con sus fotos | `nube_cam1`/`nube_cam2` (cada una en su marco), `diagnostico_solape.ply` |
| `multi_` | `nube_fusionada.ply`, `malla_fusionada.obj`, una cámara por vista con su pose | nube de cada vista en su marco, nube de calor |

Las nubes opcionales que se solapan con la principal se importan ocultas: se activan con el ojo del Outliner. También se leen los escaneos antiguos (por ejemplo, los duales sin metadatos de calibración).

## 3. Coordenadas
Los archivos están en coordenadas RealSense: x derecha, y abajo, z adelante, en metros. Cada escaneo cuelga de un objeto raíz `RS_<escaneo>` con una rotación de −90° en X, que lleva (x, y, z) a (x, z, −y) de Blender (Z arriba, la cámara mira a +Y). Si se desactiva *Ejes de Blender*, el raíz queda sin rotar.

- **Dual**: `Cam 2 (marco)` es un Empty con la extrínseca `T_cam2_to_cam1` de `metadata_dual.json`. `nube_cam2.ply` está en el marco de la Cam 2 y cuelga de ese Empty. Si se corrige la calibración, basta con mover el Empty.
- **Multi**: cada `vista_XX (marco)` lleva su pose de `resumen_multicaptura.json`.
- **Cámaras**: son objetos cámara de Blender con la distancia focal y el desplazamiento del punto principal sacados de las intrínsecas, y la foto de la toma como imagen de fondo. Al mirar a través de una cámara (`Num 0`), la nube debe coincidir con la foto.

## 4. Puntos y materiales
- Las nubes se convierten en puntos renderizables con un modificador **Geometry Nodes** (`Mesh to Points` + `Set Material`). Se ven en EEVEE y en Cycles y son mucho más ligeras que instanciar esferas.
- El **radio de punto** del panel cambia a la vez todas las nubes importadas.
- El material `RS_Color_<atributo>` lee el color por punto o por vértice: `Col` en los PLY de Open3D y `Color` en las mallas OBJ. La malla texturizada de los escaneos simples conserva su propio material (`.mtl`).

## 5. Generar malla (solo escaneos simples)
Los duales y las multicapturas ya traen `malla_fusionada.obj`. En un escaneo simple, **Generar malla (offline)** ejecuta `generar_malla_offline.py <carpeta>` con el Python configurado y luego importa `malla_texturizada.obj`, sustituyendo la que ya estuviera importada. Si falla, el mensaje muestra la última línea del error y qué Python se usó.

## 6. Pendiente
- Streaming en vivo dentro de Blender: el protocolo `RSF1` (`live3d/protocol.py`) se puede importar desde Blender, pero aún no existe el cliente.
