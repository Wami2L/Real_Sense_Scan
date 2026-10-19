# 04. Integración Procedural en Blender

Blender actúa como el motor visualizador final. Los scripts de integración proveen una interfaz fluida para diseñadores 3D que necesitan el resultado del escáner dentro de sus proyectos inmediatamente.

## 1. Estructura del Add-on (`RealSense_Importer.py`)
Está diseñado usando el estándar de la API de Blender:
- `bl_info`: Metadatos del complemento.
- `bpy.types.Panel`: Dibuja los botones interactivos en el menú lateral de la vista 3D (View3D > Sidebar).
- `bpy.types.Operator`: Las clases que ejecutan acciones reales (Importar, Procesar malla).
- `bpy.types.PropertyGroup`: Almacena la ruta (Folder Path) seleccionada por el usuario.

## 2. El Desafío de las Nubes de Puntos en Blender
Al importar el archivo `.ply`, Blender lo lee como vértices sueltos. Motores de render como Cycles o Eevee no iluminan vértices, necesitan superficies.
Para solucionar esto de manera eficiente e interactiva, el script genera mediante Python un modificador **Geometry Nodes** (`convert_to_spheres_geometry_nodes`).
- Instancia un nodo `NodeGroupInput` (la geometría original).
- Instancia un nodo `GeometryNodeMeshIcoSphere` con radio de 3 milímetros.
- Conecta ambos mediante un `GeometryNodeInstanceOnPoints`.
El resultado visual es que el `.ply` se renderiza como una superficie densa de pequeñas esferas coloreadas que reaccionan a la luz.

## 3. Shading Procedural (`setup_point_cloud_material`)
La nube exportada por Open3D guarda la información de color de cada vértice en un atributo interno. El script arma automáticamente el Shader Tree (Nodos de Material):
1. Crea un `ShaderNodeAttribute` con el nombre de atributo `"Col"`.
2. Lo conecta directamente al input "Base Color" de un `ShaderNodeBsdfPrincipled`.
Esto elimina el tiempo manual que tardaría un artista 3D en reconectar vértices de colores a los shaders.

## 4. Delegación de Procesamiento Externo
Al presionar el botón de "Generar Malla", el hilo de Blender (Operator) lanza un subproceso a nivel del Sistema Operativo:
```python
subprocess.run(["python", script_path, folder_path], check=True)
```
Esto llama silenciosamente al script externo (`generar_malla_offline.py`), ejecutando todo el trabajo matemático denso con C++ y NumPy por detrás. Al finalizar, Blender importa silenciosamente el `malla_texturizada.obj` resultante y lo anida en una Colección dedicada de manera limpia.
