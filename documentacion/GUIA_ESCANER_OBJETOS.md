# Guía rápida · Escanear un objeto y verlo en Unity

Ruta completa: **escanear** (5002/objetos) → **ajustar** en Blender (opcional) → **publicar** → aparece en
**Unity** (VR ScanWorld) construyéndose con partículas.

## 0. Una sola vez

1. **Imprime la base**: `imprimibles/base_escaneo_A3.pdf`, o A4 para objetos de hasta ~10 cm, **al 100 %**
   (sin "ajustar a página"). Comprueba la barra de 5 cm con una regla y pégala plana sobre un cartón rígido.
   Si la escala no es exacta, el escáner la mide con la profundidad y la corrige; lo verás en el chip
   "Escala hoja".
2. En Unity: menú **ScanWorld › Crear escena de demostración** (ya está creada en
   `Assets/ScanWorld/Scenes/ScanWorld.unity`).

## 1. Arrancar

```bash
iniciar_sistema.bat                 # 5002 (cámaras) + 5005 (calibración) + 5010 (enlace Unity)
```
o por separado: `python app.py` · `python -m live3d.unity_link`. Sin cámaras se puede practicar con
`python app.py --synthetic-object` (plato giratorio simulado).

## 2. Escanear (http://localhost:5002/objetos)

1. **Objeto**: nombre y base (A3/A4). El encuadre muestra el objeto en **verde** y el resto atenuado, con el
   cilindro del volumen de trabajo en azul. Ajusta *radio* y *altura* para que el objeto quede dentro y lo
   demás fuera. Chips: base (marcadores vistos), tamaño estimado, distancia.
2. **Tomas**: pulsa **Espacio** o *Tomar foto*. Gira la hoja con el objeto encima unos **30°** entre tomas
   (o muévete alrededor). Haz una vuelta completa (**10-14 tomas**) y 2-4 más desde arriba (cámara más alta)
   para cerrar la parte superior. Cada miniatura muestra el encaje con lo ya escaneado y de dónde salió la
   pose (`base` = marcadores, lo ideal).
3. **Procesar**: tarda unos segundos. **Resultado**: modelo 3D giratorio, medidas y calidad.
4. **Publicar en Unity**, o **Abrir en Blender** para ajustarlo antes.

### Consejos del sensor (D455)

- Distancia ideal **0,6-0,9 m**. Por debajo de **0,52 m** no mide y por encima de ~1,2 m pierde detalle (el
  ruido crece con la distancia²).
- Superficies **negras, brillantes o transparentes** no dan profundidad: el aviso "silueta sin profundidad"
  lo detecta. Spray mate o talco lo arreglan.
- Objetos de menos de ~6 cm salen con poco detalle.
- Con las **dos cámaras** conectadas cada toma aporta dos vistas. La segunda usa la calibración como punto
  de partida y la refina sobre el objeto, así que una calibración imperfecta ya no deja fantasmas. Si la
  Cam 2 también ve la hoja, usa los marcadores directamente.
- **Sin base** también funciona (plano de la mesa + geometría), pero gira menos entre tomas (~20°) y elige
  objetos que no sean simétricos.

## 3. Blender (opcional)

*Abrir en Blender* lanza Blender con el objeto ya importado, de pie sobre el suelo. En la barra lateral
(N) › **RealSense › Objetos escaneados**:
- **Limpieza rápida**: une vértices, quita trozos sueltos, **cierra la base** (la cara apoyada en la mesa no
  se ve) y limita triángulos.
- Retoca lo que quieras (escultura, borrar restos…) y pulsa **Publicar en Unity**. Guarda
  `objeto_limpio.ply` en la biblioteca y sube la versión: Unity sustituye el modelo por el limpio.

## 4. Unity (VR ScanWorld)

Abre la escena **ScanWorld** y dale a **Play**. Con el enlace (5010) en marcha, cada objeto publicado
aparece sobre un pedestal construyéndose; si lo vuelves a publicar, se reconstruye. **R** repite el efecto.
Arriba a la izquierda se ve el estado del enlace.

Desde otro equipo de la red: `python -m live3d.unity_link --red`, y en *ScanLinkClient* pon la IP del PC y
el token que imprime.

## Si algo falla

| Síntoma | Causa probable |
|---|---|
| Chip "Base: no visible" | Marcadores tapados, muy de canto o fuera de cuadro. Basta con uno; inclina menos la cámara. |
| "Toma descartada: no encaja" | Giraste demasiado (sin base). Deshaz y gira menos. |
| Encaje bajo (< 50 %) en una miniatura | Poca zona común con lo ya escaneado. No pasa nada si la pose es por base. |
| El modelo trae restos de la mesa | El volumen empieza 4 mm sobre el papel: la hoja tiene que estar plana (pégala sobre cartón). Sin base, comprueba que la mesa se vea alrededor del objeto. |
| Unity: "sin conexión" | `python -m live3d.unity_link` no está corriendo o la URL o el token no coinciden. |
| Blender no abre | Define la variable de entorno `BLENDER_EXE` con la ruta de blender.exe. |
