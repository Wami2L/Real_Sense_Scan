# Guía rápida · Escanear un objeto y verlo en Unity

Ruta completa: **escanear** (5002/objetos) → **ajustar** en Blender (opcional) → **publicar** → aparece en
**Unity** (VR ScanWorld) construyéndose con partículas.

El escáner toma la **forma** de la profundidad de la D455 (rápida y a escala) y el **color** de una textura
hecha con las fotos de cada toma (nítida). Detalles técnicos: [plan 14](planes/14_Plan_Escaneo_Texturizado_RealSense.md).

## 0. Una sola vez

1. **Imprime la base**: `imprimibles/base_escaneo_A3.pdf`, o A4 para objetos de hasta ~10 cm, **al 100 %**
   (sin "ajustar a página"). Comprueba la barra de 5 cm con una regla y pégala plana sobre un cartón rígido.
   Si la escala no es exacta, el escáner la mide con la profundidad y la corrige; lo verás en el chip
   "Escala hoja". **Solo una hoja a la vista**: si hay copias pegadas en la pared se usa la de la mesa, pero
   mejor quitarlas o taparlas.
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
2. **Tomas**, con *Captura automática* marcada:
   - Gira **la HOJA con el objeto encima**, nunca el objeto solo (la pose sale de la hoja: si el objeto gira
     sobre ella, las vistas se funden mal y la toma se rechaza con "misma vista").
   - Giros cortos de **~15-25°** y una pausa de 1-2 s: dispara sola cuando la vista es nueva (anillo **azul**
     en el gráfico de cobertura; **naranja** = ya tomada) y todo está quieto. Cada toma son 6 cuadros de
     profundidad combinados (~2,5 s): no muevas nada hasta que aparezca la miniatura.
   - Una vuelta completa (**16-24 tomas**). Para objetos con detalle arriba o en los laterales, otra vuelta
     con la cámara más alta o más baja.
   - Sin captura automática: **Espacio** o *Tomar foto*. Cada miniatura muestra el encaje con lo ya
     escaneado y de dónde salió la pose (`base` = marcadores, lo ideal).
3. **Procesar**: ~30-60 s (malla, atlas UV y textura). **Resultado**: modelo 3D con su textura, medidas y
   calidad.
4. **Publicar en Unity**, o **Abrir en Blender** para ajustarlo antes.

### Consejos del sensor (D455)

- Distancia ideal **0,4-0,6 m** al objeto (a 848×480 la D455 mide desde ~0,34 m). Más cerca, menos ruido
  (crece con la distancia²), marcadores más grandes y fotos con más detalle; por encima de 0,8 m avisa.
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
  `objeto_limpio.ply` en la biblioteca y sube la versión: Unity sustituye el modelo por el limpio, que se
  **re-texturiza solo** con las fotos de la sesión.
- Con textura, también tienes `objeto.obj` + `textura.jpg` en la carpeta del objeto para cualquier programa.

### Con las dos cámaras

El encuadre muestra **las dos imágenes**, cada una con de dónde sale su pose: **hoja** (ve los marcadores),
**calibración** (no ve la hoja: se coloca con la calibración dual) o **mesa** (sin base). Si las dos ven la
hoja, arriba aparece cuánto se aparta la calibración guardada de lo que miden los marcadores; con el botón
**Calibrar con la hoja** se guarda la nueva calibración en un clic. Si una cámara no aparece, el aviso dice
cuál (cable USB 3).

## 4. Unity (VR ScanWorld)

Abre la escena **ScanWorld** y dale a **Play**. Para moverte: **arrastrar** gira, **rueda** acerca, botón
**central** (o Shift+arrastrar) desplaza, **WASD/QE** mueve el foco por la sala, **Tab / ← → / 1-9** salta
entre objetos, **H** vista general y **F** modo vuelo (botón derecho para mirar). Abajo a la izquierda está la
ayuda. Con el enlace (5010) en marcha, cada objeto publicado
aparece sobre un pedestal construyéndose; si lo vuelves a publicar, se reconstruye. **R** repite el efecto.
Arriba a la izquierda se ve el estado del enlace.

Desde otro equipo de la red: `python -m live3d.unity_link --red`, y en *ScanLinkClient* pon la IP del PC y
el token que imprime.

## Volver a procesar sin cámara

Las tomas quedan en `Saves/<fecha>/objeto_HHMMSS/`. Para rehacer un objeto con el código actual:

```bash
python tools/reprocesar_objeto.py Saves/2026-10-21/objeto_162841 --reemplazar <id-de-la-biblioteca>
```

## Si algo falla

| Síntoma | Causa probable |
|---|---|
| Chip "Base: no visible" | Marcadores tapados, muy de canto o fuera de cuadro. Basta con uno; inclina menos la cámara. |
| "Toma descartada: no encaja" | Giraste demasiado (sin base). Deshaz y gira menos. |
| "Toma descartada: misma vista…" | No giraste la hoja (o giraste el objeto sobre ella). Gira la hoja con el objeto encima. |
| La captura automática no dispara | Anillo naranja: esa vista ya está; sigue girando. "Deja quieto": espera a que pare de moverse. |
| Letras o detalles dobles en la textura | Superficie negra o brillante (la profundidad se desvía). Spray mate o acércate más. |
| Encaje bajo (< 50 %) en una miniatura | Poca zona común con lo ya escaneado. No pasa nada si la pose es por base. |
| El modelo trae restos de la mesa | El volumen empieza 4 mm sobre el papel: la hoja tiene que estar plana (pégala sobre cartón). Sin base, comprueba que la mesa se vea alrededor del objeto. |
| Unity: "sin conexión" | `python -m live3d.unity_link` no está corriendo o la URL o el token no coinciden. |
| Blender no abre | Define la variable de entorno `BLENDER_EXE` con la ruta de blender.exe. |
