# Pendientes del escáner de objetos (2026-10-23)

Estado al cerrar: servidor 5002 reiniciado con las dos D455 en vivo (color 1280×800); el arreglo del
atlas UV está aplicado pero **sin commit** (ver tarea 0).

## 0. Cerrar lo de hoy
- [ ] Commit en `Pre_Produccion` del arreglo del atlas UV (`live3d/objscan/textura.py`: 4 particiones,
      muestras en uint8, mezcla vista a vista) y de este archivo. Tests de textura: 9/9.

## 1. Escaneo con las dos cámaras: textura a parches y malla rota (prioridad alta)
Sesión de prueba (sin volver a escanear): `Saves/2026-10-23/objeto_160547` — caja, 19 tomas × 2 cámaras,
36 vistas (32 con pose por la hoja). Objeto generado: `20261023_162521_tripode_hd` (nombre equivocado).
Síntomas: la etiqueta del código de barras aparece proyectada en otra cara; la malla sale con ~3 200 islas
de atlas (con una cámara, la caja daba ~400).
- [ ] Medir el desfase entre cámaras: `bias_check` (puntos de cada vista contra la malla, en el marco de su
      cámara) separando Cam 1 y Cam 2. Hipótesis: sesgo de profundidad o de escala entre las dos D455
      (Cam 2 con firmware 5.12) → dos superficies desplazadas mm → TSDF rugosa y rota.
- [ ] Comprobar si la pose por la hoja de cada cámara es coherente con su profundidad (plano de la hoja
      por profundidad vs. PnP de cada cámara; escala de la hoja medida por cada una).
- [ ] Probar: procesar solo Cam 1, solo Cam 2 y las dos (`from_folder` filtrando vistas) y comparar islas,
      rugosidad y textura (`render_vista` contra las fotos).
- [ ] Si hay sesgo: corregir por cámara (escala/offset de profundidad) o refinar cada vista con ICP contra
      la malla de la otra cámara antes del TSDF; si la textura sigue mezclando caras, endurecer la visibilidad
      (tolerancia de oclusión) para vistas de la otra cámara.
- [ ] Revisar el efecto de Taubin 2 pasadas (antes 4) en la rugosidad con dos cámaras.

## 2. El servidor se bloquea mientras procesa (prioridad alta)
Open3D (TSDF, UVAtlas, raycast) retiene el GIL: durante el procesado no responden la web ni las cámaras
(parece "colgado").
- [ ] Procesar en un **proceso aparte**: `finish()` lanza un subproceso que hace `from_folder(self.dir)` +
      `_process` y escribe el progreso en un JSON que el servidor lee (las tomas ya están en disco).
- [ ] Mostrar en `/objetos` el tiempo transcurrido de cada etapa, para distinguir "lento" de "colgado".

## 3. Nombre del objeto al reprocesar
- [ ] Guardar el nombre de la sesión en `vista.json` / `resumen_objeto.json` para que
      `tools/reprocesar_objeto.py` y `from_folder` lo recuperen (hoy usan el nombre de la carpeta).
- [ ] Renombrar `20261023_162521_tripode_hd` a «Caja RealSense (dual)» o borrarlo tras la tarea 1.

## 4. Calidad con RealSense (seguir midiendo)
- [ ] Repetir el trípode con color HD y comparar con el anterior (las tomas del primer trípode se perdieron
      por el bug ya arreglado de «Escanear otro objeto»).
- [ ] Probar «Calibrar con la hoja» con las dos cámaras reales (solo verificado con test sintético).
- [ ] Tiempo por toma con HD: ~3-3,8 s. Ver si la ráfaga o el filtro espacial a 1280×800 se pueden aligerar.
- [ ] Borde claro en la base del objeto (texels de papel en el último centímetro).

## 5. Unity
- [ ] Probar a mano la navegación nueva de `ScanWorld` (Tab, H, WASD, F); solo verificada con test PlayMode.
- [ ] Decidir si los cambios de la rama `gsplat` se llevan a `main`.

## 6. Datos y orden
- [ ] Objetos de prueba en la biblioteca: «Trípode (prueba ajustes)» no existe (falló), revisar duplicados
      («Realsense_Box», «Caja RealSense», «Figura texturizada (sim)», «Trípode (HD)»).
- [ ] Reimprimir la hoja A3 al 100 % (está al ~91 %) y pegarla plana sobre cartón (estaba ondulada).
