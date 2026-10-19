# Estudio de calibración dual (puerto 5005)

Cliente del servidor Live 3D (5002). Sirve para tomar pares de cuadros de las dos cámaras,
calcular la extrínseca **T(Cam 2 → Cam 1)** y verificarla visualmente. Es transitorio: el
plan 10 (Fase 3) lo integra en 5002 como `/studio`.

## Qué hace (y qué no)

- **Estado veraz** en la cabecera: 2 cámaras en vivo · solo 1 cámara · cámara congelada ·
  cámaras no conectadas · simulación (sin cámaras) · servidor 5002 caído. Si el estado no
  permite una toma fiable, el botón de captura se desactiva y el aviso explica por qué.
- **Tomas emparejadas**: solo se aceptan si llegan cuadros de ambas cámaras, recientes
  (< 1,5 s) y con Δt < 250 ms entre ellas.
- **Alineación**: el motor único `live3d/alignment.py`, el mismo que usa el botón
  "Auto-Alinear" de 5002. Solo se aplica y se guarda si el resultado es fiable. Si no lo es,
  muestra el motivo y las vistas de verificación con la calibración **vigente**, para comparar.
- **Sliders**: previsualizan la pose en el visor 3D y en la fusión 2D, sin guardar nada.
  **Guardar** la fija como calibración vigente y **Restaurar** vuelve a la guardada.
- **Vistas**: fusión 2D con opacidad, anaglifo (rojo Cam 1 / cian Cam 2), nivel horizontal y
  correspondencias SIFT (verde: inliers 3D; rojo: descartadas).

## Cómo leer el resultado

| Métrica | Significado |
|---|---|
| Consistencia | % de puntos comunes que coinciden con lo que mide la otra cámara (proyección + espacio libre + orientación de superficie). ≥ 85 % es buena, ≥ 70 % aceptable. |
| Error residual | RMSE de esos puntos. Entre paréntesis va el ruido esperado del D455 a esa distancia (crece con z²). |
| Inliers 3D | Correspondencias SIFT coherentes con una única pose rígida. |

## Alcance del sensor (D455)

- Profundidad útil para calibrar: **0,52 – 3,0 m**. Por debajo no mide; más allá el error
  pasa de varios cm.
- Con las cámaras muy convergentes (el montaje actual está a ~60–70° de yaw), SIFT casi no
  encuentra correspondencias entre vistas, porque su invariancia llega a unos 30–40°. En ese
  caso la alineación se apoya en la geometría y en la calibración vigente.
  **Para una primera calibración usa el tablero ChArUco** (botón "Calibrar ChArUco" en 5002,
  con el tablero visible por las dos cámaras). La alineación automática sirve después para
  verificar y refinar.
- Para que la escena ayude: objetos con volumen y textura **única** en la zona que ven las dos
  cámaras, a 0,8–2,5 m, y quietos durante la toma.

## Uso

```bash
python app.py                                  # 5002 (cámaras reales o simulación si no hay)
python test_fotogrametria/servidor_test.py     # 5005
python test_fotogrametria/test_cli.py --all-saves   # evalúa el motor sobre los escaneos guardados
```
