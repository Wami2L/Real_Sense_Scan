# Entorno Dividido: Alineación Virtual y Estudio de Cámaras (Puerto 5005)

> **ESTADO:** Operativo y Validado en Hardware Real  
> **Nombre Oficial:** Alineación Virtual Multi-Cámara  
> **Arquitectura:** Totalmente desacoplada (NO modifica ningún archivo de `live3d` ni `app.py`).

---

## 1. Novedades y Mejoras Implementadas

### A. Alineación Reflejada Directamente en el Entorno de las Cámaras (2D)
1. **🔮 Modo Fusión 2D Superpuesta (Perspective Overlay):**
   * Reproyecta la Cámara 2 sobre la perspectiva y sistema de coordenadas de la Cámara 1 utilizando la matriz de calibración física $T_{2 \to 1}$ y los mapas de profundidad densos.
   * Permite comprobar el encaje anatómico y espacial (rostro, cuerpo, paredes) de ambas cámaras fusionadas en una sola imagen.
   * **Barra de Opacidad Interactiva:** Control deslizante (0% a 100%) para desvanecer entre Cámara 1 pura y Cámara 2 reproyectada.
2. **🔴 Modo Anaglifo 3D Estéreo (Rojo / Cian):**
   * Canal rojo correspondiente a Cámara 1 y canales cian (verde + azul) a la Cámara 2 fusionada.
   * Las zonas perfectamente alineadas convergen en colores naturales; los desajustes se aprecian mediante franjas de color.
3. **📏 Modo Nivel Epipolar Horizontal:**
   * Muestra ambas cámaras lado a lado con líneas láser horizontales de nivel cada 45 px.
   * Permite verificar de un vistazo que la altura de ojos, barbilla, hombros y horizonte coincidan exactamente entre ambas cámaras.
4. **🎯 Rayos de Correspondencia 3D:**
   * Visualiza los emparejamientos geométricos SIFT / ICP calculados sobre el sujeto en primer plano.

---

### B. Control Reactivo en Caliente del Visor 3D (Three.js WebGL)
* **Proxy de Mismo Origen (`/viewer3d`):** El entorno 3D Three.js se sirve en el mismo origen web sin restricciones cross-origin.
* **Respuesta en Tiempo Real a 60 FPS:**
  * Al mover cualquiera de los sliders ($T_x, T_y, T_z$, $\text{Yaw}, \text{Pitch}, \text{Roll}$), la nube de puntos de la Cámara 2 se traslada y rota en vivo dentro de Three.js.
  * Al presionar **"Alinear con Esta Captura"**, el visor 3D aplica de inmediato la matriz calculada y sincroniza las dos nubes de puntos.

---

## 2. Cómo Usarlo

Abre tu navegador en:
👉 **[http://localhost:5005](http://localhost:5005)**

1. **Observa el stream "🔴 En Vivo"** en el visor para posicionarte en el campo de visión de ambas cámaras.
2. Haz clic en **"📸 Tomar Nueva Captura"**: se congelará tu toma y se añadirá a la galería.
3. Pulsa **"🪄 Alinear con Esta Captura"**:
   * El visor cambiará automáticamente a la pestaña **"🔮 Fusión 2D"**, donde verás la Cámara 2 reproyectada y fusionada sobre la Cámara 1.
   * En el panel izquierdo (visor 3D), las dos nubes de puntos se sincronizarán y fusionarán en tiempo real.
4. Usa las pestañas **"🔴 Anaglifo"** y **"📏 Nivel"** para inspeccionar la calidad del alineamiento estéreo y la nivelación de altura.
5. Usa la barra **"Opacidad Fusión"** para alternar entre ver la Cámara 1 o la Cámara 2.
