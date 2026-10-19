// Shaders GLSL3 (WebGL2). Toda la reconstrucción 3D ocurre en la GPU:
//   profundidad uint16 (textura R16UI) -> punto 3D (intrínsecas de profundidad)
//   punto 3D -> UV de color (extrínsecas + intrínsecas de color)
//   normales y detección de bordes con los 4 vecinos (texelFetch)
// La CPU solo sube una textura de 2 bytes/punto por cuadro.

export const vertexShader = /* glsl */ `
precision highp float;
precision highp int;

uniform highp usampler2D uDepth;
uniform vec2  uDSize;      // ancho, alto de profundidad
uniform vec4  uDIntr;      // fx, fy, ppx, ppy (profundidad)
uniform float uDScale;     // metros por unidad
uniform vec4  uCIntr;      // fx, fy, ppx, ppy (color)
uniform vec2  uCSize;
uniform mat3  uR;          // profundidad -> color
uniform vec3  uT;
uniform vec2  uRange;      // near, far
uniform float uPointScale;
uniform float uProjScale;  // px por unidad a distancia 1
uniform float uEdge;       // salto relativo para considerar borde
uniform float uFlyFilter;  // (puntos) ocultar píxeles voladores

// Sensor virtual único: la cámara secundaria solo dibuja lo que la de referencia no ve
uniform float uRefOn;              // 1 = descartar puntos ya cubiertos por la cámara de referencia
uniform highp usampler2D uRefDepth; // profundidad de la cámara de referencia (Cam 1)
uniform vec2  uRefDSize;
uniform vec4  uRefDIntr;
uniform float uRefDScale;
uniform mat4  uToRef;              // esta cámara -> referencia (coordenadas RealSense)
uniform float uRefTol;             // tolerancia relativa de profundidad

out vec2  vUv;
out float vZ;
out vec3  vN;
out float vValid;
out float vEdge;

float depthAt(ivec2 p) {
  ivec2 maxCoord = ivec2(uDSize) - ivec2(1);
  p = clamp(p, ivec2(0), maxCoord);
  return float(texelFetch(uDepth, p, 0).r) * uDScale;
}

bool okZ(float z) {
  return z > 0.0 && z >= uRange.x && z <= uRange.y;
}

// RealSense (x der, y abajo, z delante) -> Three.js (x der, y arriba, z hacia el visor)
vec3 rsPoint(vec2 pix, float z) {
  return vec3((pix.x - uDIntr.z) / uDIntr.x * z, (pix.y - uDIntr.w) / uDIntr.y * z, z);
}
vec3 toWorld(vec3 p) {
  return vec3(p.x, -p.y, -p.z);
}

void main() {
  ivec2 pix = ivec2(position.xy);
  vec2  fp  = position.xy;
  float z   = depthAt(pix);
  bool  valid = okZ(z);

  float zl = depthAt(pix + ivec2(-1, 0));
  float zr = depthAt(pix + ivec2( 1, 0));
  float zu = depthAt(pix + ivec2( 0,-1));
  float zd = depthAt(pix + ivec2( 0, 1));
  bool vl = okZ(zl), vr = okZ(zr), vu = okZ(zu), vd = okZ(zd);

  // Vértice inválido: se coloca a la profundidad de un vecino válido para que
  // sus triángulos (descartados en el fragment) no crucen la pantalla.
  if (!valid) {
    z = vl ? zl : vr ? zr : vu ? zu : vd ? zd : uRange.y;
  }

  float thr = uEdge * z;
  bool edge = (vl && abs(zl - z) > thr) || (vr && abs(zr - z) > thr) ||
              (vu && abs(zu - z) > thr) || (vd && abs(zd - z) > thr);

  vec3 p = rsPoint(fp, z);

  // Normal por diferencias centrales (solo con vecinos coherentes)
  vec3 pl = (vl && abs(zl - z) <= thr) ? rsPoint(fp + vec2(-1, 0), zl) : p;
  vec3 pr = (vr && abs(zr - z) <= thr) ? rsPoint(fp + vec2( 1, 0), zr) : p;
  vec3 pu = (vu && abs(zu - z) <= thr) ? rsPoint(fp + vec2( 0,-1), zu) : p;
  vec3 pd = (vd && abs(zd - z) <= thr) ? rsPoint(fp + vec2( 0, 1), zd) : p;
  vec3 n = cross(pr - pl, pd - pu);
  n = dot(n, n) > 1e-12 ? normalize(n) : vec3(0.0, 0.0, -1.0);
  vN = toWorld(n);

  // UV en la imagen de color (sin alineación en CPU)
  vec3 pc = uR * p + uT;
  vUv = vec2(pc.x / pc.z * uCIntr.x + uCIntr.z, pc.y / pc.z * uCIntr.y + uCIntr.w) / uCSize;

  // ¿La cámara de referencia ya ve esta misma superficie? -> no duplicar
  if (uRefOn > 0.5 && valid) {
    vec3 q = (uToRef * vec4(p, 1.0)).xyz;
    if (q.z > 0.0) {
      vec2 rp = vec2(q.x / q.z * uRefDIntr.x + uRefDIntr.z, q.y / q.z * uRefDIntr.y + uRefDIntr.w);
      if (rp.x >= 0.0 && rp.y >= 0.0 && rp.x < uRefDSize.x && rp.y < uRefDSize.y) {
        float zr = float(texelFetch(uRefDepth, ivec2(rp), 0).r) * uRefDScale;
        if (okZ(zr) && abs(zr - q.z) < max(0.01, uRefTol * q.z)) valid = false;
      }
    }
  }

  vZ = z;
  vValid = valid ? 1.0 : 0.0;
  vEdge  = edge ? 1.0 : 0.0;

  vec4 mv = modelViewMatrix * vec4(toWorld(p), 1.0);
  gl_Position = projectionMatrix * mv;

#ifdef POINTS
  if (!valid || (uFlyFilter > 0.5 && edge)) {
    gl_Position = vec4(2.0, 2.0, 2.0, 1.0);  // fuera del volumen de recorte
    gl_PointSize = 0.0;
    return;
  }
  // Cada punto cubre la huella de su píxel: la nube se ve continua sea cual sea la decimación
  float world = uPointScale * z / uDIntr.x;
  gl_PointSize = clamp(world * uProjScale / max(-mv.z, 1e-3), 1.0, 48.0);
#endif
}
`;

export const fragmentShader = /* glsl */ `
precision highp float;

uniform sampler2D uColor;
uniform float uHasColor;
uniform int   uColorMode;   // 0 RGB, 1 profundidad, 2 normales, 3 arcilla, 4 solape (tinte por cámara)
uniform vec3  uTint;
uniform vec2  uRange;
uniform float uRound;

in vec2  vUv;
in float vZ;
in vec3  vN;
in float vValid;
in float vEdge;

out vec4 fragColor;

// Turbo colormap (Google, aproximación polinómica)
vec3 turbo(float x) {
  const vec4 kR4 = vec4(0.13572138, 4.61539260, -42.66032258, 132.13108234);
  const vec4 kG4 = vec4(0.09140261, 2.19418839, 4.84296658, -14.18503333);
  const vec4 kB4 = vec4(0.10667330, 12.64194608, -60.58204836, 110.36276771);
  const vec2 kR2 = vec2(-152.94239396, 59.28637943);
  const vec2 kG2 = vec2(4.27729857, 2.82956604);
  const vec2 kB2 = vec2(-89.90310912, 27.34824973);
  x = clamp(x, 0.0, 1.0);
  vec4 v4 = vec4(1.0, x, x * x, x * x * x);
  vec2 v2 = v4.zw * v4.z;
  return vec3(dot(v4, kR4) + dot(v2, kR2), dot(v4, kG4) + dot(v2, kG2), dot(v4, kB4) + dot(v2, kB2));
}

void main() {
#ifdef MESH
  // Triángulo con algún vértice inválido o de borde -> no existe
  if (vValid < 0.999 || vEdge > 0.001) discard;
#else
  if (uRound > 0.5) {
    vec2 c = gl_PointCoord - 0.5;
    if (dot(c, c) > 0.25) discard;
  }
#endif

  float t = (vZ - uRange.x) / max(uRange.y - uRange.x, 1e-3);
  vec3 n = normalize(vN);
  vec3 col;

  if (uColorMode == 0) {
    bool inside = all(greaterThanEqual(vUv, vec2(0.0))) && all(lessThanEqual(vUv, vec2(1.0)));
    col = (uHasColor > 0.5 && inside) ? texture(uColor, vUv).rgb : turbo(t) * 0.55;
  } else if (uColorMode == 1) {
    col = turbo(1.0 - t);
  } else if (uColorMode == 2) {
    col = n * 0.5 + 0.5;
  } else {
    vec3 L1 = normalize(vec3(0.4, 0.7, 0.6));
    vec3 L2 = normalize(vec3(-0.6, 0.2, 0.3));
    float d = max(dot(n, L1), 0.0) * 0.85 + max(dot(n, L2), 0.0) * 0.3;
    float hemi = 0.5 + 0.5 * n.y;
    vec3 clay = uColorMode == 4 ? uTint : vec3(0.86, 0.80, 0.74);
    col = clay * (0.18 + d) + vec3(0.10, 0.13, 0.20) * hemi;
  }
  fragColor = vec4(col, 1.0);
}
`;
