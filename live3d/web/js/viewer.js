import * as THREE from 'three';
import { OrbitControls } from 'three/addons/controls/OrbitControls.js';
import { vertexShader, fragmentShader } from './shaders.js';

const COLOR_MODES = { rgb: 0, depth: 1, normals: 2, clay: 3, overlap: 4 };
const TINTS = [new THREE.Color(0x22d3ee), new THREE.Color(0xe040fb)];  // cian / magenta
// Cambio de base RealSense (x der, y abajo, z delante) <-> Three.js (y arriba, z hacia el visor)
const C = new THREE.Matrix4().makeScale(1, -1, -1);

// Textura R16UI 1x1 para el sampler de referencia mientras Cam 1 no tiene datos
function makeEmptyDepthTex() {
  const t = new THREE.DataTexture(new Uint16Array(1), 1, 1, THREE.RedIntegerFormat, THREE.UnsignedShortType);
  t.internalFormat = 'R16UI';
  t.minFilter = t.magFilter = THREE.NearestFilter;
  t.generateMipmaps = false;
  t.unpackAlignment = 2;
  t.needsUpdate = true;
  return t;
}
const EMPTY_DEPTH = makeEmptyDepthTex();

/**
 * Una cámara = su rejilla W×H, sus texturas (profundidad R16UI + color) y sus
 * materiales. Los uniforms de ajustes globales se comparten por referencia.
 */
class CameraRenderNode {
  constructor(scene, camIdx, shared) {
    this.camIdx = camIdx;
    this.group = new THREE.Group();
    this.group.matrixAutoUpdate = false;
    scene.add(this.group);
    this.uniforms = {
      ...shared,
      uDepth: { value: null },
      uDSize: { value: new THREE.Vector2(1, 1) },
      uDIntr: { value: new THREE.Vector4(1, 1, 0, 0) },
      uDScale: { value: 0.001 },
      uColor: { value: null },
      uHasColor: { value: 0 },
      uCIntr: { value: new THREE.Vector4(1, 1, 0, 0) },
      uCSize: { value: new THREE.Vector2(1, 1) },
      uR: { value: new THREE.Matrix3() },
      uT: { value: new THREE.Vector3() },
      uTint: { value: TINTS[camIdx] || TINTS[0] },
      uRefOn: { value: 0 },
      uRefDepth: { value: EMPTY_DEPTH },
      uRefDSize: { value: new THREE.Vector2(1, 1) },
      uRefDIntr: { value: new THREE.Vector4(1, 1, 0, 0) },
      uRefDScale: { value: 0.001 },
      uToRef: { value: new THREE.Matrix4() },
      uRefTol: { value: 0.03 },
    };
    const common = { uniforms: this.uniforms, vertexShader, fragmentShader, glslVersion: THREE.GLSL3 };
    this.pointsMat = new THREE.ShaderMaterial({ ...common, defines: { POINTS: '' } });
    this.meshMat = new THREE.ShaderMaterial({ ...common, defines: { MESH: '' }, side: THREE.DoubleSide });
    this.points = null;
    this.mesh = null;
    this.size = [0, 0];
    this.colorSize = [0, 0];
    this.depthTex = null;
    this.colorTex = null;
    this.bitmap = null;
    this.mode = 'points';
    this.enabled = true;
    this.hasData = false;
    this.gizmo = new THREE.Group();
    this.group.add(this.gizmo);
  }

  _buildGrid(w, h) {
    for (const o of [this.points, this.mesh]) {
      if (o) { this.group.remove(o); o.geometry.dispose(); }
    }
    const pos = new Float32Array(w * h * 3);
    let k = 0;
    for (let y = 0; y < h; y++) for (let x = 0; x < w; x++) { pos[k++] = x; pos[k++] = y; pos[k++] = 0; }
    const attr = new THREE.BufferAttribute(pos, 3);
    const idx = new Uint32Array((w - 1) * (h - 1) * 6);
    k = 0;
    for (let y = 0; y < h - 1; y++) {
      for (let x = 0; x < w - 1; x++) {
        const a = y * w + x, b = a + 1, c = a + w, d = c + 1;
        idx[k++] = a; idx[k++] = c; idx[k++] = b;
        idx[k++] = b; idx[k++] = c; idx[k++] = d;
      }
    }
    const pg = new THREE.BufferGeometry(); pg.setAttribute('position', attr);
    const mg = new THREE.BufferGeometry(); mg.setAttribute('position', attr);
    mg.setIndex(new THREE.BufferAttribute(idx, 1));
    this.points = new THREE.Points(pg, this.pointsMat);
    this.mesh = new THREE.Mesh(mg, this.meshMat);
    for (const o of [this.points, this.mesh]) { o.frustumCulled = false; this.group.add(o); }
    this.setMode(this.mode);
  }

  _buildFrustum(meta) {
    this.gizmo.clear();
    const d = meta.depth, L = 0.12;
    const corners = [[0, 0], [d.width, 0], [d.width, d.height], [0, d.height]].map(([u, v]) =>
      new THREE.Vector3((u - d.ppx) / d.fx * L, -(v - d.ppy) / d.fy * L, -L));
    const pts = [];
    for (const c of corners) pts.push(new THREE.Vector3(), c);
    for (let i = 0; i < 4; i++) pts.push(corners[i], corners[(i + 1) % 4]);
    const g = new THREE.BufferGeometry().setFromPoints(pts);
    const m = new THREE.LineBasicMaterial({ color: this.uniforms.uTint.value, transparent: true, opacity: 0.85 });
    this.gizmo.add(new THREE.LineSegments(g, m));
    const body = new THREE.Mesh(new THREE.BoxGeometry(0.124, 0.029, 0.026),
      new THREE.MeshBasicMaterial({ color: 0x0b1220 }));
    body.position.z = 0.014;
    const edges = new THREE.LineSegments(new THREE.EdgesGeometry(body.geometry),
      new THREE.LineBasicMaterial({ color: 0x8b5cf6 }));
    edges.position.copy(body.position);
    this.gizmo.add(body, edges);
  }

  setMeta(meta) {
    const d = meta.depth, c = meta.color, u = this.uniforms;
    if (d.width !== this.size[0] || d.height !== this.size[1]) {
      this.size = [d.width, d.height];
      this._buildGrid(d.width, d.height);
      if (this.depthTex) this.depthTex.dispose();
      const tex = new THREE.DataTexture(new Uint16Array(d.width * d.height), d.width, d.height,
        THREE.RedIntegerFormat, THREE.UnsignedShortType);
      tex.internalFormat = 'R16UI';
      tex.minFilter = tex.magFilter = THREE.NearestFilter;
      tex.generateMipmaps = false;
      tex.flipY = false;
      tex.unpackAlignment = 2;
      tex.needsUpdate = true;
      this.depthTex = tex;
      u.uDepth.value = tex;
    }
    u.uDSize.value.set(d.width, d.height);
    u.uDIntr.value.set(d.fx, d.fy, d.ppx, d.ppy);
    u.uDScale.value = d.scale;
    if (c) {
      u.uCIntr.value.set(c.fx, c.fy, c.ppx, c.ppy);
      u.uCSize.value.set(c.width, c.height);
    }
    const r = meta.depth_to_color.rotation, t = meta.depth_to_color.translation;
    u.uR.value.set(r[0], r[1], r[2], r[3], r[4], r[5], r[6], r[7], r[8]);  // fila-mayor
    u.uT.value.set(t[0], t[1], t[2]);
    this._buildFrustum(meta);
  }

  setFrame(depth, color) {
    if (!this.depthTex || depth.length !== this.size[0] * this.size[1]) return false;
    this.depthTex.image.data = depth;
    this.depthTex.needsUpdate = true;
    const u = this.uniforms;
    if (color) {
      if (!this.colorTex || color.width !== this.colorSize[0] || color.height !== this.colorSize[1]) {
        if (this.colorTex) this.colorTex.dispose();
        this.colorTex = new THREE.Texture(color);
        this.colorTex.flipY = false;
        this.colorTex.generateMipmaps = false;
        this.colorTex.minFilter = this.colorTex.magFilter = THREE.LinearFilter;
        this.colorSize = [color.width, color.height];
        u.uColor.value = this.colorTex;
      } else {
        this.colorTex.image = color;
      }
      this.colorTex.needsUpdate = true;
      if (this.bitmap && this.bitmap !== color) this.bitmap.close();
      this.bitmap = color;
      u.uHasColor.value = 1;
    } else {
      u.uHasColor.value = 0;
    }
    this.hasData = true;
    this._applyVisibility();
    return true;
  }

  setMode(mode) { this.mode = mode; this._applyVisibility(); }
  setEnabled(on) { this.enabled = on; this._applyVisibility(); }
  _applyVisibility() {
    const show = this.enabled && this.hasData;
    if (this.points) this.points.visible = show && this.mode === 'points';
    if (this.mesh) this.mesh.visible = show && this.mode === 'mesh';
    this.gizmo.visible = this.enabled && this._gizmoOn !== false;
  }
  setGizmo(on) { this._gizmoOn = on; this._applyVisibility(); }

  /** m: matriz 4x4 fila-mayor en coordenadas RealSense (cam -> cam 1). */
  setTransformRS(m) {
    const T = new THREE.Matrix4().set(...m.flat());
    this.uniforms.uToRef.value.copy(T);  // para la deduplicación en GPU (coordenadas RealSense)
    this.group.matrix.copy(C).multiply(T).multiply(C);  // C·T·C⁻¹ (C es su propia inversa)
    this.group.matrixWorldNeedsUpdate = true;
  }
}

/**
 * Visor GPU de profundidad en vivo con hasta dos cámaras en la misma escena.
 * Cam 1 en el origen; Cam 2 colocada con la extrínseca T(2->1).
 */
export class Viewer {
  constructor(canvas) {
    this.canvas = canvas;
    this.renderer = new THREE.WebGLRenderer({
      canvas, antialias: true, powerPreference: 'high-performance', alpha: false,
    });
    this.renderer.setPixelRatio(Math.min(window.devicePixelRatio, 2));
    this.renderer.setClearColor(0x05070d, 1);

    this.scene = new THREE.Scene();
    this.scene.fog = new THREE.Fog(0x05070d, 6, 14);
    this.camera = new THREE.PerspectiveCamera(55, 1, 0.01, 60);
    this.controls = new OrbitControls(this.camera, canvas);
    this.controls.enableDamping = true;
    this.controls.dampingFactor = 0.08;
    this.controls.screenSpacePanning = true;
    this.controls.autoRotateSpeed = 0.8;

    // Ajustes globales compartidos por todas las cámaras
    this.shared = {
      uRange: { value: new THREE.Vector2(0.2, 4.0) },
      uPointScale: { value: 1.25 },
      uProjScale: { value: 500 },
      uEdge: { value: 0.04 },
      uFlyFilter: { value: 1 },
      uColorMode: { value: 0 },
      uRound: { value: 0 },
    };
    this.nodes = [new CameraRenderNode(this.scene, 0, this.shared),
                  new CameraRenderNode(this.scene, 1, this.shared)];
    this.mode = 'points';

    this.grid = new THREE.GridHelper(8, 32, 0x1e3a5f, 0x0f1d33);
    this.grid.position.y = -0.8;
    this.grid.material.transparent = true;
    this.grid.material.opacity = 0.55;
    this.scene.add(this.grid);

    this.resetView('front', true);
    this._resize();
    new ResizeObserver(() => this._resize()).observe(canvas.parentElement);
  }

  _node(meta) { return this.nodes[(meta && meta.cam_idx) === 1 ? 1 : 0]; }

  // ------------------------------------------------------------------ datos
  setMeta(meta) { this._node(meta).setMeta(meta); }
  /** cam: 0 | 1. depth: Uint16Array. color: ImageBitmap | null */
  setFrame(depth, color, cam = 0) { return this.nodes[cam === 1 ? 1 : 0].setFrame(depth, color); }

  /** Qué cámaras se dibujan: 'dual' | 'cam1' | 'cam2' */
  setView(view) {
    this.nodes[0].setEnabled(view !== 'cam2');
    this.nodes[1].setEnabled(view !== 'cam1');
  }
  setExtrinsics(m4x4) { this.nodes[1].setTransformRS(m4x4); }
  /** Sensor virtual único: Cam 2 solo completa lo que Cam 1 no ve (sin duplicados). */
  setDedup(on) { this.dedup = !!on; }
  _syncDedup() {
    const ref = this.nodes[0], u = this.nodes[1].uniforms, r = ref.uniforms;
    const on = this.dedup !== false && ref.enabled && ref.hasData && !!ref.depthTex
      && this.shared.uColorMode.value !== COLOR_MODES.overlap;  // en "Solape" se muestran ambas para calibrar
    u.uRefOn.value = on ? 1 : 0;
    u.uRefDepth.value = on ? ref.depthTex : EMPTY_DEPTH;
    u.uRefDSize.value.copy(r.uDSize.value);
    u.uRefDIntr.value.copy(r.uDIntr.value);
    u.uRefDScale.value = r.uDScale.value;
  }

  // ------------------------------------------------------------------ opciones
  setMode(mode) { this.mode = mode; for (const n of this.nodes) n.setMode(mode); }
  setColorMode(name) { this.shared.uColorMode.value = COLOR_MODES[name] ?? 0; }
  setRange(near, far) { this.shared.uRange.value.set(near, far); }
  setPointScale(v) { this.shared.uPointScale.value = v; }
  setEdge(v) { this.shared.uEdge.value = v; }
  setRound(on) { this.shared.uRound.value = on ? 1 : 0; }
  setFlyFilter(on) { this.shared.uFlyFilter.value = on ? 1 : 0; }
  setGrid(on) { this.grid.visible = on; }
  setGizmo(on) { for (const n of this.nodes) n.setGizmo(on); }
  setAutoRotate(on) { this.controls.autoRotate = on; }

  resetView(preset = 'front', instant = false) {
    const target = new THREE.Vector3(0, 0, -1.4);
    const views = {
      front: new THREE.Vector3(0, 0.05, 0.25),
      side: new THREE.Vector3(1.9, 0.25, -1.3),
      top: new THREE.Vector3(0, 2.3, -1.25),
      orbit: new THREE.Vector3(1.2, 0.7, 0.4),
    };
    const to = views[preset] || views.front;
    if (instant) {
      this.camera.position.copy(to);
      this.controls.target.copy(target);
      this.controls.update();
      return;
    }
    const from = this.camera.position.clone(), tFrom = this.controls.target.clone();
    const t0 = performance.now(), dur = 650;
    const step = () => {
      const k = Math.min(1, (performance.now() - t0) / dur);
      const e = 1 - Math.pow(1 - k, 3);
      this.camera.position.lerpVectors(from, to, e);
      this.controls.target.lerpVectors(tFrom, target, e);
      if (k < 1) requestAnimationFrame(step);
    };
    step();
  }

  _resize() {
    const el = this.canvas.parentElement;
    const w = el.clientWidth, h = el.clientHeight;
    this.renderer.setSize(w, h, false);
    this.camera.aspect = w / Math.max(1, h);
    this.camera.updateProjectionMatrix();
    const hPx = h * this.renderer.getPixelRatio();
    this.shared.uProjScale.value = hPx / (2 * Math.tan(THREE.MathUtils.degToRad(this.camera.fov) / 2));
  }

  render() {
    this.controls.update();
    this._syncDedup();
    this.renderer.render(this.scene, this.camera);
  }

  screenshot() {
    this.render();
    return new Promise((res) => this.canvas.toBlob(res, 'image/png'));
  }

  get currentBitmap() { return this.nodes[0].bitmap || this.nodes[1].bitmap; }
}
