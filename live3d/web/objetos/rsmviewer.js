// Visor del resultado: lee el mismo modelo.rsm que descarga Unity y lo muestra con Three.js.
// RSM está en la convención de Unity (zurdo): aquí se vuelve a diestra (z -> -z) y se invierte
// el orden de los triángulos.
import * as THREE from 'three';
import { OrbitControls } from 'three/addons/controls/OrbitControls.js';

export function parseRSM(buf) {
  const dv = new DataView(buf);
  const magic = String.fromCharCode(dv.getUint8(0), dv.getUint8(1), dv.getUint8(2), dv.getUint8(3));
  if (magic !== 'RSM1') throw new Error('no es un archivo RSM');
  const nv = dv.getUint32(8, true), ni = dv.getUint32(12, true), flags = dv.getUint32(16, true);
  let o = 44;
  const pos = new Float32Array(buf.slice(o, o + nv * 12)); o += nv * 12;
  let nrm = null, col = null;
  if (flags & 1) { nrm = new Float32Array(buf.slice(o, o + nv * 12)); o += nv * 12; }
  if (flags & 2) { col = new Uint8Array(buf.slice(o, o + nv * 4)); o += nv * 4; }
  const idx = new Uint32Array(buf.slice(o, o + ni * 4));
  for (let i = 2; i < pos.length; i += 3) pos[i] = -pos[i];
  if (nrm) for (let i = 2; i < nrm.length; i += 3) nrm[i] = -nrm[i];
  for (let i = 0; i < idx.length; i += 3) { const t = idx[i]; idx[i] = idx[i + 2]; idx[i + 2] = t; }
  const g = new THREE.BufferGeometry();
  g.setAttribute('position', new THREE.BufferAttribute(pos, 3));
  if (nrm) g.setAttribute('normal', new THREE.BufferAttribute(nrm, 3));
  if (col) g.setAttribute('color', new THREE.BufferAttribute(col, 4, true));
  g.setIndex(new THREE.BufferAttribute(idx, 1));
  if (!nrm) g.computeVertexNormals();
  g.computeBoundingBox();
  return g;
}

export class RSMViewer {
  constructor(canvas) {
    this.canvas = canvas;
    this.renderer = new THREE.WebGLRenderer({ canvas, antialias: true });
    this.renderer.setPixelRatio(Math.min(devicePixelRatio, 2));
    this.renderer.outputColorSpace = THREE.SRGBColorSpace;
    this.scene = new THREE.Scene();
    this.scene.background = new THREE.Color(0x05070d);
    this.camera = new THREE.PerspectiveCamera(40, 1, 0.01, 50);
    this.controls = new OrbitControls(this.camera, canvas);
    this.controls.enableDamping = true;
    this.scene.add(new THREE.HemisphereLight(0xdfe8ff, 0x20242c, 1.2));
    const key = new THREE.DirectionalLight(0xffffff, 1.6);
    key.position.set(1, 2, 1.5);
    this.scene.add(key);
    const grid = new THREE.GridHelper(1, 20, 0x22d3ee, 0x1b2333);
    grid.material.opacity = 0.35; grid.material.transparent = true;
    this.scene.add(grid);
    this.mesh = null;
    this._loop = this._loop.bind(this);
    this._running = false;
  }

  async load(url) {
    const r = await fetch(url);
    if (!r.ok) throw new Error(`no se pudo descargar el modelo (${r.status})`);
    const g = parseRSM(await r.arrayBuffer());
    if (this.mesh) { this.scene.remove(this.mesh); this.mesh.geometry.dispose(); }
    const mat = new THREE.MeshStandardMaterial({ vertexColors: !!g.getAttribute('color'), roughness: 0.75, metalness: 0.0,
                                                 color: g.getAttribute('color') ? 0xffffff : 0xb8c4d6 });
    this.mesh = new THREE.Mesh(g, mat);
    this.scene.add(this.mesh);
    const bb = g.boundingBox, size = new THREE.Vector3(), c = new THREE.Vector3();
    bb.getSize(size); bb.getCenter(c);
    const r0 = Math.max(size.x, size.y, size.z);
    this.controls.target.copy(c);
    this.camera.position.set(c.x + r0 * 1.3, c.y + r0 * 0.8, c.z + r0 * 1.6);
    this.camera.near = r0 / 100; this.camera.far = r0 * 50; this.camera.updateProjectionMatrix();
    this.start();
  }

  start() { if (!this._running) { this._running = true; requestAnimationFrame(this._loop); } }
  stop() { this._running = false; }

  _loop() {
    if (!this._running) return;
    const w = this.canvas.clientWidth, h = this.canvas.clientHeight;
    if (this.canvas.width !== Math.floor(w * this.renderer.getPixelRatio()) || this.canvas.height !== Math.floor(h * this.renderer.getPixelRatio())) {
      this.renderer.setSize(w, h, false);
      this.camera.aspect = w / Math.max(1, h);
      this.camera.updateProjectionMatrix();
    }
    this.controls.update();
    this.renderer.render(this.scene, this.camera);
    requestAnimationFrame(this._loop);
  }
}
