import { Viewer } from './viewer.js';
import { LiveStream } from './stream.js';

window.onerror = function(msg, url, line, col, error) {
  fetch('/api/client_log', {
    method: 'POST',
    headers: { 'Content-Type': 'application/json' },
    body: JSON.stringify({ type: 'window_error', msg, url, line, col, stack: error ? error.stack : null })
  }).catch(() => {});
};
window.onunhandledrejection = function(e) {
  fetch('/api/client_log', {
    method: 'POST',
    headers: { 'Content-Type': 'application/json' },
    body: JSON.stringify({ type: 'unhandled_rejection', reason: String(e.reason), stack: e.reason ? e.reason.stack : null })
  }).catch(() => {});
};

// ---------------------------------------------------------------------------
// Inicialización principal
// ---------------------------------------------------------------------------
const canvas = document.getElementById('view-canvas');
const viewer = new Viewer(canvas);

const proto = location.protocol === 'https:' ? 'wss:' : 'ws:';
const wsUrl = `${proto}//${location.host}/ws`;
const stream = new LiveStream(wsUrl);

// Elementos DOM
const appEl = document.getElementById('app');
const deviceLabel = document.getElementById('device-label');
const connPill = document.getElementById('conn-pill');
const connText = document.getElementById('conn-text');
const overlay = document.getElementById('overlay');
const overlayTitle = document.getElementById('overlay-title');
const overlayMsg = document.getElementById('overlay-msg');
const toastEl = document.getElementById('toast');

// Stats HUD
const stStream = document.getElementById('st-stream');
const stRender = document.getElementById('st-render');
const stPoints = document.getElementById('st-points');
const stMbps = document.getElementById('st-mbps');
const stLat = document.getElementById('st-lat');
const stCam = document.getElementById('st-cam');

// Panel KV
const kvProfile = document.getElementById('kv-profile');
const kvGeo = document.getElementById('kv-geo');
const kvProc = document.getElementById('kv-proc');
const kvEnc = document.getElementById('kv-enc');
const kvFrame = document.getElementById('kv-frame');
const kvDrop = document.getElementById('kv-drop');
const kvClients = document.getElementById('kv-clients');

// Estado interno
let currentMeta = null;
let lastFrameData = null;
let lastFrameTime = performance.now();
let frameCount = 0;
let bytesSinceLastSec = 0;
let lastSecTime = performance.now();
let streamFps = 0;
let renderFps = 0;
let renderFrames = 0;
let lastRenderTime = performance.now();
let isUserDraggingSlider = false;

// ---------------------------------------------------------------------------
// Toast Notification
// ---------------------------------------------------------------------------
let toastTimer = null;
function showToast(msg, isError = false) {
  if (!toastEl) return;
  toastEl.textContent = msg;
  toastEl.className = 'toast show' + (isError ? ' err' : '');
  clearTimeout(toastTimer);
  toastTimer = setTimeout(() => {
    toastEl.className = 'toast';
  }, 3000);
}

// ---------------------------------------------------------------------------
// Helpers para sliders con barra de progreso estilizada (--p)
// ---------------------------------------------------------------------------
function updateSliderProgress(slider) {
  const min = parseFloat(slider.min) || 0;
  const max = parseFloat(slider.max) || 100;
  const val = parseFloat(slider.value) || 0;
  const pct = Math.max(0, Math.min(100, ((val - min) / (max - min)) * 100));
  slider.style.setProperty('--p', `${pct}%`);
}

document.querySelectorAll('input[type="range"]').forEach(slider => {
  updateSliderProgress(slider);
  slider.addEventListener('input', () => updateSliderProgress(slider));
});

// ---------------------------------------------------------------------------
// Eventos del Stream WebSocket
// ---------------------------------------------------------------------------
stream.on('state', (state) => {
  if (state === 'open') {
    connPill.setAttribute('data-state', 'starting');
    connText.textContent = 'Sincronizando';
    overlayTitle.textContent = 'Conectado al servidor';
    overlayMsg.textContent = 'Inicializando flujo de geometría 3D...';
  } else if (state === 'closed') {
    connPill.setAttribute('data-state', 'closed');
    connText.textContent = 'Desconectado';
    overlay.classList.remove('hidden');
    overlayTitle.textContent = 'Conexión perdida';
    overlayMsg.textContent = 'Reintentando conexión con el servidor RealSense...';
  } else if (state === 'connecting') {
    connPill.setAttribute('data-state', 'connecting');
    connText.textContent = 'Conectando';
    overlay.classList.remove('hidden');
    overlayTitle.textContent = 'Conectando con el servidor';
    overlayMsg.textContent = 'Esperando socket en ' + wsUrl;
  }
});

stream.on('source_changed', (msg) => {
  showToast(msg.message || 'Fuente de cámara cambiada');
  checkDualCameraStatus();
});

stream.on('meta', (meta) => {
  currentMeta = meta;
  viewer.setMeta(meta);
  overlay.classList.add('hidden');
  connPill.setAttribute('data-state', 'live');
  connText.textContent = 'En Vivo';

  const d = meta.depth;
  const c = meta.color;
  stPoints.textContent = `${(d.width * d.height).toLocaleString()}`;
  kvGeo.textContent = `D ${d.width}×${d.height} · RGB ${c ? `${c.width}×${c.height}` : 'N/A'}`;

  const decSpan = document.getElementById('dec-res');
  if (decSpan) decSpan.textContent = `(${d.width}×${d.height})`;
});

stream.on('frame', (frame) => {
  lastFrameData = frame;
  const accepted = viewer.setFrame(frame.depth, frame.color, frame.cam ?? 0);
  stream.ack(); // Liberar crédito de flujo de inmediato

  if (accepted) {
    frameCount++;
    bytesSinceLastSec += frame.bytes;

    // Latencia estimada si hay reloj razonablemente sincronizado
    if (frame.timestamp) {
      const nowS = Date.now() / 1000;
      const latMs = Math.max(0, Math.round((nowS - frame.timestamp) * 1000));
      if (latMs < 10000) {
        stLat.textContent = latMs;
      }
    }

    if (kvFrame) {
      kvFrame.textContent = `${(frame.bytes / 1024).toFixed(1)} KB`;
    }
  }
});

stream.on('status', (msg) => {
  if (msg.camera) {
    const cam = msg.camera;
    deviceLabel.textContent = `${cam.device || 'Cámara'} · ${cam.profile || cam.message || ''}`;
    stCam.textContent = cam.fps ? cam.fps.toFixed(1) : '–';
    kvProfile.textContent = cam.profile || '–';
    kvProc.textContent = cam.proc_ms ? `${cam.proc_ms} ms` : '–';

    const camsTxt = (cam.cams || []).map(c => `Cam ${c.idx + 1}: ${c.state === 'streaming' ? (c.fps || 0).toFixed(0) + ' FPS' : c.message}`).join(' · ');
    if (camsTxt) deviceLabel.title = camsTxt;
    if (cam.state === 'no_device') {
      connPill.setAttribute('data-state', 'no_device');
      connText.textContent = 'Sin Cámara';
      overlay.classList.remove('hidden');
      overlayTitle.textContent = 'Cámara No Detectada';
      overlayMsg.textContent = cam.message || 'Verifica que la cámara esté conectada al puerto USB 3.0.';
    } else if (cam.state === 'error') {
      connPill.setAttribute('data-state', 'error');
      connText.textContent = 'Error';
    } else if (cam.state === 'degraded' && !stream.paused) {
      connPill.setAttribute('data-state', 'starting');
      connText.textContent = '1 de 2 cámaras';
      connPill.title = cam.message || '';
    } else if (cam.state === 'streaming' && !stream.paused) {
      connPill.setAttribute('data-state', cam.simulated ? 'paused' : 'live');
      connText.textContent = cam.simulated ? 'Simulación' : 'En Vivo';
      connPill.title = cam.simulated ? 'Sin RealSense conectadas: escena sintética. Cambia sola a las cámaras al conectarlas.' : '';
    }
  }

  if (msg.encode_ms !== undefined && kvEnc) {
    kvEnc.textContent = `${msg.encode_ms} ms`;
  }
  if (msg.clients !== undefined && kvClients) {
    kvClients.textContent = `${msg.clients}`;
  }

  // Sincronizar ajustes del servidor si el usuario no los está moviendo
  if (msg.settings && !isUserDraggingSlider) {
    syncServerSettingsUI(msg.settings);
  }

  if (msg.you) {
    if (msg.you.drop_fps !== undefined && kvDrop) {
      kvDrop.textContent = `${msg.you.drop_fps} fps`;
    }
  }
});

stream.on('error', (err) => {
  showToast(err, true);
});

// Sincronización de UI con settings remotos
function syncServerSettingsUI(s) {
  const selProf = document.getElementById('sel-profile');
  if (selProf && s.depth_res && s.color_res && s.fps) {
    const targetVal = `${s.depth_res[0]},${s.depth_res[1]},${s.color_res[0]},${s.color_res[1]},${s.fps}`;
    for (const opt of selProf.options) {
      if (opt.value === targetVal) {
        selProf.value = targetVal;
        break;
      }
    }
  }

  // Decimación
  if (s.decimation) {
    document.querySelectorAll('#seg-dec button').forEach(b => {
      b.classList.toggle('on', parseInt(b.dataset.v) === s.decimation);
    });
  }

  // Distancia
  if (s.near !== undefined) {
    const rn = document.getElementById('rng-near');
    const on = document.getElementById('out-near');
    if (rn && on) {
      rn.value = s.near;
      on.textContent = `${s.near.toFixed(2)} m`;
      updateSliderProgress(rn);
    }
  }
  if (s.far !== undefined) {
    const rf = document.getElementById('rng-far');
    const of = document.getElementById('out-far');
    if (rf && of) {
      rf.value = s.far;
      of.textContent = `${s.far.toFixed(2)} m`;
      updateSliderProgress(rf);
    }
  }

  // Preset
  const selPreset = document.getElementById('sel-preset');
  if (selPreset && s.preset) selPreset.value = s.preset;

  // Toggles de filtros
  const tgTemp = document.getElementById('tg-temporal');
  if (tgTemp && s.temporal !== undefined) tgTemp.checked = !!s.temporal;
  const tgSpat = document.getElementById('tg-spatial');
  if (tgSpat && s.spatial !== undefined) tgSpat.checked = !!s.spatial;
  const tgHole = document.getElementById('tg-hole');
  if (tgHole && s.hole_fill !== undefined) tgHole.checked = !!s.hole_fill;
  const tgEmit = document.getElementById('tg-emitter');
  if (tgEmit && s.emitter !== undefined) tgEmit.checked = !!s.emitter;

  // Calidad JPEG
  if (s.jpeg_quality !== undefined) {
    const rj = document.getElementById('rng-jpeg');
    const oj = document.getElementById('out-jpeg');
    if (rj && oj) {
      rj.value = s.jpeg_quality;
      oj.textContent = s.jpeg_quality;
      updateSliderProgress(rj);
    }
  }
}

// ---------------------------------------------------------------------------
// Loop de renderizado y cálculo de estadísticas por segundo
// ---------------------------------------------------------------------------
function animate() {
  requestAnimationFrame(animate);

  viewer.render();
  renderFrames++;

  const now = performance.now();
  if (now - lastSecTime >= 1000) {
    const dt = (now - lastSecTime) / 1000;
    streamFps = (frameCount / dt).toFixed(1);
    renderFps = (renderFrames / dt).toFixed(1);
    const mbps = ((bytesSinceLastSec * 8) / dt / 1e6).toFixed(2);

    stStream.textContent = streamFps;
    stRender.textContent = renderFps;
    stMbps.textContent = mbps;

    frameCount = 0;
    renderFrames = 0;
    bytesSinceLastSec = 0;
    lastSecTime = now;
  }
}
animate();

// ---------------------------------------------------------------------------
// Controles de Visualización (Cliente Three.js)
// ---------------------------------------------------------------------------

// Modo: Puntos / Malla
const segMode = document.getElementById('seg-mode');
const ctrlPsize = document.getElementById('ctrl-psize');
const ctrlEdge = document.getElementById('ctrl-edge');

function setGeometryMode(mode) {
  viewer.setMode(mode);
  segMode.querySelectorAll('button').forEach(b => {
    b.classList.toggle('on', b.dataset.v === mode);
  });
  if (mode === 'points') {
    ctrlPsize.classList.remove('disabled');
  } else {
    ctrlPsize.classList.add('disabled');
  }
}
segMode.addEventListener('click', (e) => {
  const btn = e.target.closest('button');
  if (btn) setGeometryMode(btn.dataset.v);
});

// Modos de color: RGB, Prof, Normal, Arcilla
const segColor = document.getElementById('seg-color');
function setColorMode(colMode) {
  viewer.setColorMode(colMode);
  segColor.querySelectorAll('button').forEach(b => {
    b.classList.toggle('on', b.dataset.v === colMode);
  });
}
segColor.addEventListener('click', (e) => {
  const btn = e.target.closest('button');
  if (btn) setColorMode(btn.dataset.v);
});

// Sliders de visualización directa
const rngPsize = document.getElementById('rng-psize');
const outPsize = document.getElementById('out-psize');
rngPsize.addEventListener('input', () => {
  const val = parseFloat(rngPsize.value);
  outPsize.textContent = `${val.toFixed(2)}×`;
  viewer.setPointScale(val);
});

const rngEdge = document.getElementById('rng-edge');
const outEdge = document.getElementById('out-edge');
rngEdge.addEventListener('input', () => {
  const val = parseFloat(rngEdge.value);
  outEdge.textContent = `${Math.round(val * 100)}%`;
  viewer.setEdge(val);
});

// Toggles de visualización
document.getElementById('tg-fly').addEventListener('change', (e) => {
  viewer.setFlyFilter(e.target.checked);
});
document.getElementById('tg-round').addEventListener('change', (e) => {
  viewer.setRound(e.target.checked);
});
document.getElementById('tg-grid').addEventListener('change', (e) => {
  viewer.setGrid(e.target.checked);
});
document.getElementById('tg-gizmo').addEventListener('change', (e) => {
  viewer.setGizmo(e.target.checked);
});
document.getElementById('tg-rotate').addEventListener('change', (e) => {
  viewer.setAutoRotate(e.target.checked);
});

// ---------------------------------------------------------------------------
// Controles de Cámara (Envío al Servidor)
// ---------------------------------------------------------------------------

// Cambio de perfil (resolución y FPS)
const selProfile = document.getElementById('sel-profile');
selProfile.addEventListener('change', () => {
  const parts = selProfile.value.split(',').map(Number);
  if (parts.length === 5) {
    stream.sendSettings({
      depth_res: [parts[0], parts[1]],
      color_res: [parts[2], parts[3]],
      fps: parts[4],
    });
    showToast('Reconfigurando pipeline de cámara...');
  }
});

// Decimación
document.getElementById('seg-dec').addEventListener('click', (e) => {
  const btn = e.target.closest('button');
  if (btn) {
    const val = parseInt(btn.dataset.v);
    stream.sendSettings({ decimation: val });
  }
});

// Debounce helper
function debounce(fn, ms = 200) {
  let timer;
  return (...args) => {
    clearTimeout(timer);
    timer = setTimeout(() => fn(...args), ms);
  };
}

// Rango Near / Far
const rngNear = document.getElementById('rng-near');
const outNear = document.getElementById('out-near');
const rngFar = document.getElementById('rng-far');
const outFar = document.getElementById('out-far');

const sendRangeDebounced = debounce(() => {
  const near = parseFloat(rngNear.value);
  const far = parseFloat(rngFar.value);
  viewer.setRange(near, far);
  stream.sendSettings({ near, far });
  isUserDraggingSlider = false;
}, 150);

rngNear.addEventListener('input', () => {
  isUserDraggingSlider = true;
  const n = parseFloat(rngNear.value);
  outNear.textContent = `${n.toFixed(2)} m`;
  viewer.setRange(n, parseFloat(rngFar.value));
  sendRangeDebounced();
});

rngFar.addEventListener('input', () => {
  isUserDraggingSlider = true;
  const f = parseFloat(rngFar.value);
  outFar.textContent = `${f.toFixed(2)} m`;
  viewer.setRange(parseFloat(rngNear.value), f);
  sendRangeDebounced();
});

// Preset RealSense
document.getElementById('sel-preset').addEventListener('change', (e) => {
  stream.sendSettings({ preset: e.target.value });
});

// Filtros RealSense en servidor
document.getElementById('tg-temporal').addEventListener('change', (e) => {
  stream.sendSettings({ temporal: e.target.checked });
});
document.getElementById('tg-spatial').addEventListener('change', (e) => {
  stream.sendSettings({ spatial: e.target.checked });
});
document.getElementById('tg-hole').addEventListener('change', (e) => {
  stream.sendSettings({ hole_fill: e.target.checked });
});
document.getElementById('tg-emitter').addEventListener('change', (e) => {
  stream.sendSettings({ emitter: e.target.checked });
});

// Calidad JPEG
const rngJpeg = document.getElementById('rng-jpeg');
const outJpeg = document.getElementById('out-jpeg');
const sendJpegDebounced = debounce(() => {
  stream.sendSettings({ jpeg_quality: parseInt(rngJpeg.value) });
  isUserDraggingSlider = false;
}, 200);

rngJpeg.addEventListener('input', () => {
  isUserDraggingSlider = true;
  outJpeg.textContent = rngJpeg.value;
  sendJpegDebounced();
});

// ---------------------------------------------------------------------------
// Barra inferior (Dock)
// ---------------------------------------------------------------------------

// Congelar / Reanudar
const btnFreeze = document.getElementById('btn-freeze');
btnFreeze.addEventListener('click', () => {
  const willPause = !stream.paused;
  stream.setPaused(willPause);
  btnFreeze.classList.toggle('on', willPause);
  btnFreeze.querySelector('span').textContent = willPause ? 'Reanudar' : 'Congelar';
  connPill.setAttribute('data-state', willPause ? 'paused' : 'live');
  connText.textContent = willPause ? 'Pausado' : 'En Vivo';
  showToast(willPause ? 'Stream congelado' : 'Stream reanudado');
});

// Presets de vistas de cámara
document.querySelectorAll('.dock-btn[data-view]').forEach(btn => {
  btn.addEventListener('click', () => {
    viewer.resetView(btn.dataset.view);
  });
});

// Captura de pantalla PNG
document.getElementById('btn-shot').addEventListener('click', async () => {
  try {
    const blob = await viewer.screenshot();
    const url = URL.createObjectURL(blob);
    const a = document.createElement('a');
    a.href = url;
    a.download = `realsense_3d_${Date.now()}.png`;
    a.click();
    URL.revokeObjectURL(url);
    showToast('Captura PNG guardada');
  } catch (err) {
    showToast('Error al capturar imagen', true);
  }
});

// Guardar Escaneo nativo en Saves/ (formato RealSenseScans)
const btnSaveScan = document.getElementById('btn-save-scan');
let isSavingScan = false;

async function triggerSaveScan() {
  if (isSavingScan) return;
  isSavingScan = true;
  if (btnSaveScan) {
    btnSaveScan.disabled = true;
    btnSaveScan.classList.add('loading');
  }
  showToast('Capturando y guardando escaneo en Saves/…');
  try {
    const res = await fetch('/api/capture', { method: 'POST' });
    const data = await res.json();
    if (!res.ok || data.status === 'error') {
      throw new Error(data.message || 'Error al guardar el escaneo');
    }
    showToast(`✓ Guardado: ${data.rel_folder || data.filename}`);
  } catch (err) {
    console.error('Error guardando escaneo:', err);
    showToast(`Error al guardar: ${err.message}`, true);
  } finally {
    isSavingScan = false;
    if (btnSaveScan) {
      btnSaveScan.disabled = false;
      btnSaveScan.classList.remove('loading');
    }
  }
}

if (btnSaveScan) {
  btnSaveScan.addEventListener('click', triggerSaveScan);
}

// Exportar Nube PLY en coordenadas de Blender (Z arriba)
document.getElementById('btn-ply').addEventListener('click', () => {
  exportCurrentFrameToPly();
});

function exportCurrentFrameToPly() {
  if (!lastFrameData || !currentMeta || !lastFrameData.depth) {
    showToast('No hay datos 3D disponibles para exportar', true);
    return;
  }

  showToast('Generando archivo PLY para Blender...');
  setTimeout(() => {
    try {
      const meta = currentMeta;
      const d = meta.depth;
      const depth = lastFrameData.depth;
      const w = d.width;
      const h = d.height;
      const fx = d.fx, fy = d.fy, ppx = d.ppx, ppy = d.ppy;
      const scale = d.scale;
      const near = parseFloat(rngNear.value) || 0.1;
      const far = parseFloat(rngFar.value) || 5.0;

      // Extraer píxeles de color si hay ImageBitmap disponible en canvas temporal
      let colorData = null;
      const bmp = viewer.currentBitmap;
      if (bmp && meta.color) {
        const offCanvas = document.createElement('canvas');
        offCanvas.width = bmp.width;
        offCanvas.height = bmp.height;
        const ctx = offCanvas.getContext('2d');
        ctx.drawImage(bmp, 0, 0);
        colorData = ctx.getImageData(0, 0, bmp.width, bmp.height).data;
      }

      const c = meta.color;
      const R = meta.depth_to_color.rotation;
      const T = meta.depth_to_color.translation;

      // Contar puntos válidos
      let validCount = 0;
      for (let i = 0; i < depth.length; i++) {
        const z = depth[i] * scale;
        if (z > 0 && z >= near && z <= far) validCount++;
      }

      if (validCount === 0) {
        showToast('No hay puntos válidos dentro del rango', true);
        return;
      }

      // Estructura PLY binaria: cada vértice tiene (x: f32, y: f32, z: f32, r: u8, g: u8, b: u8) = 15 bytes
      const headerText =
        `ply\nformat binary_little_endian 1.0\nelement vertex ${validCount}\n` +
        `property float x\nproperty float y\nproperty float z\n` +
        `property uchar red\nproperty uchar green\nproperty uchar blue\nend_header\n`;
      const headerBytes = new TextEncoder().encode(headerText);

      const buffer = new ArrayBuffer(headerBytes.length + validCount * 15);
      new Uint8Array(buffer, 0, headerBytes.length).set(headerBytes);
      const view = new DataView(buffer, headerBytes.length);

      let offset = 0;
      for (let y = 0; y < h; y++) {
        for (let x = 0; x < w; x++) {
          const idx = y * w + x;
          const z = depth[idx] * scale;
          if (!(z > 0 && z >= near && z <= far)) continue;

          // RealSense coordenadas (metros): x der, y abajo, z delante
          const rsX = (x - ppx) / fx * z;
          const rsY = (y - ppy) / fy * z;
          const rsZ = z;

          // Conversión a Blender (Z arriba): (x, z, -y)
          const bx = rsX;
          const by = rsZ;
          const bz = -rsY;

          // Proyección a color si existe
          let r = 200, g = 200, bVal = 200;
          if (colorData && c) {
            const pcX = R[0] * rsX + R[1] * rsY + R[2] * rsZ + T[0];
            const pcY = R[3] * rsX + R[4] * rsY + R[5] * rsZ + T[1];
            const pcZ = R[6] * rsX + R[7] * rsY + R[8] * rsZ + T[2];
            if (pcZ > 1e-4) {
              const u = Math.round((pcX / pcZ) * c.fx + c.ppx);
              const v = Math.round((pcY / pcZ) * c.fy + c.ppy);
              if (u >= 0 && u < c.width && v >= 0 && v < c.height) {
                const cIdx = (v * c.width + u) * 4;
                r = colorData[cIdx];
                g = colorData[cIdx + 1];
                bVal = colorData[cIdx + 2];
              }
            }
          }

          view.setFloat32(offset, bx, true);
          view.setFloat32(offset + 4, by, true);
          view.setFloat32(offset + 8, bz, true);
          view.setUint8(offset + 12, r);
          view.setUint8(offset + 13, g);
          view.setUint8(offset + 14, bVal);
          offset += 15;
        }
      }

      const blob = new Blob([buffer], { type: 'application/octet-stream' });
      const url = URL.createObjectURL(blob);
      const a = document.createElement('a');
      a.href = url;
      a.download = `realsense_scan_${Date.now()}.ply`;
      a.click();
      URL.revokeObjectURL(url);
      showToast(`PLY exportado con ${validCount.toLocaleString()} puntos`);
    } catch (err) {
      console.error(err);
      showToast('Error al exportar PLY: ' + err.message, true);
    }
  }, 30);
}

// Pantalla completa
const btnFull = document.getElementById('btn-full');
btnFull.addEventListener('click', () => {
  if (!document.fullscreenElement) {
    document.documentElement.requestFullscreen().catch(() => {});
  } else {
    document.exitFullscreen().catch(() => {});
  }
});

// Panel lateral Toggle
const panelToggle = document.getElementById('panel-toggle');
panelToggle.addEventListener('click', () => {
  appEl.classList.toggle('panel-hidden');
  setTimeout(() => viewer._resize(), 360);
});

// ---------------------------------------------------------------------------
// Atajos de Teclado
// ---------------------------------------------------------------------------
window.addEventListener('keydown', (e) => {
  if (['INPUT', 'SELECT', 'TEXTAREA'].includes(e.target.tagName)) return;

  if (e.code === 'Space') {
    e.preventDefault();
    btnFreeze.click();
  } else if (e.code === 'KeyM') {
    setGeometryMode(viewer.mode === 'points' ? 'mesh' : 'points');
  } else if (e.code === 'Digit1') {
    setColorMode('rgb');
  } else if (e.code === 'Digit2') {
    setColorMode('depth');
  } else if (e.code === 'Digit3') {
    setColorMode('normals');
  } else if (e.code === 'Digit4') {
    setColorMode('clay');
  } else if (e.code === 'KeyH') {
    panelToggle.click();
  } else if (e.code === 'KeyF') {
    btnFull.click();
  } else if (e.code === 'KeyC') {
    triggerSaveScan();
  } else if (e.code === 'KeyR') {
    viewer.resetView('front');
  }
});

// ---------------------------------------------------------------------------
// Módulo de Escaneo Unificado (Multi-Scan y Saves)
// ---------------------------------------------------------------------------

// Captura desde el botón del panel lateral
const btnPanelCapture = document.getElementById('btn-panel-capture');
if (btnPanelCapture) {
  btnPanelCapture.addEventListener('click', triggerSaveScan);
}

// Recargar Cámara
const btnReloadCam = document.getElementById('btn-reload-cam');
if (btnReloadCam) {
  btnReloadCam.addEventListener('click', async () => {
    btnReloadCam.disabled = true;
    showToast('Reiniciando pipeline de cámara…');
    try {
      const res = await fetch('/api/reload_camera', { method: 'POST' });
      const data = await res.json();
      showToast(data.message || 'Cámara reiniciada');
    } catch (err) {
      showToast(`Error al reiniciar: ${err.message}`, true);
    } finally {
      setTimeout(() => { btnReloadCam.disabled = false; }, 2000);
    }
  });
}

// ---------------------------------------------------------------------------
// Controles de Modo Dual RealSense
// ---------------------------------------------------------------------------
const dualControls = document.getElementById('dual-cam-controls');
const segDualCam = document.getElementById('seg-dual-cam');
const btnDualCapture = document.getElementById('btn-dual-capture');
const btnDualCalibrate = document.getElementById('btn-dual-calibrate');

// Controles de Calibración Interactiva y Fusión
const rngTx = document.getElementById('rng-tx');
const rngTy = document.getElementById('rng-ty');
const rngTz = document.getElementById('rng-tz');
const rngYaw = document.getElementById('rng-yaw');
const rngPitch = document.getElementById('rng-pitch');
const rngRoll = document.getElementById('rng-roll');

const valTx = document.getElementById('val-tx');
const valTy = document.getElementById('val-ty');
const valTz = document.getElementById('val-tz');
const valYaw = document.getElementById('val-yaw');
const valPitch = document.getElementById('val-pitch');
const valRoll = document.getElementById('val-roll');

const btnAutoAlign = document.getElementById('btn-auto-align');
const btnResetExtrinsics = document.getElementById('btn-reset-extrinsics');
const btnSaveExtrinsics = document.getElementById('btn-save-extrinsics');
const calibStatusBadge = document.getElementById('calib-status-badge');

// Convención en el marco RealSense (x der, y abajo, z delante): R = Ry(yaw) · Rx(pitch) · Rz(roll)
// Debe coincidir con live3d/registration.py::euler_translation_to_matrix
function eulerTranslationToMatrix(tx_cm, ty_cm, tz_cm, yaw_deg, pitch_deg, roll_deg) {
  const d = Math.PI / 180.0;
  const cy = Math.cos(yaw_deg * d), sy = Math.sin(yaw_deg * d);
  const cp = Math.cos(pitch_deg * d), sp = Math.sin(pitch_deg * d);
  const cr = Math.cos(roll_deg * d), sr = Math.sin(roll_deg * d);
  return [
    [cy * cr + sy * sp * sr, -cy * sr + sy * sp * cr, sy * cp, tx_cm / 100.0],
    [cp * sr,                cp * cr,                 -sp,     ty_cm / 100.0],
    [-sy * cr + cy * sp * sr, sy * sr + cy * sp * cr, cy * cp, tz_cm / 100.0],
    [0.0, 0.0, 0.0, 1.0]
  ];
}

function getCurrentExtrinsicsFromSliders() {
  if (!rngTx) return null;
  const tx = parseFloat(rngTx.value);
  const ty = parseFloat(rngTy.value);
  const tz = parseFloat(rngTz.value);
  const yaw = parseFloat(rngYaw.value);
  const pitch = parseFloat(rngPitch.value);
  const roll = parseFloat(rngRoll.value);
  return eulerTranslationToMatrix(tx, ty, tz, yaw, pitch, roll);
}

function updateSlidersFromEuler(euler) {
  if (!euler || !rngTx) return;
  const t = euler.translation_cm || {};
  const r = euler.rotation_deg || {};
  if (t.x !== undefined) { rngTx.value = t.x; valTx.textContent = Number(t.x).toFixed(1); }
  if (t.y !== undefined) { rngTy.value = t.y; valTy.textContent = Number(t.y).toFixed(1); }
  if (t.z !== undefined) { rngTz.value = t.z; valTz.textContent = Number(t.z).toFixed(1); }
  if (r.yaw !== undefined) { rngYaw.value = r.yaw; valYaw.textContent = Number(r.yaw).toFixed(1); }
  if (r.pitch !== undefined) { rngPitch.value = r.pitch; valPitch.textContent = Number(r.pitch).toFixed(1); }
  if (r.roll !== undefined) { rngRoll.value = r.roll; valRoll.textContent = Number(r.roll).toFixed(1); }
}

function onSliderInput() {
  if (valTx) valTx.textContent = Number(rngTx.value).toFixed(1);
  if (valTy) valTy.textContent = Number(rngTy.value).toFixed(1);
  if (valTz) valTz.textContent = Number(rngTz.value).toFixed(1);
  if (valYaw) valYaw.textContent = Number(rngYaw.value).toFixed(1);
  if (valPitch) valPitch.textContent = Number(rngPitch.value).toFixed(1);
  if (valRoll) valRoll.textContent = Number(rngRoll.value).toFixed(1);

  const m4x4 = getCurrentExtrinsicsFromSliders();
  if (m4x4) {
    viewer.setExtrinsics(m4x4);
  }
  if (calibStatusBadge) calibStatusBadge.textContent = 'En vivo';
}

[rngTx, rngTy, rngTz, rngYaw, rngPitch, rngRoll].forEach(s => {
  if (s) s.addEventListener('input', onSliderInput);
});

const tgDedup = document.getElementById('tg-dedup');
if (tgDedup) {
  viewer.setDedup(tgDedup.checked);
  tgDedup.addEventListener('change', () => viewer.setDedup(tgDedup.checked));
}

async function checkDualCameraStatus() {
  try {
    const res = await fetch('/api/dual/status');
    const data = await res.json();
    if (data.is_dual && dualControls) {
      dualControls.style.display = 'block';
      if (data.extrinsics) {
        viewer.setExtrinsics(data.extrinsics);
      }
      const btnCam1 = segDualCam?.querySelector('[data-view="cam1"]');
      const btnCam2 = segDualCam?.querySelector('[data-view="cam2"]');
      if (btnCam1 && data.cameras?.[0]) {
        btnCam1.textContent = `Cam 1 (${data.cameras[0].slice(-4)})`;
      }
      if (btnCam2 && data.cameras?.[1]) {
        btnCam2.textContent = `Cam 2 (${data.cameras[1].slice(-4)})`;
      }
      const activeView = data.view || 'dual';
      if (segDualCam) {
        segDualCam.querySelectorAll('button').forEach(b => {
          b.classList.toggle('on', b.dataset.view === activeView);
        });
      }
      viewer.setView(activeView);

      // Cargar extrínsecos descompuestos para los sliders interactivos
      try {
        const extRes = await fetch('/api/dual/extrinsics');
        const extData = await extRes.json();
        const comp = extData.components || extData.euler_translation;
        if (comp) {
          updateSlidersFromEuler(comp);
        }
      } catch (extErr) {
        console.warn('Error cargando extrínsecos detallados:', extErr);
      }

      // Asegurar que el modo sensor único sin duplicados esté siempre encendido en modo dual
      if (tgDedup) {
        tgDedup.checked = true;
        viewer.setDedup(true);
      }
    }
  } catch (e) {
    console.warn('Error comprobando estado dual:', e);
  }
}

if (btnAutoAlign) {
  btnAutoAlign.addEventListener('click', async () => {
    btnAutoAlign.disabled = true;
    showToast('Alineando cámaras con ICP multiescala...');
    if (calibStatusBadge) calibStatusBadge.textContent = 'Calculando...';
    try {
      const res = await fetch('/api/dual/auto_align', {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify({ use_colored_icp: true })
      });
      const data = await res.json();
      if (!res.ok || data.status === 'error') throw new Error(data.message);
      if (data.status === 'warning') {
        const why = (data.warnings || []).slice(0, 2).join(' ');
        throw new Error(`${data.message}${why ? ' ' + why : ''}`);
      }

      const comp = data.components || data.euler_translation;
      if (comp) {
        updateSlidersFromEuler(comp);
      }
      if (data.matrix_4x4) {
        viewer.setExtrinsics(data.matrix_4x4);
      }
      if (calibStatusBadge) {
        calibStatusBadge.textContent = `${data.quality || ''} ${Math.round(data.fitness * 100)}%`;
      }
      showToast(`✓ ${data.message}`);

      // Cambiar temporalmente a modo Solape para que el usuario aprecie el alineamiento
      const btnOverlap = document.querySelector('#seg-color button[data-v="overlap"]');
      if (btnOverlap) {
        document.querySelectorAll('#seg-color button').forEach(b => b.classList.remove('on'));
        btnOverlap.classList.add('on');
        viewer.setColorMode('overlap');
      }
    } catch (err) {
      showToast(`Auto-alineación: ${err.message}`, true);
      if (calibStatusBadge) calibStatusBadge.textContent = 'Error';
    } finally {
      btnAutoAlign.disabled = false;
    }
  });
}

if (btnSaveExtrinsics) {
  btnSaveExtrinsics.addEventListener('click', async () => {
    btnSaveExtrinsics.disabled = true;
    try {
      const m4x4 = getCurrentExtrinsicsFromSliders();
      const res = await fetch('/api/dual/extrinsics', {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify({ matrix_4x4: m4x4 })
      });
      const data = await res.json();
      if (!res.ok || data.status === 'error') throw new Error(data.message);
      if (calibStatusBadge) calibStatusBadge.textContent = 'Guardado';
      showToast('✓ Calibración guardada exitosamente');
    } catch (err) {
      showToast(`Error al guardar: ${err.message}`, true);
    } finally {
      btnSaveExtrinsics.disabled = false;
    }
  });
}

if (btnResetExtrinsics) {
  btnResetExtrinsics.addEventListener('click', async () => {
    try {
      const res = await fetch('/api/dual/extrinsics');
      const data = await res.json();
      if (data.components) updateSlidersFromEuler(data.components);
      if (data.extrinsics) viewer.setExtrinsics(data.extrinsics);
      if (calibStatusBadge) calibStatusBadge.textContent = 'Guardada';
      showToast('Calibración guardada restablecida');
    } catch (err) {
      showToast(`Error al restablecer: ${err.message}`, true);
    }
  });
}

if (segDualCam) {
  segDualCam.querySelectorAll('button').forEach((btn) => {
    btn.addEventListener('click', async () => {
      const view = btn.dataset.view; // 'dual' | 'cam1' | 'cam2'
      segDualCam.querySelectorAll('button').forEach(b => b.classList.remove('on'));
      btn.classList.add('on');
      viewer.setView(view);

      try {
        if (view === 'dual') {
          await fetch('/api/dual/select', {
            method: 'POST',
            headers: { 'Content-Type': 'application/json' },
            body: JSON.stringify({ mode: 'dual' })
          });
          showToast('Modo Dual 3D en vivo activado (ambas cámaras)');
        } else {
          const camIdx = view === 'cam1' ? 0 : 1;
          await fetch('/api/dual/select', {
            method: 'POST',
            headers: { 'Content-Type': 'application/json' },
            body: JSON.stringify({ mode: 'solo', index: camIdx })
          });
          showToast(`Mostrando señal de Cámara ${camIdx + 1}`);
        }
      } catch (err) {
        showToast(`Error seleccionando vista: ${err.message}`, true);
      }
    });
  });
}

if (btnDualCapture) {
  btnDualCapture.addEventListener('click', async () => {
    btnDualCapture.disabled = true;
    showToast('Capturando de ambas cámaras y fusionando con Open3D…');
    try {
      const res = await fetch('/api/dual/capture', { method: 'POST' });
      const data = await res.json();
      if (!res.ok || data.status === 'error') throw new Error(data.message);
      showToast(`✓ ${data.message} (${data.points_fused || 0} puntos)`);
    } catch (err) {
      showToast(`Error en captura dual: ${err.message}`, true);
    } finally {
      btnDualCapture.disabled = false;
    }
  });
}

if (btnDualCalibrate) {
  btnDualCalibrate.addEventListener('click', async () => {
    btnDualCalibrate.disabled = true;
    showToast('Buscando tablero ChArUco en ambas cámaras…');
    try {
      const res = await fetch('/api/dual/calibrate', { method: 'POST' });
      const data = await res.json();
      if (!res.ok || data.status === 'error') throw new Error(data.message);
      if (data.matrix_4x4) {
        viewer.setExtrinsics(data.matrix_4x4);
      }
      showToast(`✓ ${data.message}`);
    } catch (err) {
      showToast(`Calibración: ${err.message}`, true);
    } finally {
      btnDualCalibrate.disabled = false;
    }
  });
}

checkDualCameraStatus();

// Multi-Scan Controller
const multiIdleControls = document.getElementById('multi-idle-controls');
const multiActiveControls = document.getElementById('multi-active-controls');
const multiCount = document.getElementById('multi-count');
const multiHint = document.getElementById('multi-hint');
const multiQualityPill = document.getElementById('multi-quality-pill');
const multiThumbs = document.getElementById('multi-thumbs');
const btnMultiStart = document.getElementById('btn-multi-start');
const btnMultiAdd = document.getElementById('btn-multi-add');
const btnMultiUndo = document.getElementById('btn-multi-undo');
const btnMultiFinish = document.getElementById('btn-multi-finish');
const btnMultiDiscard = document.getElementById('btn-multi-discard');
const multiProgressBox = document.getElementById('multi-progress-box');
const multiProgressBar = document.getElementById('multi-progress-bar');
const multiProgressLabel = document.getElementById('multi-progress-label');

let multiPollTimer = null;

function setMultiQuality(quality) {
  if (!multiQualityPill) return;
  multiQualityPill.className = `pill-quality quality-${quality || 'buena'}`;
  multiQualityPill.textContent = quality || '–';
}

function updateMultiScanView(session) {
  if (!session || !session.state || session.state === 'idle' || session.state === 'discarded') {
    if (multiIdleControls) multiIdleControls.style.display = 'block';
    if (multiActiveControls) multiActiveControls.style.display = 'none';
    if (multiCount) multiCount.textContent = 'Inactivo';
    if (multiThumbs) multiThumbs.innerHTML = '';
    if (multiProgressBox) multiProgressBox.style.display = 'none';
    return;
  }

  // Activo
  if (multiIdleControls) multiIdleControls.style.display = 'none';
  if (multiActiveControls) multiActiveControls.style.display = 'block';

  const nViews = (session.views || []).length;
  if (multiCount) multiCount.textContent = `${nViews} vista${nViews === 1 ? '' : 's'}`;

  // Miniaturas
  if (multiThumbs && session.views) {
    multiThumbs.innerHTML = session.views.map((v, i) => `
      <div class="multi-thumb-item" title="Vista ${i + 1} (${v.quality})">
        ${v.thumb ? `<img src="${v.thumb}" alt="V${i + 1}">` : ''}
        <span class="thumb-idx">#${i + 1}</span>
      </div>
    `).join('');
    multiThumbs.scrollLeft = multiThumbs.scrollWidth;
  }

  // Última calidad
  if (session.views && session.views.length > 0) {
    const lastView = session.views[session.views.length - 1];
    setMultiQuality(lastView.quality);
  } else {
    setMultiQuality('referencia');
  }

  // Progreso si está finalizando
  if (session.state === 'finishing') {
    if (multiProgressBox) multiProgressBox.style.display = 'block';
    if (multiProgressBar) multiProgressBar.style.width = '70%';
    if (multiProgressLabel) multiProgressLabel.textContent = 'Fusionando nubes y generando malla TSDF...';
    if (btnMultiAdd) btnMultiAdd.disabled = true;
    if (btnMultiFinish) btnMultiFinish.disabled = true;
  } else if (session.state === 'finished') {
    if (multiProgressBox) multiProgressBox.style.display = 'none';
    showToast(`✓ ¡Mapeo completado! Guardado en Saves/${session.id || ''}`);
    stopMultiPolling();
    // Volver a estado inactivo tras 2 segundos
    setTimeout(() => {
      checkMultiStatus();
    }, 2000);
  }
}

async function checkMultiStatus() {
  try {
    const res = await fetch('/api/multi/status');
    const data = await res.json();
    if (data.active && data.session) {
      updateMultiScanView(data.session);
    } else {
      updateMultiScanView(null);
    }
  } catch (err) {
    console.warn('Error consultando estado multi-scan:', err);
  }
}

function startMultiPolling() {
  stopMultiPolling();
  multiPollTimer = setInterval(checkMultiStatus, 1500);
}

function stopMultiPolling() {
  if (multiPollTimer) {
    clearInterval(multiPollTimer);
    multiPollTimer = null;
  }
}

if (btnMultiStart) {
  btnMultiStart.addEventListener('click', async () => {
    btnMultiStart.disabled = true;
    showToast('Iniciando sesión de Mapeo 360°…');
    try {
      const res = await fetch('/api/multi/start', { method: 'POST' });
      const data = await res.json();
      if (!res.ok || data.status === 'error') throw new Error(data.message);
      showToast('✓ Sesión iniciada. Añade la primera foto de referencia.');
      await checkMultiStatus();
      startMultiPolling();
    } catch (err) {
      showToast(`Error: ${err.message}`, true);
    } finally {
      btnMultiStart.disabled = false;
    }
  });
}

if (btnMultiAdd) {
  btnMultiAdd.addEventListener('click', async () => {
    btnMultiAdd.disabled = true;
    showToast('Capturando y registrando vista…');
    try {
      const res = await fetch('/api/multi/add', { method: 'POST' });
      const data = await res.json();
      if (!res.ok || data.status === 'error') throw new Error(data.message);
      const q = data.view ? data.view.quality : 'OK';
      showToast(`✓ Vista añadida (Calidad: ${q})`);
      await checkMultiStatus();
    } catch (err) {
      showToast(`Error al añadir: ${err.message}`, true);
    } finally {
      btnMultiAdd.disabled = false;
    }
  });
}

if (btnMultiUndo) {
  btnMultiUndo.addEventListener('click', async () => {
    btnMultiUndo.disabled = true;
    try {
      const res = await fetch('/api/multi/undo', { method: 'POST' });
      const data = await res.json();
      if (!res.ok || data.status === 'error') throw new Error(data.message);
      showToast('Última vista deshecha');
      await checkMultiStatus();
    } catch (err) {
      showToast(`Error: ${err.message}`, true);
    } finally {
      btnMultiUndo.disabled = false;
    }
  });
}

if (btnMultiFinish) {
  btnMultiFinish.addEventListener('click', async () => {
    btnMultiFinish.disabled = true;
    if (multiProgressBox) multiProgressBox.style.display = 'block';
    if (multiProgressBar) multiProgressBar.style.width = '35%';
    if (multiProgressLabel) multiProgressLabel.textContent = 'Iniciando optimización del grafo de poses...';
    showToast('Procesando mapeo 3D en segundo plano…');
    try {
      const res = await fetch('/api/multi/finish', { method: 'POST' });
      const data = await res.json();
      if (!res.ok || data.status === 'error') throw new Error(data.message);
      startMultiPolling();
    } catch (err) {
      showToast(`Error: ${err.message}`, true);
      btnMultiFinish.disabled = false;
    }
  });
}

if (btnMultiDiscard) {
  btnMultiDiscard.addEventListener('click', async () => {
    if (!confirm('¿Seguro que deseas descartar esta sesión de mapeo?')) return;
    try {
      await fetch('/api/multi/discard', { method: 'POST' });
      stopMultiPolling();
      showToast('Sesión de mapeo descartada');
      updateMultiScanView(null);
    } catch (err) {
      showToast(`Error: ${err.message}`, true);
    }
  });
}

// Historial de Saves Modal
const modalSaves = document.getElementById('modal-saves');
const btnOpenSaves = document.getElementById('btn-open-saves');
const btnCloseSaves = document.getElementById('btn-close-saves');
const savesGrid = document.getElementById('saves-grid');
const savesLoading = document.getElementById('saves-loading');
const savesEmpty = document.getElementById('saves-empty');

async function loadSavesList() {
  if (!savesGrid) return;
  savesLoading.style.display = 'block';
  savesEmpty.style.display = 'none';
  savesGrid.innerHTML = '';

  try {
    const res = await fetch('/api/saves');
    const data = await res.json();
    savesLoading.style.display = 'none';

    if (!data.saves || data.saves.length === 0) {
      savesEmpty.style.display = 'block';
      return;
    }

    savesGrid.innerHTML = data.saves.map(s => {
      const plyFile = s.files.find(f => f.endsWith('.ply'));
      const objFile = s.files.find(f => f.endsWith('.obj'));
      const downloadLink = plyFile || objFile;

      return `
        <div class="save-item">
          ${s.thumb
            ? `<img class="save-thumb" src="${s.thumb}" alt="${s.name}" loading="lazy">`
            : `<div class="save-thumb-placeholder">Sin miniatura</div>`
          }
          <div class="save-info">
            <div class="save-title">${s.is_multi ? '🔮 ' : '📷 '}${s.name}</div>
            <div class="save-meta">📅 ${s.date} · ${s.is_multi ? 'Mapeo Multi-Scan' : 'Captura 3D'}</div>
            <div class="save-badges">
              ${s.files.map(f => `<span class="save-badge">${f.split('.').pop().toUpperCase()}</span>`).join('')}
            </div>
            <div class="save-actions">
              ${downloadLink ? `<a href="/saves/${s.id}/${downloadLink}" download class="save-btn" title="Descargar modelo 3D">Descargar</a>` : ''}
              <button class="save-btn btn-copy-path" data-path="${s.full_path.replace(/\\/g, '/')}" title="Copiar ruta absoluta para Blender">Copiar Ruta</button>
            </div>
          </div>
        </div>
      `;
    }).join('');

    // Eventos para copiar ruta
    savesGrid.querySelectorAll('.btn-copy-path').forEach(btn => {
      btn.addEventListener('click', () => {
        const p = btn.getAttribute('data-path');
        navigator.clipboard.writeText(p).then(() => {
          showToast('✓ Ruta copiada al portapapeles para Blender');
        }).catch(() => {
          showToast(`Ruta: ${p}`);
        });
      });
    });

  } catch (err) {
    savesLoading.style.display = 'none';
    showToast(`Error cargando Saves: ${err.message}`, true);
  }
}

if (btnOpenSaves && modalSaves) {
  btnOpenSaves.addEventListener('click', () => {
    modalSaves.classList.add('open');
    modalSaves.setAttribute('aria-hidden', 'false');
    loadSavesList();
  });
}

if (btnCloseSaves && modalSaves) {
  btnCloseSaves.addEventListener('click', () => {
    modalSaves.classList.remove('open');
    modalSaves.setAttribute('aria-hidden', 'true');
  });
}

if (modalSaves) {
  modalSaves.addEventListener('click', (e) => {
    if (e.target === modalSaves) {
      modalSaves.classList.remove('open');
      modalSaves.setAttribute('aria-hidden', 'true');
    }
  });
}

// Comprobar estado de MultiScan al cargar la página
checkMultiStatus();

// ---------------------------------------------------------------------------
// Conexión inicial del stream
// ---------------------------------------------------------------------------
stream.connect();

