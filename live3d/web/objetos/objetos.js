// Escáner de objetos: un paso a la vez (objeto · tomas · procesar · resultado) + biblioteca.
import { RSMViewer } from './rsmviewer.js';

const $ = (id) => document.getElementById(id);
const api = async (path, opts = {}) => {
  const r = await fetch(path, { headers: { 'Content-Type': 'application/json' }, ...opts,
                                 body: opts.body ? JSON.stringify(opts.body) : undefined });
  const data = await r.json().catch(() => ({}));
  if (!r.ok) throw new Error(data.error || `HTTP ${r.status}`);
  return data;
};
const esc = (t) => String(t ?? '').replace(/[&<>"']/g, (c) => ({ '&': '&amp;', '<': '&lt;', '>': '&gt;', '"': '&quot;', "'": '&#39;' }[c]));

let session = null;        // estado de la sesión (servidor)
let step = 1;
let framing = false;       // bucle de encuadre activo
let busy = false;
let viewer = null;
let resultShown = null;

// ---------------- avisos ----------------
let toastTimer = null;
function toast(msg, err = false) {
  const t = $('toast');
  t.textContent = msg;
  t.className = 'toast' + (err ? ' err' : '');
  clearTimeout(toastTimer);
  toastTimer = setTimeout(() => t.classList.add('hidden'), err ? 7000 : 4000);
}

// ---------------- pestañas ----------------
document.querySelectorAll('.tab').forEach((b) => b.addEventListener('click', () => {
  document.querySelectorAll('.tab').forEach((x) => x.classList.toggle('on', x === b));
  const lib = b.dataset.tab === 'biblioteca';
  $('view-escanear').classList.toggle('hidden', lib);
  $('view-biblioteca').classList.toggle('hidden', !lib);
  if (lib) loadLibrary();
  updateFraming();
}));

// ---------------- pasos ----------------
function setStep(n) {
  step = n;
  document.querySelectorAll('.step').forEach((s) => {
    const k = +s.dataset.step;
    s.classList.toggle('active', k === n);
    s.classList.toggle('done', k < n);
  });
  const showViewer = n === 4;
  $('visor3d').classList.toggle('hidden', !showViewer);
  $('encuadre').classList.toggle('hidden', showViewer);
  if (showViewer) { $('chips').innerHTML = ''; $('warnings').innerHTML = ''; } else if (viewer) viewer.stop();
  updateFraming();
}

function applySession(s) {
  session = s && s.activa ? s : null;
  if (!session) { setStep(1); $('s2-state').textContent = ''; return; }
  $('s1-state').textContent = `${session.nombre} · ${session.usar_base ? 'hoja ' + session.hoja : 'sin base'}`;
  $('s2-state').textContent = `${session.tomas} toma${session.tomas === 1 ? '' : 's'}`;
  renderThumbs(session.vistas || []);
  $('btn-procesar').disabled = session.tomas < 2;
  $('btn-deshacer').disabled = session.tomas < 1;
  if (session.state === 'processing') {
    setStep(3);
    $('bar').style.width = `${Math.round(session.progress * 100)}%`;
    $('etapa').textContent = session.stage || '…';
    $('s3-state').textContent = `${Math.round(session.progress * 100)} %`;
  } else if (session.state === 'done' && session.result) {
    $('s3-state').textContent = 'listo';
    setStep(4);
    showResult(session.result);
  } else {
    setStep(2);
    if (session.error) toast(session.error, true);
  }
}

function renderThumbs(views) {
  const box = $('thumbs');
  box.innerHTML = '';
  for (const v of views) {
    const q = v.calidad || {};
    const fit = q.fitness;
    const cls = fit == null ? 'ok' : fit >= 0.6 ? 'ok' : 'mid';
    const label = fit == null ? 'ref.' : `${Math.round(fit * 100)}%`;
    const src = { base: 'base', 'calibración': 'calib', icp: 'icp', referencia: 'ref' }[v.origen_pose] || '';
    const d = document.createElement('div');
    d.className = 'th';
    d.title = `${v.etiqueta} · pose por ${v.origen_pose}${v.base?.markers?.length ? ` (${v.base.markers.length} marcadores)` : ''}`;
    d.innerHTML = `<img src="${v.thumb || ''}" alt=""><span class="${cls}">${label}</span><em>${src}${v.camara === 'cam2' ? ' · C2' : ''}</em>`;
    box.appendChild(d);
  }
}

// ---------------- encuadre (vista en vivo con el objeto resaltado) ----------------
function updateFraming() {
  const want = !$('view-escanear').classList.contains('hidden') && (step === 1 || step === 2) && !document.hidden;
  if (want && !framing) { framing = true; framingLoop(); }
  framing = want;
}
document.addEventListener('visibilitychange', updateFraming);

async function framingLoop() {
  while (framing) {
    const t0 = performance.now();
    try {
      const q = session ? '' : `?hoja=${$('hoja').value}&radio_cm=${$('radio').value}&altura_cm=${$('altura').value}`;
      const r = await fetch('/api/objscan/encuadre' + q);
      const d = await r.json();
      if (!r.ok) throw new Error(d.error || r.status);
      if (!framing) break;
      $('encuadre').src = d.imagen;
      $('stage-empty').classList.add('hidden');
      renderChips(d);
    } catch (e) {
      $('stage-empty').textContent = `Sin imagen de la cámara: ${e.message}`;
      $('stage-empty').classList.remove('hidden');
      await new Promise((r) => setTimeout(r, 1500));
    }
    const dt = performance.now() - t0;
    if (dt < 300) await new Promise((r) => setTimeout(r, 300 - dt));
  }
}

function chip(label, value, ok) {
  return `<span class="chip ${ok === true ? 'ok' : ok === false ? 'bad' : ''}">${esc(label)} <b>${esc(value)}</b></span>`;
}

function renderChips(d) {
  const c = [];
  if (d.base) c.push(chip('Base', d.base.ok ? `${d.base.markers.length} marcadores` : 'no visible', d.base.ok));
  else c.push(chip('Base', 'sin base', null));
  if (d.extent) c.push(chip('Objeto', `${d.extent.ancho_cm}×${d.extent.fondo_cm}×${d.extent.alto_cm} cm`, d.ok));
  else c.push(chip('Objeto', 'no detectado', false));
  if (d.distance_m) c.push(chip('Distancia', `${d.distance_m} m`, d.distance_m >= 0.55 && d.distance_m <= 1.1));
  if (d.base?.scale_ratio && Math.abs(d.base.scale_ratio - 1) > 0.015) c.push(chip('Escala hoja', `${(d.base.scale_ratio * 100).toFixed(1)} %`, false));
  if (d.simulada) c.push(chip('Cámara', 'simulada', null));
  $('chips').innerHTML = c.join('');
  $('warnings').innerHTML = (d.warnings || []).map((w) => `<li>${esc(w)}</li>`).join('');
}

// ---------------- paso 1: objeto ----------------
const syncLabels = () => {
  $('v-radio').textContent = `${$('radio').value} cm`;
  $('v-altura').textContent = `${$('altura').value} cm`;
};
let volTimer = null;
['radio', 'altura'].forEach((id) => $(id).addEventListener('input', () => {
  syncLabels();
  if (session) {
    clearTimeout(volTimer);
    volTimer = setTimeout(() => api('/api/objscan/volumen', { method: 'POST', body: { radio_cm: +$('radio').value, altura_cm: +$('altura').value } }).catch((e) => toast(e.message, true)), 250);
  }
}));
syncLabels();

$('btn-empezar').addEventListener('click', async () => {
  try {
    const s = await api('/api/objscan/nueva', { method: 'POST', body: {
      nombre: $('nombre').value.trim() || 'Objeto', hoja: $('hoja').value,
      radio_cm: +$('radio').value, altura_cm: +$('altura').value } });
    applySession(s);
  } catch (e) { toast(e.message, true); }
});

// ---------------- paso 2: tomas ----------------
async function takeShot() {
  if (busy || step !== 2) return;
  busy = true;
  $('btn-tomar').disabled = true;
  try {
    const r = await api('/api/objscan/tomar', { method: 'POST' });
    $('consejo').textContent = r.toma.consejo;
    if (r.toma.descartadas?.length) toast(`Descartado: ${r.toma.descartadas.join(' · ')}`, true);
    applySession({ activa: true, ...r.sesion });
  } catch (e) { toast(e.message, true); }
  finally { busy = false; $('btn-tomar').disabled = false; }
}
$('btn-tomar').addEventListener('click', takeShot);
document.addEventListener('keydown', (e) => {
  if (e.code === 'Space' && step === 2 && e.target.tagName !== 'INPUT') { e.preventDefault(); takeShot(); }
});
$('btn-deshacer').addEventListener('click', async () => {
  try { applySession({ activa: true, ...(await api('/api/objscan/deshacer', { method: 'POST' })) }); }
  catch (e) { toast(e.message, true); }
});
$('btn-descartar').addEventListener('click', async () => {
  if (!confirm('¿Descartar todas las tomas de este objeto?')) return;
  await api('/api/objscan/descartar', { method: 'POST' }).catch(() => {});
  applySession(null);
});
$('btn-procesar').addEventListener('click', async () => {
  try { applySession({ activa: true, ...(await api('/api/objscan/procesar', { method: 'POST' })) }); pollProcessing(); }
  catch (e) { toast(e.message, true); }
});

async function pollProcessing() {
  while (true) {
    const s = await api('/api/objscan/estado').catch(() => null);
    if (!s) break;
    applySession(s);
    if (!s.activa || s.state !== 'processing') break;
    await new Promise((r) => setTimeout(r, 500));
  }
}

// ---------------- paso 4: resultado ----------------
async function showResult(man) {
  $('s4-state').textContent = man.publicado ? 'publicado' : man.estado;
  const t = man.tamano_m || [0, 0, 0];
  const poses = man.poses || {};
  $('metrics').innerHTML = [
    ['Tamaño', `${(t[0] * 100).toFixed(1)} × ${(t[2] * 100).toFixed(1)} × ${(t[1] * 100).toFixed(1)} cm`],
    ['Triángulos', (man.triangulos || 0).toLocaleString('es')],
    ['Tomas / vistas', `${man.tomas ?? '-'} / ${man.vistas ?? '-'}`],
    ['Poses por base', `${poses.base ?? 0} de ${man.vistas ?? '-'}`],
    ['Encaje medio', man.encaje_medio != null ? `${Math.round(man.encaje_medio * 100)} %` : '-'],
    ['Proceso', `${man.tiempo_proceso_s ?? '-'} s`],
  ].concat(calibRow(man)).map(([k, v]) => `<dt>${k}</dt><dd>${esc(v)}</dd>`).join('');
  const cal = man.correccion_calibracion_dual;
  $('btn-calib').classList.toggle('hidden', !(cal?.aplicada && (cal.mm > 5 || cal.deg > 0.5)) || !session?.result || session.result.id !== man.id);
  $('btn-publicar').textContent = man.publicado ? 'Retirar de Unity' : 'Publicar en Unity';
  $('btn-publicar').dataset.id = man.id;
  $('btn-publicar').dataset.pub = man.publicado ? '1' : '';
  $('btn-blender').dataset.id = man.id;
  if (resultShown !== `${man.id}:${man.version}`) {
    resultShown = `${man.id}:${man.version}`;
    loadLibrary();
    if (!viewer) viewer = new RSMViewer($('visor3d'));
    viewer.load(`/api/objetos/${man.id}/modelo.rsm?v=${man.version}`).catch((e) => toast(e.message, true));
  } else if (viewer) viewer.start();
}

function calibRow(man) {
  const c = man.correccion_calibracion_dual;
  if (!c) return [];
  if (!c.aplicada) return [['Calibración dual', `sin corregir (${c.muestras}/${c.tomas} tomas)`]];
  return [['Calibración dual', c.mm > 5 || c.deg > 0.5 ? `desajuste ${c.mm} mm · ${c.deg}° (corregido)` : 'correcta']];
}

$('btn-calib').addEventListener('click', async () => {
  if (!confirm('¿Guardar como calibración dual la corrección medida por este escaneo? (se puede deshacer en el estudio de calibración)')) return;
  try {
    const r = await api('/api/objscan/aplicar_calibracion', { method: 'POST' });
    toast(`Calibración dual actualizada (corrección de ${r.mm} mm · ${r.deg}°).`);
    $('btn-calib').classList.add('hidden');
  } catch (e) { toast(e.message, true); }
});

async function togglePublish(id, published) {
  const man = await api(`/api/objetos/${id}/publicar`, { method: 'POST', body: { publicado: !published } });
  toast(man.publicado ? 'Publicado: aparecerá en Unity en unos segundos.' : 'Retirado de Unity.');
  return man;
}
$('btn-publicar').addEventListener('click', async (e) => {
  try {
    const man = await togglePublish(e.target.dataset.id, !!e.target.dataset.pub);
    if (session?.result) { session.result = { ...session.result, ...man }; showResult(session.result); }
  } catch (err) { toast(err.message, true); }
});
async function openBlender(id) {
  try {
    await api(`/api/objetos/${id}/blender`, { method: 'POST' });
    toast('Abriendo Blender… Ajusta el objeto y usa «Publicar en Unity» en el panel RealSense.');
  } catch (e) { toast(e.message, true); }
}
$('btn-blender').addEventListener('click', (e) => openBlender(e.target.dataset.id));
$('btn-otro').addEventListener('click', async () => {
  await api('/api/objscan/descartar', { method: 'POST' }).catch(() => {});
  resultShown = null;
  $('nombre').value = '';
  applySession(null);
});

// ---------------- biblioteca ----------------
async function loadLibrary() {
  const { objetos } = await api('/api/objetos?todos=1').catch(() => ({ objetos: [] }));
  $('lib-count').textContent = objetos.length;
  const box = $('cards');
  box.innerHTML = '';
  for (const o of objetos) {
    const el = $('tpl-card').content.firstElementChild.cloneNode(true);
    el.querySelector('.thumb').src = `/api/objetos/${o.id}/miniatura.jpg?v=${o.version}`;
    el.querySelector('h3').textContent = o.nombre;
    const t = o.tamano_m || [0, 0, 0];
    el.querySelector('.meta').textContent = `${(t[0] * 100).toFixed(0)}×${(t[2] * 100).toFixed(0)}×${(t[1] * 100).toFixed(0)} cm · ${(o.triangulos || 0).toLocaleString('es')} tri`;
    el.querySelector('.badges').innerHTML =
      `<span class="badge ${o.estado === 'limpio' ? 'limpio' : ''}">${o.estado === 'limpio' ? 'ajustado en Blender' : 'del escáner'}</span>` +
      (o.publicado ? `<span class="badge pub">en Unity · v${o.version}</span>` : '');
    const pub = el.querySelector('[data-act=publicar]');
    pub.textContent = o.publicado ? 'Retirar' : 'Publicar';
    el.querySelector('[data-act=ver]').onclick = () => {
      document.querySelector('.tab[data-tab=escanear]').click();
      setStep(4);
      showResult(o);
    };
    el.querySelector('[data-act=blender]').onclick = () => openBlender(o.id);
    pub.onclick = async () => { try { await togglePublish(o.id, o.publicado); loadLibrary(); } catch (e) { toast(e.message, true); } };
    el.querySelector('[data-act=borrar]').onclick = async () => {
      if (!confirm(`¿Borrar «${o.nombre}» de la biblioteca? Esto no se puede deshacer.`)) return;
      await api(`/api/objetos/${o.id}`, { method: 'DELETE' }).catch((e) => toast(e.message, true));
      loadLibrary();
    };
    box.appendChild(el);
  }
}

// ---------------- estado de la cámara ----------------
async function cameraPill() {
  try {
    const st = await api('/api/status');
    const c = st.camera || {};
    const pill = $('cam-pill');
    let state = 'error', text = c.message || c.state;
    if (c.state === 'streaming') { state = c.simulated ? 'sim' : 'live'; text = c.simulated ? 'Cámara simulada' : (c.dual ? '2 cámaras' : 'Cámara en vivo'); }
    else if (c.state === 'degraded' || c.state === 'starting') state = 'warn';
    pill.dataset.state = state;
    pill.querySelector('span').textContent = text;
    pill.title = c.message || '';
  } catch { $('cam-pill').dataset.state = 'error'; }
}
setInterval(cameraPill, 3000);
cameraPill();

// ---------------- arranque ----------------
(async () => {
  applySession(await api('/api/objscan/estado').catch(() => null));
  if (session?.state === 'processing') pollProcessing();
  loadLibrary();
})();
