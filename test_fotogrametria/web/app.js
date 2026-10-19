// app.js - Cliente Web para Estudio de Capturas, Fusión 2D y Alineación Virtual en Tiempo Real

const workspace = document.getElementById("workspace");
const tabSplit = document.getElementById("tab-split");
const tab3D = document.getElementById("tab-3d");
const tabMatches = document.getElementById("tab-matches");

// Acciones principales
const btnTakeCapture = document.getElementById("btn-take-capture");
const btnAlign = document.getElementById("btn-align");
const statusBanner = document.getElementById("status-banner");
const captureFlash = document.getElementById("capture-flash");

// Visor de Cámaras
const vtabLive = document.getElementById("vtab-live");
const vtabSnapshot = document.getElementById("vtab-snapshot");
const vtabOverlay = document.getElementById("vtab-overlay");
const vtabAnaglyph = document.getElementById("vtab-anaglyph");
const vtabEpipolar = document.getElementById("vtab-epipolar");
const vtabMatches = document.getElementById("vtab-matches");

const visorImg = document.getElementById("visor-img");
const visorBadge = document.getElementById("visor-badge");
const visorLoader = document.getElementById("visor-loader");
const loaderText = document.getElementById("loader-text");

// Barra de Opacidad de Fusión 2D
const overlayOpacityBar = document.getElementById("overlay-opacity-bar");
const slOverlayAlpha = document.getElementById("sl-overlay-alpha");
const lblOverlayAlpha = document.getElementById("lbl-overlay-alpha");

// Galería de Capturas
const capturesCount = document.getElementById("captures-count");
const capturesStrip = document.getElementById("captures-strip");
const galleryEmpty = document.getElementById("gallery-empty");
const btnClearCaptures = document.getElementById("btn-clear-captures");

// Métricas
const mFitness = document.getElementById("m-fitness");
const mRmse = document.getElementById("m-rmse");
const mTime = document.getElementById("m-time");

// Sliders de Ajuste Fino
const slTx = document.getElementById("sl-tx");
const slTy = document.getElementById("sl-ty");
const slTz = document.getElementById("sl-tz");
const slYaw = document.getElementById("sl-yaw");
const slPitch = document.getElementById("sl-pitch");
const slRoll = document.getElementById("sl-roll");

const valTx = document.getElementById("val-tx");
const valTy = document.getElementById("val-ty");
const valTz = document.getElementById("val-tz");
const valYaw = document.getElementById("val-yaw");
const valPitch = document.getElementById("val-pitch");
const valRoll = document.getElementById("val-roll");
const btnResetPose = document.getElementById("btn-reset-pose");

// Estado local
let visorMode = "live"; // "live" | "snapshot" | "overlay" | "anaglyph" | "epipolar" | "matches"
let activeCaptureId = null;
let capturesList = [];
let isAligning = false;
let isCapturing = false;
let manualDebounce = null;
let alphaDebounce = null;

// ==========================================
// 1. SINCRONIZACIÓN EN TIEMPO REAL CON EL VISOR 3D (THREE.JS)
// ==========================================
function applyPoseToViewer3D(matrix_4x4, components) {
  const iframe = document.getElementById("iframe-3d");
  if (!iframe || !iframe.contentWindow) return;

  try {
    // Si Three.js ya inicializó viewer en la ventana del iframe
    if (iframe.contentWindow.viewer) {
      iframe.contentWindow.viewer.setExtrinsics(matrix_4x4);
    }
    if (typeof iframe.contentWindow.updateSlidersFromEuler === "function" && components) {
      iframe.contentWindow.updateSlidersFromEuler(components);
    }
  } catch (e) {
    // SOP fallback por si acaso
  }

  // Envío cruzado por postMessage
  try {
    iframe.contentWindow.postMessage({
      type: "SET_EXTRINSICS",
      matrix_4x4: matrix_4x4,
      components: components
    }, "*");
  } catch (e) {}
}

// ==========================================
// 2. SELECTOR DE VISTA GENERAL (SPLIT / 3D / 2D)
// ==========================================
function setWorkspaceMode(mode) {
  workspace.classList.remove("mode-3d", "mode-matches");
  tabSplit.classList.remove("active");
  tab3D.classList.remove("active");
  tabMatches.classList.remove("active");

  if (mode === "3d") {
    workspace.classList.add("mode-3d");
    tab3D.classList.add("active");
  } else if (mode === "matches") {
    workspace.classList.add("mode-matches");
    tabMatches.classList.add("active");
  } else {
    tabSplit.classList.add("active");
  }
}

tabSplit.addEventListener("click", () => setWorkspaceMode("split"));
tab3D.addEventListener("click", () => setWorkspaceMode("3d"));
tabMatches.addEventListener("click", () => setWorkspaceMode("matches"));

// ==========================================
// 3. MODOS DEL VISOR DE CÁMARAS
// ==========================================
function setVisorMode(mode) {
  visorMode = mode;
  [vtabLive, vtabSnapshot, vtabOverlay, vtabAnaglyph, vtabEpipolar, vtabMatches].forEach(b => {
    if (b) b.classList.remove("active");
  });
  visorBadge.className = "visor-badge";
  overlayOpacityBar.classList.add("hidden");

  const activeCap = capturesList.find(c => c.id === activeCaptureId);

  if (mode === "live") {
    vtabLive.classList.add("active");
    visorBadge.classList.add("live");
    visorBadge.textContent = "🔴 EN DIRECTO";
    visorImg.src = "/api/live_stream?t=" + Date.now();
  }
  else if (mode === "snapshot") {
    vtabSnapshot.classList.add("active");
    visorBadge.classList.add("snapshot");
    if (activeCap) {
      visorBadge.textContent = `📸 ${activeCap.title} · ${activeCap.timestamp}`;
      visorImg.src = activeCap.preview_url;
    } else {
      visorBadge.textContent = "📸 SIN CAPTURAS";
      visorImg.src = "/api/live_frame?t=" + Date.now();
    }
  }
  else if (mode === "overlay") {
    vtabOverlay.classList.add("active");
    overlayOpacityBar.classList.remove("hidden");
    visorBadge.classList.add("fusion");
    if (activeCap && activeCap.overlay_url) {
      visorBadge.textContent = `🔮 FUSIÓN 2D · ${activeCap.title}`;
      visorImg.src = activeCap.overlay_url;
    } else {
      visorBadge.textContent = "🔮 SIN ALINEAR";
      statusBanner.className = "status-banner info";
      statusBanner.textContent = "Esta toma aún no ha sido alineada. Pulsa 'Alinear con Esta Captura' para generar la fusión superpuesta.";
      if (activeCap) visorImg.src = activeCap.preview_url;
    }
  }
  else if (mode === "anaglyph") {
    vtabAnaglyph.classList.add("active");
    visorBadge.classList.add("anaglyph");
    if (activeCap && activeCap.anaglyph_url) {
      visorBadge.textContent = `🔴 ANAGLIFO 3D · ${activeCap.title}`;
      visorImg.src = activeCap.anaglyph_url;
    } else {
      visorBadge.textContent = "🔴 SIN ALINEAR";
      if (activeCap) visorImg.src = activeCap.preview_url;
    }
  }
  else if (mode === "epipolar") {
    vtabEpipolar.classList.add("active");
    visorBadge.classList.add("epipolar");
    if (activeCap && activeCap.epipolar_url) {
      visorBadge.textContent = `📏 NIVEL EPIPOLAR · ${activeCap.title}`;
      visorImg.src = activeCap.epipolar_url;
    } else {
      visorBadge.textContent = "📏 SIN ALINEAR";
      if (activeCap) visorImg.src = activeCap.preview_url;
    }
  }
  else if (mode === "matches") {
    vtabMatches.classList.add("active");
    if (activeCap && activeCap.matches_url) {
      visorBadge.classList.add("snapshot");
      visorBadge.textContent = `🎯 RAYOS · ${activeCap.title}`;
      visorImg.src = activeCap.matches_url;
    } else {
      visorBadge.textContent = "🎯 SIN ALINEAR";
      if (activeCap) visorImg.src = activeCap.preview_url;
    }
  }
}

vtabLive.addEventListener("click", () => setVisorMode("live"));
vtabSnapshot.addEventListener("click", () => setVisorMode("snapshot"));
vtabOverlay.addEventListener("click", () => setVisorMode("overlay"));
vtabAnaglyph.addEventListener("click", () => setVisorMode("anaglyph"));
vtabEpipolar.addEventListener("click", () => setVisorMode("epipolar"));
vtabMatches.addEventListener("click", () => setVisorMode("matches"));

// Control de opacidad / blend en modo Fusión 2D
if (slOverlayAlpha) {
  slOverlayAlpha.addEventListener("input", () => {
    const alphaVal = parseInt(slOverlayAlpha.value);
    lblOverlayAlpha.textContent = `${alphaVal}%`;
    if (alphaDebounce) clearTimeout(alphaDebounce);
    alphaDebounce = setTimeout(async () => {
      if (!activeCaptureId) return;
      try {
        const res = await fetch("/api/reproject", {
          method: "POST",
          headers: { "Content-Type": "application/json" },
          body: JSON.stringify({
            capture_id: activeCaptureId,
            alpha: alphaVal / 100.0
          })
        });
        const data = await res.json();
        if (data.status === "success" && visorMode === "overlay") {
          visorImg.src = data.overlay_url;
        }
      } catch (e) {}
    }, 60);
  });
}

// ==========================================
// 4. TOMAR NUEVA CAPTURA PERSONALIZADA
// ==========================================
async function takeCapture() {
  if (isCapturing) return;
  isCapturing = true;
  btnTakeCapture.disabled = true;

  captureFlash.classList.add("active");
  setTimeout(() => captureFlash.classList.remove("active"), 350);

  statusBanner.className = "status-banner info";
  statusBanner.textContent = "Congelando fotogramas de ambas cámaras...";

  try {
    const res = await fetch("/api/capture", { method: "POST" });
    const data = await res.json();

    if (data.status === "success") {
      activeCaptureId = data.capture.id;
      statusBanner.className = "status-banner success";
      statusBanner.innerHTML = `📸 <strong>¡${data.capture.title} capturada!</strong> Pulsa <strong>"Alinear con Esta Captura"</strong> para proyectar la fusión en las cámaras y el visor 3D.`;

      await refreshStatus();
      setVisorMode("snapshot");
    } else {
      statusBanner.className = "status-banner error";
      statusBanner.textContent = `Aviso: ${data.message}`;
    }
  } catch (e) {
    statusBanner.className = "status-banner error";
    statusBanner.textContent = `Error al capturar: ${e.message}`;
  } finally {
    isCapturing = false;
    btnTakeCapture.disabled = false;
  }
}

btnTakeCapture.addEventListener("click", takeCapture);

// ==========================================
// 5. EJECUTAR ALINEACIÓN VIRTUAL Y REFLEJAR EN CÁMARAS Y 3D
// ==========================================
async function runAlignment() {
  if (isAligning) return;
  isAligning = true;
  btnAlign.disabled = true;
  visorLoader.classList.remove("hidden");
  loaderText.textContent = "Calculando geometría 3D y proyectando la fusión en las cámaras...";
  statusBanner.className = "status-banner info";
  statusBanner.innerHTML = "Calculando correspondencias 3D y fusionando en tiempo real...";

  try {
    const payload = activeCaptureId ? { capture_id: activeCaptureId } : {};
    const res = await fetch("/api/align", {
      method: "POST",
      headers: { "Content-Type": "application/json" },
      body: JSON.stringify(payload)
    });
    const data = await res.json();

    if (data.status === "success") {
      statusBanner.className = "status-banner success";
      statusBanner.innerHTML = `✨ <strong>Alineación Reflejada:</strong> Solape <strong>${data.fitness_pct}%</strong> · RMSE <strong>${data.rmse_mm} mm</strong>. Fusión activa en cámaras y visor 3D.`;

      // 1. Actualizar métricas
      mFitness.textContent = `${data.fitness_pct} %`;
      mRmse.textContent = `${data.rmse_mm} mm`;
      mTime.textContent = `${data.total_time_ms} ms`;

      // 2. Sincronizar sliders UI
      updateSlidersUI(data.components);

      // 3. APLICAR DIRECTAMENTE AL VISOR 3D (THREE.JS) PARA QUE LA NUBE SE ALINEE DE INMEDIATO
      applyPoseToViewer3D(data.matrix_4x4, data.components);

      // 4. Refrescar estado para registrar URLs de overlay, anaglyph y epipolar
      await refreshStatus();

      // 5. MOSTRAR DIRECTAMENTE LA FUSIÓN SUPERPUESTA EN EL ENTORNO DE LAS CÁMARAS
      setVisorMode("overlay");
    } else {
      statusBanner.className = "status-banner error";
      statusBanner.textContent = `Aviso de alineación: ${data.message}`;
    }
  } catch (e) {
    statusBanner.className = "status-banner error";
    statusBanner.textContent = `Error: ${e.message}`;
  } finally {
    isAligning = false;
    btnAlign.disabled = false;
    visorLoader.classList.add("hidden");
  }
}

btnAlign.addEventListener("click", runAlignment);

// ==========================================
// 6. RENDERIZAR GALERÍA DE CAPTURAS
// ==========================================
function renderCapturesStrip() {
  capturesCount.textContent = capturesList.length;

  if (capturesList.length === 0) {
    galleryEmpty.classList.remove("hidden");
    const oldCards = capturesStrip.querySelectorAll(".capture-card");
    oldCards.forEach(c => c.remove());
    return;
  }

  galleryEmpty.classList.add("hidden");
  capturesStrip.innerHTML = "";

  capturesList.forEach(cap => {
    const card = document.createElement("div");
    card.className = "capture-card" + (cap.id === activeCaptureId ? " active" : "");
    card.dataset.id = cap.id;

    const badgeClass = cap.aligned ? "aligned" : "pending";
    const badgeText = cap.aligned ? `✓ ${cap.fitness_pct}%` : "Sin alinear";

    card.innerHTML = `
      <img class="card-thumb" src="${cap.preview_url}" alt="${cap.title}" loading="lazy">
      <div class="card-del-btn" title="Eliminar toma">✕</div>
      <div class="card-info">
        <div class="card-title-row">
          <span class="card-name">${cap.title}</span>
          <span class="card-time">${cap.timestamp}</span>
        </div>
        <span class="card-badge ${badgeClass}">${badgeText}</span>
      </div>
    `;

    card.addEventListener("click", (e) => {
      if (e.target.classList.contains("card-del-btn")) return;
      selectCapture(cap.id);
    });

    const delBtn = card.querySelector(".card-del-btn");
    delBtn.addEventListener("click", (e) => {
      e.stopPropagation();
      deleteCapture(cap.id);
    });

    capturesStrip.appendChild(card);
  });
}

async function selectCapture(id) {
  activeCaptureId = id;
  try {
    await fetch("/api/captures/select", {
      method: "POST",
      headers: { "Content-Type": "application/json" },
      body: JSON.stringify({ id })
    });
  } catch (e) {}

  capturesStrip.querySelectorAll(".capture-card").forEach(c => {
    c.classList.toggle("active", parseInt(c.dataset.id) === id);
  });

  const cap = capturesList.find(c => c.id === id);
  if (cap) {
    if (cap.aligned && cap.overlay_url) {
      setVisorMode("overlay");
    } else {
      setVisorMode("snapshot");
    }
    statusBanner.className = "status-banner info";
    statusBanner.textContent = `Seleccionada ${cap.title} (${cap.timestamp}). ${cap.aligned ? 'Alineada: Fusión visible en cámaras.' : 'Lista para alinear.'}`;
  }
}

async function deleteCapture(id) {
  try {
    const res = await fetch("/api/captures/delete", {
      method: "POST",
      headers: { "Content-Type": "application/json" },
      body: JSON.stringify({ id })
    });
    const data = await res.json();
    if (data.status === "success") {
      activeCaptureId = data.active_capture_id;
      await refreshStatus();
      if (activeCaptureId) {
        selectCapture(activeCaptureId);
      } else {
        setVisorMode("live");
      }
    }
  } catch (e) {}
}

if (btnClearCaptures) {
  btnClearCaptures.addEventListener("click", async () => {
    if (capturesList.length === 0) return;
    if (confirm("¿Deseas vaciar todas las tomas de la galería?")) {
      for (const cap of [...capturesList]) {
        await deleteCapture(cap.id);
      }
      setVisorMode("live");
    }
  });
}

// ==========================================
// 7. SLIDERS MANUALES A 60 HZ (SINCRONIZADOS CON THREE.JS)
// ==========================================
function updateSlidersUI(components) {
  if (!components) return;
  const t = components.translation_cm;
  const r = components.rotation_deg;

  if (t) {
    slTx.value = t.x;
    valTx.textContent = (t.x >= 0 ? "+" : "") + t.x.toFixed(1) + " cm";

    slTy.value = t.y;
    valTy.textContent = (t.y >= 0 ? "+" : "") + t.y.toFixed(1) + " cm";

    slTz.value = t.z;
    valTz.textContent = (t.z >= 0 ? "+" : "") + t.z.toFixed(1) + " cm";
  }

  if (r) {
    slYaw.value = r.yaw;
    valYaw.textContent = (r.yaw >= 0 ? "+" : "") + r.yaw.toFixed(1) + "°";

    slPitch.value = r.pitch;
    valPitch.textContent = (r.pitch >= 0 ? "+" : "") + r.pitch.toFixed(1) + "°";

    slRoll.value = r.roll;
    valRoll.textContent = (r.roll >= 0 ? "+" : "") + r.roll.toFixed(1) + "°";
  }
}

function sendManualPose() {
  const tx = parseFloat(slTx.value);
  const ty = parseFloat(slTy.value);
  const tz = parseFloat(slTz.value);
  const yaw = parseFloat(slYaw.value);
  const pitch = parseFloat(slPitch.value);
  const roll = parseFloat(slRoll.value);

  valTx.textContent = (tx >= 0 ? "+" : "") + tx.toFixed(1) + " cm";
  valTy.textContent = (ty >= 0 ? "+" : "") + ty.toFixed(1) + " cm";
  valTz.textContent = (tz >= 0 ? "+" : "") + tz.toFixed(1) + " cm";
  valYaw.textContent = (yaw >= 0 ? "+" : "") + yaw.toFixed(1) + "°";
  valPitch.textContent = (pitch >= 0 ? "+" : "") + pitch.toFixed(1) + "°";
  valRoll.textContent = (roll >= 0 ? "+" : "") + roll.toFixed(1) + "°";

  if (manualDebounce) clearTimeout(manualDebounce);
  manualDebounce = setTimeout(async () => {
    try {
      const res = await fetch("/api/set_manual", {
        method: "POST",
        headers: { "Content-Type": "application/json" },
        body: JSON.stringify({ tx, ty, tz, yaw, pitch, roll })
      });
      const data = await res.json();
      if (data.status === "success" && data.matrix_4x4) {
        // ACTUALIZAR INMEDIATAMENTE THREE.JS EN EL IFRAME A 60 FPS
        applyPoseToViewer3D(data.matrix_4x4, data.components);
      }
    } catch (e) {}
  }, 25);
}

[slTx, slTy, slTz, slYaw, slPitch, slRoll].forEach(sl => {
  sl.addEventListener("input", sendManualPose);
});

document.querySelectorAll(".step-btn").forEach(btn => {
  btn.addEventListener("click", () => {
    const axis = btn.dataset.axis;
    const delta = parseFloat(btn.dataset.delta);
    const slider = document.getElementById("sl-" + axis);
    if (slider) {
      slider.value = (parseFloat(slider.value) + delta).toString();
      sendManualPose();
    }
  });
});

btnResetPose.addEventListener("click", () => {
  updateSlidersUI({
    translation_cm: { x: -61.8, y: 2.9, z: 25.2 },
    rotation_deg: { yaw: 52.9, pitch: 2.9, roll: -4.5 }
  });
  sendManualPose();
  statusBanner.textContent = "Pose restaurada a calibración base física.";
});

// ==========================================
// 8. CONTROL DE ANCHO DE VENTANA Y RESIZER
// ==========================================
const resizer = document.getElementById("resizer");
const panelPhotogrammetry = document.getElementById("panel-photogrammetry");
const slPanelWidth = document.getElementById("sl-panel-width");
const lblPanelWidth = document.getElementById("lbl-panel-width");
const iframe3d = document.getElementById("iframe-3d");

function setPanelWidth(width) {
  const maxW = Math.max(400, window.innerWidth - 300);
  const clamped = Math.max(360, Math.min(maxW, width));
  panelPhotogrammetry.style.flex = `0 0 ${clamped}px`;
  panelPhotogrammetry.style.maxWidth = `${clamped}px`;
  if (slPanelWidth) slPanelWidth.value = clamped;
  if (lblPanelWidth) lblPanelWidth.textContent = `${clamped}px`;
  try {
    localStorage.setItem("rs_panel_width", clamped);
  } catch (e) {}
}

if (slPanelWidth) {
  slPanelWidth.addEventListener("input", (e) => {
    setPanelWidth(parseFloat(e.target.value));
  });
}

if (resizer) {
  let isDragging = false;
  resizer.addEventListener("mousedown", (e) => {
    isDragging = true;
    resizer.classList.add("dragging");
    document.body.style.cursor = "col-resize";
    document.body.style.userSelect = "none";
    if (iframe3d) iframe3d.style.pointerEvents = "none";
    e.preventDefault();
  });

  window.addEventListener("mousemove", (e) => {
    if (!isDragging) return;
    const newWidth = window.innerWidth - e.clientX;
    setPanelWidth(newWidth);
  });

  window.addEventListener("mouseup", () => {
    if (isDragging) {
      isDragging = false;
      resizer.classList.remove("dragging");
      document.body.style.cursor = "";
      document.body.style.userSelect = "";
      if (iframe3d) iframe3d.style.pointerEvents = "";
    }
  });
}

// ==========================================
// 9. SINCRONIZACIÓN Y ESTADO PERIÓDICO
// ==========================================
async function refreshStatus() {
  try {
    const res = await fetch("/api/status");
    const data = await res.json();

    if (data.captures) {
      capturesList = data.captures;
      renderCapturesStrip();
    }

    if (data.active_capture_id !== undefined && activeCaptureId === null) {
      activeCaptureId = data.active_capture_id;
    }

    if (data.current_pose && !document.querySelector("input[type=range]:active")) {
      updateSlidersUI(data.current_pose);
    }
  } catch (e) {}
}

async function init() {
  try {
    const savedWidth = localStorage.getItem("rs_panel_width");
    if (savedWidth) setPanelWidth(parseFloat(savedWidth));
  } catch (e) {}

  await refreshStatus();

  if (capturesList.length === 0) {
    setVisorMode("live");
  } else if (activeCaptureId) {
    selectCapture(activeCaptureId);
  }

  setInterval(refreshStatus, 4000);
}

init();
