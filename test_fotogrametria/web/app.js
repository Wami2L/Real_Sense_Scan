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
const vtabError = document.getElementById("vtab-error");
const vtabPoints = document.getElementById("vtab-points");

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
let manualDirty = false;      // sliders movidos y aún sin guardar: no pisarlos con el estado
let lastStatus = null;

const liveIndicator = document.getElementById("live-indicator");
const liveStatusText = document.getElementById("live-status-text");
const systemBanner = document.getElementById("system-banner");
const btnSavePose = document.getElementById("btn-save-pose");

// Previsualización de la toma
const pf = Object.fromEntries(["level", "quality", "guides", "headline", "depth", "area", "area-bar", "area-min",
  "cons", "dist", "noise", "sync", "issues", "pose", "markers", "texture"].map(k => [k, document.getElementById("pf-" + k)]));

const HTML_ESC = { "&": "&amp;", "<": "&lt;", ">": "&gt;", '"': "&quot;", "'": "&#39;" };
function escapeHtml(t) {
  return String(t ?? "").replace(/[&<>"']/g, c => HTML_ESC[c]);
}

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
  [vtabLive, vtabSnapshot, vtabOverlay, vtabAnaglyph, vtabEpipolar, vtabMatches, vtabError, vtabPoints].forEach(b => {
    if (b) b.classList.remove("active");
  });
  visorBadge.className = "visor-badge";
  overlayOpacityBar.classList.add("hidden");
  pointsPicker.classList.add("hidden");
  pointsBar.classList.add("hidden");
  pp.loupe.classList.add("hidden");
  visorImg.classList.remove("hidden");

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
  else if (mode === "error") {
    vtabError.classList.add("active");
    if (activeCap && activeCap.error_url) {
      visorBadge.classList.add("fusion");
      visorBadge.textContent = `📊 ERROR · ${activeCap.title}`;
      visorImg.src = activeCap.error_url;
    } else {
      visorBadge.textContent = "📊 SIN ALINEAR";
      if (activeCap) visorImg.src = activeCap.preview_url;
    }
  }
  else if (mode === "points") {
    vtabPoints.classList.add("active");
    visorBadge.classList.add("snapshot");
    visorBadge.textContent = activeCap ? `📍 PUNTOS · ${activeCap.title}` : "📍 SIN TOMA";
    openPointsPicker();
  }
}

vtabLive.addEventListener("click", () => setVisorMode("live"));
vtabSnapshot.addEventListener("click", () => setVisorMode("snapshot"));
vtabOverlay.addEventListener("click", () => setVisorMode("overlay"));
vtabAnaglyph.addEventListener("click", () => setVisorMode("anaglyph"));
vtabEpipolar.addEventListener("click", () => setVisorMode("epipolar"));
vtabMatches.addEventListener("click", () => setVisorMode("matches"));
vtabError.addEventListener("click", () => setVisorMode("error"));
vtabPoints.addEventListener("click", () => setVisorMode("points"));

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
    applySystemState(lastStatus?.system);
  }
}

btnTakeCapture.addEventListener("click", takeCapture);

// ==========================================
// 5. EJECUTAR ALINEACIÓN VIRTUAL Y REFLEJAR EN CÁMARAS Y 3D
// ==========================================
async function runAlignment(opts = {}) {
  if (isAligning) return;
  const multi = opts.multi ? [...multiSelected] : null;
  const payload = multi ? { capture_ids: multi } : (activeCaptureId ? { capture_id: activeCaptureId } : {});
  let sentIdx = null;
  if (!multi) {
    if (manualDirty) payload.seed = currentSliderPose();   // ajuste a mano como punto de partida
    if (alignMethod === "manual") {
      sentIdx = manualPairs.map((p, i) => (p.c2 ? i : -1)).filter(i => i >= 0);
      if (sentIdx.length < 3) {
        statusBanner.className = "status-banner error";
        statusBanner.textContent = "Puntos a mano: marca al menos 3 parejas completas (mejor 5–6) en la pestaña 📍 Puntos.";
        setVisorMode("points");
        return;
      }
      payload.manual_pairs = sentIdx.map(i => ({ c1: manualPairs[i].c1, c2: manualPairs[i].c2 }));
    }
  }
  if (opts.ignorePrior) payload.ignore_prior = true;

  isAligning = true;
  btnAlign.disabled = true;
  updateMultiButton();
  visorLoader.classList.remove("hidden");
  loaderText.textContent = multi ? `Alineando ${multi.length} tomas y combinando sus poses...`
    : "Calculando correspondencias 3D, ICP y mapa de error...";
  statusBanner.className = "status-banner info";
  statusBanner.innerHTML = multi ? "Alineando cada toma y combinando poses..." : "Calculando la alineación...";

  try {
    const res = await fetch(multi ? "/api/align_multi" : "/api/align", {
      method: "POST",
      headers: { "Content-Type": "application/json" },
      body: JSON.stringify(payload)
    });
    const data = await res.json();
    const m = data.metrics || {};
    const warnList = (data.warnings || []).length
      ? `<ul class="warn-list">${data.warnings.map(w => `<li>${escapeHtml(w)}</li>`).join("")}</ul>` : "";

    if (m.consistency_pct !== undefined) {
      mFitness.textContent = `${m.consistency_pct} %`;
      mRmse.textContent = m.rmse_mm != null ? `${m.rmse_mm} mm (±${m.expected_sigma_mm})` : "-- mm";
    }
    mTime.textContent = data.total_time_ms ? `${Math.round(data.total_time_ms)} ms` : "-- ms";
    if (data.capture_id) activeCaptureId = data.capture_id;
    renderDetail(data);
    renderErrorStats(data.error_stats);
    updatePairsFromResult(data, sentIdx);

    if (data.status === "success") {
      statusBanner.className = data.quality === "buena" ? "status-banner success" : "status-banner warning";
      statusBanner.innerHTML = `✨ <strong>Alineación ${escapeHtml(data.quality)}</strong>${data.applied ? " · guardada en 5002" : ""}: `
        + `${escapeHtml(data.message)}${warnList}`;
      manualDirty = false;
      updateSlidersUI(data.components);
      applyPoseToViewer3D(data.matrix_4x4, data.components);
      await refreshStatus();
      setVisorMode(alignMethod === "manual" && !multi ? "points" : "overlay");
    } else if (data.status === "warning") {
      // No fiable: no se aplica. Las vistas muestran la pose candidata o la calibración VIGENTE.
      statusBanner.className = "status-banner warning";
      statusBanner.innerHTML = `⚠️ <strong>No se aplicó.</strong> ${escapeHtml(data.message)}${warnList}`;
      if (data.needs_confirmation && data.quality_if_confirmed) {
        // La geometría encaja bien pero contradice la calibración guardada: lo decide el usuario
        const c = data.candidate_components || {};
        const t = c.translation_cm || {}, r = c.rotation_deg || {};
        const btn = document.createElement("button");
        btn.className = "btn-confirm-moved";
        btn.textContent = `Moví las cámaras: aplicar esta pose (${data.quality_if_confirmed})`;
        btn.title = `Tx ${t.x} cm · Ty ${t.y} cm · Tz ${t.z} cm · Yaw ${r.yaw}° · Pitch ${r.pitch}° · Roll ${r.roll}°`;
        btn.addEventListener("click", () => runAlignment({ ...opts, ignorePrior: true }));
        statusBanner.appendChild(btn);
        const hint = document.createElement("div");
        hint.className = "confirm-hint";
        hint.textContent = "Antes de confirmar, mira la 🔮 Fusión 2D y el 📊 mapa de error: muestran esta pose candidata.";
        statusBanner.appendChild(hint);
      }
      await refreshStatus();
      setVisorMode(data.error_url ? "error" : (data.overlay_url ? "overlay" : "matches"));
    } else {
      statusBanner.className = "status-banner error";
      statusBanner.textContent = `Aviso de alineación: ${data.message}`;
    }
  } catch (e) {
    statusBanner.className = "status-banner error";
    statusBanner.textContent = `Error: ${e.message}`;
  } finally {
    isAligning = false;
    applySystemState(lastStatus?.system);
    updateMultiButton();
    visorLoader.classList.add("hidden");
    refreshHistory();
    updateWizard();
  }
}

btnAlign.addEventListener("click", () => runAlignment());

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
    const badgeText = cap.aligned ? `✓ ${cap.quality || ""} ${cap.fitness_pct}%`
      : (cap.quality === "insuficiente" ? "✗ No fiable" : "Sin alinear") + (cap.simulated ? " · sim" : "");

    card.innerHTML = `
      <img class="card-thumb" src="${cap.preview_url}" alt="${cap.title}" loading="lazy">
      <div class="card-del-btn" title="Eliminar toma">✕</div>
      <label class="card-multi" title="Usar en la alineación multi-toma"><input type="checkbox" ${multiSelected.has(cap.id) ? "checked" : ""}> multi</label>
      <div class="card-info">
        <div class="card-title-row">
          <span class="card-name">${cap.title}</span>
          <span class="card-time">${cap.timestamp}</span>
        </div>
        <span class="card-badge ${badgeClass}">${badgeText}</span>
      </div>
    `;

    card.addEventListener("click", (e) => {
      if (e.target.classList.contains("card-del-btn") || e.target.closest(".card-multi")) return;
      selectCapture(cap.id);
    });
    const multiBox = card.querySelector(".card-multi input");
    multiBox.addEventListener("change", () => {
      if (multiBox.checked) multiSelected.add(cap.id); else multiSelected.delete(cap.id);
      updateMultiButton();
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
    if (cap.detail) renderDetail(cap.detail);
    renderErrorStats(cap.error_stats);
    if (alignMethod === "manual") setVisorMode("points");
  }
  updateWizard();
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
        if (visorMode === "error") scheduleErrorMap();
      }
    } catch (e) {}
  }, 25);
}

function currentSliderPose() {
  return {
    tx: parseFloat(slTx.value), ty: parseFloat(slTy.value), tz: parseFloat(slTz.value),
    yaw: parseFloat(slYaw.value), pitch: parseFloat(slPitch.value), roll: parseFloat(slRoll.value)
  };
}

[slTx, slTy, slTz, slYaw, slPitch, slRoll].forEach(sl => {
  sl.addEventListener("input", () => { manualDirty = true; sendManualPose(); });
});

document.querySelectorAll(".step-btn").forEach(btn => {
  btn.addEventListener("click", () => {
    const axis = btn.dataset.axis;
    const delta = parseFloat(btn.dataset.delta);
    const slider = document.getElementById("sl-" + axis);
    if (slider) {
      slider.value = (parseFloat(slider.value) + delta).toString();
      manualDirty = true;
      sendManualPose();
    }
  });
});

btnResetPose.addEventListener("click", async () => {
  await refreshStatus();
  if (!lastStatus?.current_pose) {
    statusBanner.className = "status-banner error";
    statusBanner.textContent = "No hay calibración disponible en 5002.";
    return;
  }
  manualDirty = false;
  updateSlidersUI(lastStatus.current_pose);
  sendManualPose();
  statusBanner.className = "status-banner info";
  statusBanner.textContent = "Sliders restaurados a la calibración guardada (no se ha modificado nada).";
});

if (btnSavePose) {
  btnSavePose.addEventListener("click", async () => {
    btnSavePose.disabled = true;
    try {
      const res = await fetch("/api/save_pose", {
        method: "POST", headers: { "Content-Type": "application/json" },
        body: JSON.stringify(currentSliderPose())
      });
      const data = await res.json();
      statusBanner.className = data.status === "success" ? "status-banner success" : "status-banner error";
      statusBanner.textContent = data.message;
      if (data.status === "success") manualDirty = false;
    } catch (e) {
      statusBanner.className = "status-banner error";
      statusBanner.textContent = `Error al guardar: ${e.message}`;
    } finally {
      btnSavePose.disabled = false;
    }
  });
}

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
function applySystemState(sys) {
  if (!sys) return;
  const cls = { live: "online", simulated: "sim", degraded: "warn", stale: "warn", starting: "warn" }[sys.state] || "error";
  liveIndicator.className = `status-indicator ${cls}`;
  liveStatusText.textContent = sys.label;
  liveIndicator.title = sys.detail || sys.label;

  if (sys.state === "live") {
    systemBanner.className = "system-banner hidden";
  } else {
    systemBanner.className = `system-banner ${sys.state === "simulated" ? "sim" : (cls === "error" ? "error" : "")}`;
    systemBanner.innerHTML = `<strong>${escapeHtml(sys.label)}.</strong> ${escapeHtml(sys.detail || "")}`;
  }
  btnTakeCapture.disabled = isCapturing || !sys.can_capture;
  btnTakeCapture.title = sys.can_capture ? "Capturar lo que ven ambas cámaras ahora" : `No disponible: ${sys.label}`;
  // Una toma ya guardada se puede alinear aunque las cámaras estén caídas
  btnAlign.disabled = isAligning || (!sys.can_capture && !activeCaptureId);
}

async function refreshStatus() {
  try {
    const res = await fetch("/api/status");
    const data = await res.json();
    lastStatus = data;
    applySystemState(data.system);

    if (data.captures) {
      capturesList = data.captures;
      renderCapturesStrip();
    }

    if (data.active_capture_id !== undefined && activeCaptureId === null) {
      activeCaptureId = data.active_capture_id;
    }

    if (data.current_pose && !manualDirty && !document.querySelector("input[type=range]:active")) {
      updateSlidersUI(data.current_pose);
    }
    updateMultiButton();
    updateWizard();
  } catch (e) {
    applySystemState({ state: "offline", label: "Estudio sin respuesta", detail: e.message, can_capture: false });
  }
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

  setInterval(refreshStatus, 2000);
}

// ==========================================
// 10. PREVISUALIZACIÓN DE LA TOMA (CALIDAD ESPERADA ANTES DE CAPTURAR)
// ==========================================
const PF_LEVEL = {
  lista: ["pf-ok", "✅ Lista"], mejorable: ["pf-warn", "⚠️ Mejorable"], bloqueada: ["pf-block", "⛔ No capturar"],
};
const PF_ICON = { block: "⛔", warn: "⚠️", info: "ℹ️", ok: "✅" };

function renderPreflight(d) {
  if (!d || d.status !== "ok") return;
  const [cls, txt] = PF_LEVEL[d.level] || ["pf-pending", "Analizando…"];
  pf.level.className = `pf-level ${cls}`;
  pf.level.textContent = txt;
  pf.quality.textContent = `Calidad esperada: ${d.expected_quality || "--"}`;
  pf.headline.textContent = d.headline || "";
  btnTakeCapture.dataset.preflight = d.level || "";

  const cams = d.cams || [];
  pf.depth.innerHTML = cams.length
    ? cams.map((c, i) => `<span class="${c.valid_pct < 3 || c.near_pct > 10 ? "pf-bad" : ""}">C${i + 1} ${Math.round(c.valid_pct)}%${c.near_pct > 10 ? " ⚠" : ""}</span>`).join(" · ")
    : "--";
  const o = d.overlap;
  if (o) {
    pf.area.textContent = `${o.area_m2.toFixed(3)} m² (mín. ${o.min_area_m2})`;
    const full = o.recommended_area_m2 * 1.5;
    pf["area-bar"].style.width = `${Math.min(100, (o.area_m2 / full) * 100)}%`;
    pf["area-bar"].className = `pf-bar-fill ${o.area_m2 >= o.recommended_area_m2 ? "good" : o.area_m2 >= o.min_area_m2 ? "ok" : "bad"}`;
    pf["area-min"].style.left = `${(o.min_area_m2 / full) * 100}%`;
    pf.cons.textContent = `${o.consistency_pct}% · exacto ${o.pose_precision_pct}%`;
    pf.dist.textContent = o.recommended_depth_m != null
      ? `${o.recommended_depth_m} m (${o.depth_range_m[0]}–${o.depth_range_m[1]})` : "--";
  } else {
    pf.area.textContent = "sin pose";
    pf["area-bar"].style.width = "0%";
    pf.cons.textContent = "--";
    pf.dist.textContent = "--";
  }
  pf.noise.textContent = d.expected_noise_mm != null ? `~${d.expected_noise_mm} mm` : "--";
  const moving = Math.max(0, ...cams.map(c => c.moving_pct || 0));
  pf.sync.textContent = `${d.dt_ms != null ? d.dt_ms + " ms" : "--"} · ${Math.round(moving)}% mov.`;
  pf.issues.innerHTML = (d.issues || []).map(i =>
    `<li class="pf-${i.severity}">${PF_ICON[i.severity] || ""} ${escapeHtml(i.text)}</li>`).join("");
  pf.pose.textContent = `Pose usada: ${d.pose_source || "--"}${d.age_s != null ? ` · hace ${d.age_s.toFixed(1)} s` : ""}`;
  const mk = d.markers;
  pf.markers.innerHTML = !mk ? "--" : mk.common.length ? `<span class="pf-good">comunes ${mk.common.join(", ")}</span>`
    : (mk.cam1.length || mk.cam2.length) ? `C1 ${fmtIds(mk.cam1)} · C2 ${fmtIds(mk.cam2)}` : "ninguno";
  pf.texture.textContent = d.texture_points != null ? `${d.texture_points} puntos` : "--";
  lastPreflight = d;
  updateWizard();
  if (pf.guides.checked !== d.guides) pf.guides.checked = d.guides;
}

async function refreshPreflight() {
  if (document.hidden || visorMode !== "live") return;
  try {
    const res = await fetch("/api/preflight");
    renderPreflight(await res.json());
  } catch (e) {}
}

pf.guides.addEventListener("change", () => {
  fetch("/api/guides", {
    method: "POST", headers: { "Content-Type": "application/json" },
    body: JSON.stringify({ on: pf.guides.checked }),
  }).catch(() => {});
});

setInterval(refreshPreflight, 700);

// ==========================================
// 11. ASISTENTE, MÉTODO, PUNTOS A MANO, DETALLE, MULTI-TOMA E HISTORIAL
// ==========================================
const wz = {
  hint: document.getElementById("wz-hint"),
  mount: document.getElementById("wz-mount-ok"),
  steps: [1, 2, 3, 4, 5, 6].map(n => document.getElementById("wz-" + n)),
};
const pointsPicker = document.getElementById("points-picker");
const pointsBar = document.getElementById("points-bar");
const pp = {
  img: [document.getElementById("pp-img1"), document.getElementById("pp-img2")],
  svg: [document.getElementById("pp-svg1"), document.getElementById("pp-svg2")],
  loupe: document.getElementById("pp-loupe"),
  status: document.getElementById("pp-status"),
  list: document.getElementById("pp-list"),
  align: document.getElementById("pp-align"),
};
const adSummary = document.getElementById("ad-summary");
const adCandidates = document.getElementById("ad-candidates");
const adCorr = document.getElementById("ad-corr");
const adError = document.getElementById("ad-error");
const btnAlignMulti = document.getElementById("btn-align-multi");
const historyList = document.getElementById("history-list");
const btnUndoCalib = document.getElementById("btn-undo-calib");

let alignMethod = "auto";
let manualPairs = [];          // [{c1: [u, v], c2: [u, v] | null, status: null | {ok, residual_mm, note}}]
const pairsByCap = {};         // id de toma -> sus parejas (se conservan al cambiar de toma)
let ppCapId = null;
let lastPreflight = null;
let corrKind = "all";
let lastCorr = [];
const multiSelected = new Set();

function activeCap() { return capturesList.find(c => c.id === activeCaptureId); }
function fmtIds(a) { return a && a.length ? a.join(", ") : "ninguno"; }

// ---------- Asistente ----------
const WZ_HINTS = {
  1: "Paso 1 · Fija las dos cámaras (trípode o soporte rígido): si una se mueve, la calibración deja de valer. Marca «Montaje fijo».",
  2: "Paso 2 · Mira la vista en vivo: pon el sujeto u objetos con volumen dentro del contorno cian y nada a menos de 0,52 m (rojo). Lo más fiable: marcadores ArUco impresos que vean ambas cámaras.",
  3: "Paso 3 · Cuando el panel diga «Lista» (o haya marcadores comunes), pulsa «Tomar Nueva Captura» y quédate quieto.",
  4: "Paso 4 · «Automático» usa marcadores, fotogrametría y geometría. Si no hay marcadores y falla, usa «Puntos a mano»: marca 4–6 parejas en la pestaña 📍 Puntos.",
  5: "Paso 5 · Pulsa «Alinear». En el detalle verás qué hipótesis ganó, las parejas y el mapa de error (📊: verde = encaja).",
  6: "Paso 6 · Si es fiable se guarda solo. Si pide confirmación porque moviste las cámaras, revisa la fusión 2D y el mapa de error y confirma. Siempre puedes deshacer en el historial.",
};

function updateWizard() {
  const cap = activeCap();
  const pairsDone = manualPairs.filter(p => p.c2).length;
  const done = {
    1: wz.mount.checked,
    2: !!lastPreflight && !!lastPreflight.level && lastPreflight.level !== "bloqueada",
    3: !!cap,
    4: alignMethod === "auto" || pairsDone >= 3,
    5: !!cap && !!cap.quality,
    6: !!cap && !!cap.aligned,
  };
  const current = [1, 2, 3, 4, 5, 6].find(n => !done[n]);
  wz.steps.forEach((el, i) => { el.className = done[i + 1] ? "done" : (i + 1 === current ? "current" : ""); });
  wz.hint.textContent = current
    ? WZ_HINTS[current]
    : "✅ Calibración guardada. Verifícala en 📊 Error o con el visor 3D en modo Solape; puedes deshacerla en el historial.";
}

try { wz.mount.checked = localStorage.getItem("rs_mount_ok") === "1"; } catch (e) {}
wz.mount.addEventListener("change", () => {
  try { localStorage.setItem("rs_mount_ok", wz.mount.checked ? "1" : "0"); } catch (e) {}
  updateWizard();
});

function setAlignMethod(m) {
  alignMethod = m;
  document.querySelectorAll(".seg-btn[data-method]").forEach(b => b.classList.toggle("active", b.dataset.method === m));
  if (m === "manual") {
    if (!activeCap()) {
      statusBanner.className = "status-banner info";
      statusBanner.textContent = "Primero toma una captura: los puntos se marcan sobre una toma congelada.";
    } else {
      setVisorMode("points");
    }
  } else if (visorMode === "points") {
    setVisorMode("snapshot");
  }
  updateWizard();
}
document.querySelectorAll(".seg-btn[data-method]").forEach(b => b.addEventListener("click", () => setAlignMethod(b.dataset.method)));

// ---------- Puntos a mano ----------
function openPointsPicker() {
  const cap = activeCap();
  visorImg.classList.add("hidden");
  pointsPicker.classList.remove("hidden");
  pointsBar.classList.remove("hidden");
  if (!cap || !cap.cam1_url) {
    pp.status.textContent = cap ? "Esta toma no tiene imágenes por cámara: toma una nueva." : "Toma primero una captura.";
    pp.img.forEach(i => i.removeAttribute("src"));
    return;
  }
  if (ppCapId !== cap.id) {
    ppCapId = cap.id;
    manualPairs = pairsByCap[cap.id] || (pairsByCap[cap.id] = []);
  }
  pp.img[0].src = cap.cam1_url;
  pp.img[1].src = cap.cam2_url;
  renderPairs();
}

function imgCoords(img, ev) {
  const r = img.getBoundingClientRect();
  return [(ev.clientX - r.left) * img.naturalWidth / r.width, (ev.clientY - r.top) * img.naturalHeight / r.height];
}

pp.img.forEach((img, k) => {
  img.addEventListener("load", () => {
    pp.svg[k].setAttribute("viewBox", `0 0 ${img.naturalWidth} ${img.naturalHeight}`);
    renderPairs();
  });
  img.addEventListener("click", ev => {
    if (!img.naturalWidth) return;
    const uv = imgCoords(img, ev).map(v => Math.round(v * 10) / 10);
    const last = manualPairs[manualPairs.length - 1];
    if (k === 0) {
      if (last && !last.c2) last.c1 = uv;
      else manualPairs.push({ c1: uv, c2: null, status: null });
    } else {
      if (!last || last.c2) {
        pp.status.textContent = "Primero marca el punto en la Cam 1; luego el mismo punto en la Cam 2.";
        return;
      }
      last.c2 = uv;
    }
    manualPairs.forEach(p => { p.status = null; });  // las parejas cambiaron: el resultado anterior ya no vale
    renderPairs();
    updateWizard();
  });
  // Lupa 4x para afinar el clic
  img.addEventListener("mousemove", ev => {
    if (!img.naturalWidth) return;
    const [u, v] = imgCoords(img, ev);
    const c = pp.loupe, ctx = c.getContext("2d");
    const src = 40;
    ctx.imageSmoothingEnabled = false;
    ctx.fillStyle = "#000";
    ctx.fillRect(0, 0, c.width, c.height);
    ctx.drawImage(img, u - src / 2, v - src / 2, src, src, 0, 0, c.width, c.height);
    ctx.strokeStyle = "#facc15";
    ctx.beginPath();
    ctx.moveTo(c.width / 2, 0); ctx.lineTo(c.width / 2, c.height);
    ctx.moveTo(0, c.height / 2); ctx.lineTo(c.width, c.height / 2);
    ctx.stroke();
    const host = pointsPicker.getBoundingClientRect();
    let x = ev.clientX - host.left + 18, y = ev.clientY - host.top + 18;
    if (x + c.width > host.width) x -= c.width + 36;
    if (y + c.height > host.height) y -= c.height + 36;
    c.style.left = `${x}px`;
    c.style.top = `${y}px`;
    c.classList.remove("hidden");
  });
  img.addEventListener("mouseleave", () => pp.loupe.classList.add("hidden"));
});

function renderPairs() {
  [0, 1].forEach(k => {
    const W = pp.img[k].naturalWidth || 848;
    const r = W / 110;
    pp.svg[k].innerHTML = manualPairs.map((p, i) => {
      const c = k === 0 ? p.c1 : p.c2;
      if (!c) return "";
      const col = !p.c2 ? "#facc15" : p.status ? (p.status.ok ? "#22c55e" : "#ef4444") : "#22d3ee";
      return `<circle cx="${c[0]}" cy="${c[1]}" r="${r}" fill="none" stroke="${col}" stroke-width="${r / 3}"/>` +
        `<circle cx="${c[0]}" cy="${c[1]}" r="${r / 4}" fill="${col}"/>` +
        `<text x="${c[0] + r * 1.3}" y="${c[1] - r * 1.1}" fill="${col}" font-size="${r * 2.3}" font-weight="700" ` +
        `stroke="#000" stroke-width="${r / 5}" paint-order="stroke">${i + 1}</text>`;
    }).join("");
  });
  const complete = manualPairs.filter(p => p.c2).length;
  pp.align.disabled = complete < 3;
  pp.align.textContent = `Alinear con ${complete} pareja${complete === 1 ? "" : "s"}`;
  const pending = manualPairs.length && !manualPairs[manualPairs.length - 1].c2;
  pp.status.textContent = pending
    ? `Punto ${manualPairs.length}: ahora haz clic en el MISMO punto en la Cam 2.`
    : complete < 3 ? `Llevas ${complete} pareja(s). Hacen falta al menos 3 (mejor 5–6).`
      : `${complete} parejas listas. Añade más o pulsa «Alinear».`;
  pp.list.innerHTML = manualPairs.map((p, i) => {
    const st = p.status;
    const cls = st ? (st.ok ? "ok" : "bad") : "";
    const res = st ? `${st.residual_mm != null ? ` · ${st.residual_mm} mm` : ""}${st.note ? ` · ${escapeHtml(st.note)}` : ""}${st.ok ? " ✓" : " ✗"}` : "";
    return `<li class="${cls}">C1 (${p.c1.map(Math.round).join(", ")}) → ${p.c2 ? `C2 (${p.c2.map(Math.round).join(", ")})` : "<i>falta Cam 2</i>"}${res}` +
      ` <button class="pp-del" data-i="${i}" title="Quitar esta pareja">✕</button></li>`;
  }).join("");
  pp.list.querySelectorAll(".pp-del").forEach(b => b.addEventListener("click", () => {
    manualPairs.splice(+b.dataset.i, 1);
    renderPairs();
    updateWizard();
  }));
}

document.getElementById("pp-undo").addEventListener("click", () => {
  const last = manualPairs[manualPairs.length - 1];
  if (!last) return;
  if (last.c2) last.c2 = null; else manualPairs.pop();
  renderPairs();
  updateWizard();
});
document.getElementById("pp-clear").addEventListener("click", () => {
  manualPairs.length = 0;
  renderPairs();
  updateWizard();
});
document.getElementById("pp-full").addEventListener("click", () => {
  setWorkspaceMode(workspace.classList.contains("mode-matches") ? "split" : "matches");
});
pp.align.addEventListener("click", () => runAlignment());

function updatePairsFromResult(data, sentIdx) {
  if (!sentIdx) return;
  const byLabel = {};
  (data.correspondences || []).filter(c => c.kind === "manual").forEach(c => { byLabel[c.label] = c; });
  sentIdx.forEach((pairIdx, k) => {
    const c = byLabel[`P${k + 1}`];
    if (c && manualPairs[pairIdx]) {
      manualPairs[pairIdx].status = { ok: c.ok, residual_mm: c.residual_mm, note: c.note };
    }
  });
  renderPairs();
}

// ---------- Detalle de la alineación ----------
const CAND_NAMES = [
  ["puntos fijados a mano", "📍 Puntos a mano"], ["marcadores ArUco", "🔲 Marcadores ArUco"],
  ["SIFT 3D + RANSAC", "🖼️ Fotogrametría (SIFT)"], ["calibración vigente", "💾 Calibración guardada"],
  ["pose de los sliders", "🎚️ Pose de los sliders"], ["FPFH global (geometría)", "🧊 Forma 3D (FPFH)"],
  ["búsqueda de montaje", "🔍 Búsqueda de montaje"],
];
function candName(n) {
  for (const [k, v] of CAND_NAMES) if (n.startsWith(k)) return v + n.slice(k.length);
  return n;
}

function renderErrorStats(st) {
  if (!st || st.median_mm == null) {
    adError.innerHTML = st ? "<p>📊 Mapa de error: sin superficie común con esta pose.</p>" : "";
    return;
  }
  const verdict = st.median_mm <= Math.max(5, 2 * st.expected_sigma_mm) ? "encaja bien"
    : st.median_mm <= 15 ? "encaja con algo de error" : "no encaja";
  adError.innerHTML = `<p>📊 <b>Mapa de error (pose ${escapeHtml(st.pose || "")}): ${verdict}.</b> ` +
    `Mitad de la superficie común a menos de <b>${st.median_mm} mm</b>, 90 % a menos de ${st.p90_mm} mm; ` +
    `${st.pct_lt_10} % por debajo de 1 cm sobre ${st.area_m2} m² comunes (ruido esperado ~${st.expected_sigma_mm} mm). ` +
    `<button class="btn-tiny" id="ad-see-error">Ver mapa</button></p>`;
  document.getElementById("ad-see-error").addEventListener("click", () => setVisorMode("error"));
}

function renderDetail(d) {
  if (!d) return;
  const m = d.metrics || {};
  const ev = { manual: "los puntos que marcaste", marcadores: "los marcadores ArUco", sift: "la fotogrametría (SIFT)" }[m.evidence]
    || "solo la geometría (no hubo marcadores, puntos a mano ni fotogrametría suficiente)";
  const L = [`<b>Resultado: ${escapeHtml(d.quality || "--")}.</b> ${escapeHtml(d.message || "")}`];
  if (m.consistency_pct != null) {
    L.push(`De cada 100 puntos que ven las dos cámaras, <b>${Math.round(m.consistency_pct)}</b> coinciden con la pose elegida ` +
      `(${m.overlap_points} puntos comunes en una rejilla de 1 cm).`);
  }
  if (m.rmse_mm != null) {
    L.push(`Distancia media entre superficies (RMSE): <b>${m.rmse_mm} mm</b>. El ruido propio del sensor a ${m.median_depth_m} m ` +
      `es ~${m.expected_sigma_mm} mm: hasta 2–3 veces ese valor es normal.`);
  }
  if (m.matches !== undefined || m.markers) {
    L.push(`Base de la pose: <b>${ev}</b>. Fotogrametría: ${m.matches ?? 0} parejas SIFT, ${m.lifted ?? 0} con profundidad, ` +
      `${m.inliers ?? 0} coherentes.` + (m.markers ? ` Marcadores: Cam 1 ${fmtIds(m.markers.cam1)} · Cam 2 ${fmtIds(m.markers.cam2)} · usados ${fmtIds(m.markers.used)}.` : "") +
      (m.manual_pairs ? ` A mano: ${m.manual_pairs.valid} de ${m.manual_pairs.given} parejas con profundidad.` : ""));
  }
  if (d.multi && d.multi.per_capture) {
    L.push("Tomas: " + d.multi.per_capture.map(p => `${escapeHtml(p.title)} → ${escapeHtml(p.quality)}` +
      (p.quality_if_confirmed ? ` (${escapeHtml(p.quality_if_confirmed)} si confirmas)` : "") +
      (p.consistency_pct != null ? `, ${p.consistency_pct}%` : "")).join(" · ") + ".");
  }
  if (d.multi && d.multi.dispersion) {
    L.push(`Multi-toma: ${d.multi.used.length} tomas combinadas (dispersión máx. ${d.multi.dispersion.max_translation_cm} cm / ` +
      `${d.multi.dispersion.max_rotation_deg}°)` + (d.multi.discarded && d.multi.discarded.length ? `; descartadas ${d.multi.discarded.join(", ")}` : "") + ".");
  }
  (d.warnings || []).forEach(w => L.push(`⚠️ ${escapeHtml(w)}`));
  adSummary.innerHTML = L.map(l => `<p>${l}</p>`).join("");
  adCandidates.innerHTML = "<tr><th>Hipótesis</th><th>Encaje</th><th>Comunes</th><th>RMSE</th><th>Acuerdo</th></tr>" +
    (d.candidates || []).map(c => `<tr class="${c.chosen ? "chosen" : ""}${c.viable === false ? " dim" : ""}" ` +
      `title="${c.viable === false ? "Descartada: poca zona común o pocas pruebas" : ""}">` +
      `<td>${c.chosen ? "★ " : ""}${escapeHtml(candName(c.name))}</td><td>${c.consistency_pct}%</td><td>${c.overlap_points}</td>` +
      `<td>${c.rmse_mm ?? "--"}</td><td>${c.feature_agreement_pct != null ? c.feature_agreement_pct + "%" : "--"}</td></tr>`).join("");
  lastCorr = d.correspondences || [];
  renderCorr();
}

function renderCorr() {
  const rows = lastCorr.filter(c => corrKind === "all" || c.kind === corrKind).slice(0, 80);
  const KIND = { manual: "📍 a mano", marcador: "🔲 marcador", sift: "🖼️ SIFT" };
  adCorr.innerHTML = rows.length
    ? "<tr><th>#</th><th>Tipo</th><th>Residuo</th><th>Estado</th></tr>" + rows.map(c =>
      `<tr class="${c.ok ? "" : "bad"}"><td>${escapeHtml(c.label)}</td><td>${KIND[c.kind] || c.kind}</td>` +
      `<td>${c.residual_mm != null ? c.residual_mm + " mm" : "--"}</td>` +
      `<td>${c.ok ? "✓ encaja" : "✗ " + escapeHtml(c.note || "no encaja")}</td></tr>`).join("")
    : `<tr><td class="dim">Sin parejas de este tipo.</td></tr>`;
}
document.querySelectorAll(".seg-btn[data-kind]").forEach(b => b.addEventListener("click", () => {
  corrKind = b.dataset.kind;
  document.querySelectorAll(".seg-btn[data-kind]").forEach(x => x.classList.toggle("active", x === b));
  renderCorr();
}));

// ---------- Mapa de error que sigue a los sliders ----------
let errDebounce = null;
function scheduleErrorMap() {
  clearTimeout(errDebounce);
  errDebounce = setTimeout(async () => {
    if (!activeCaptureId) return;
    try {
      const res = await fetch("/api/error_map", {
        method: "POST", headers: { "Content-Type": "application/json" },
        body: JSON.stringify({ capture_id: activeCaptureId }),
      });
      const d = await res.json();
      if (d.status !== "success") return;
      const cap = activeCap();
      if (cap) { cap.error_url = d.error_url; cap.error_stats = d.error_stats; }
      if (visorMode === "error") visorImg.src = d.error_url;
      renderErrorStats(d.error_stats);
    } catch (e) {}
  }, 300);
}

// ---------- Multi-toma ----------
function updateMultiButton() {
  for (const id of [...multiSelected]) if (!capturesList.some(c => c.id === id)) multiSelected.delete(id);
  btnAlignMulti.disabled = multiSelected.size < 2 || isAligning;
  btnAlignMulti.textContent = `Alinear con ${multiSelected.size} tomas`;
}
btnAlignMulti.addEventListener("click", () => runAlignment({ multi: true }));

// ---------- Historial de calibraciones ----------
function fmtCal(e) {
  return `${escapeHtml(e.calibration_date || "?")} · ${escapeHtml(e.method || "?")}` +
    `${e.quality ? " · " + escapeHtml(e.quality) : ""}${e.consistency_pct != null ? " " + e.consistency_pct + "%" : ""}`;
}
async function refreshHistory() {
  try {
    const res = await fetch("/api/calibration/history");
    const d = await res.json();
    if (d.status !== "success") {
      historyList.innerHTML = `<li class="dim">${escapeHtml(d.message || "Sin historial")}</li>`;
      return;
    }
    const comp = d.current_components ? ` <span class="dim">Tx ${d.current_components.translation_cm.x} cm · Yaw ${d.current_components.rotation_deg.yaw}°</span>` : "";
    historyList.innerHTML = `<li class="current"><b>Vigente</b> · ${fmtCal(d.current || {})}${comp}</li>` +
      (d.history || []).map(e => `<li>${fmtCal(e)} <span class="dim">Tx ${e.components.translation_cm.x} cm · ` +
        `Yaw ${e.components.rotation_deg.yaw}°</span> <button class="btn-tiny" data-restore="${e.index}">Restaurar</button></li>`).join("");
    btnUndoCalib.disabled = !(d.history && d.history.length);
    historyList.querySelectorAll("[data-restore]").forEach(b => b.addEventListener("click", () => restoreCalibration(+b.dataset.restore)));
  } catch (e) {}
}
async function restoreCalibration(index) {
  try {
    const res = await fetch("/api/calibration/restore", {
      method: "POST", headers: { "Content-Type": "application/json" }, body: JSON.stringify({ index }),
    });
    const d = await res.json();
    statusBanner.className = d.status === "success" ? "status-banner success" : "status-banner error";
    statusBanner.textContent = d.status === "success" ? "↶ Calibración anterior restaurada (puedes rehacer con «Deshacer» otra vez)." : `No se pudo restaurar: ${d.message}`;
    if (d.status === "success") {
      manualDirty = false;
      await refreshStatus();
      if (lastStatus && lastStatus.current_pose) {
        updateSlidersUI(lastStatus.current_pose);
        applyPoseToViewer3D(d.matrix_4x4, lastStatus.current_pose);
      }
    }
  } catch (e) {
    statusBanner.className = "status-banner error";
    statusBanner.textContent = `No se pudo restaurar: ${e.message}`;
  }
  refreshHistory();
}
btnUndoCalib.addEventListener("click", () => restoreCalibration(0));

setInterval(refreshHistory, 10000);

init().then(() => { refreshHistory(); updateWizard(); });
