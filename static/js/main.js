document.addEventListener('DOMContentLoaded', () => {
    const captureBtn = document.getElementById('captureBtn');
    const statusMessage = document.getElementById('statusMessage');

    function showStatus(message, type, duration=5000) {
        statusMessage.textContent = message;
        statusMessage.className = `status-message ${type}`;
        
        if (type === 'success' && duration > 0) {
            setTimeout(() => {
                statusMessage.className = 'status-message hidden';
            }, duration);
        }
    }

    // --- CAPTURA ÚNICA ---
    captureBtn.addEventListener('click', async () => {
        captureBtn.disabled = true;
        const originalContent = captureBtn.innerHTML;
        captureBtn.innerHTML = '<i data-lucide="loader" class="spin"></i> Capturando...';
        lucide.createIcons();

        try {
            const response = await fetch('/api/capture', { method: 'POST' });
            const data = await response.json();
            if (response.ok) {
                showStatus(data.message, 'success');
            } else {
                showStatus(data.message || 'Error al capturar', 'error');
            }
        } catch (error) {
            showStatus('Error de conexión con el escáner', 'error');
        } finally {
            captureBtn.disabled = false;
            captureBtn.innerHTML = originalContent;
            lucide.createIcons();
        }
    });

    const reloadBtn = document.getElementById('reloadBtn');
    if (reloadBtn) {
        reloadBtn.addEventListener('click', async () => {
            reloadBtn.disabled = true;
            const originalContent = reloadBtn.innerHTML;
            reloadBtn.innerHTML = '<i data-lucide="loader" class="spin"></i> Recargando...';
            lucide.createIcons();

            try {
                const response = await fetch('/api/reload_camera', { 
                    method: 'POST',
                    headers: {'Content-Type': 'application/json'},
                    body: JSON.stringify({hard: true})
                });
                const data = await response.json();
                if (response.ok) {
                    showStatus(data.message, 'success');
                } else {
                    showStatus(data.message || 'Error al recargar la cámara', 'error');
                }
            } catch (error) {
                showStatus('Error de conexión con el servidor', 'error');
            } finally {
                reloadBtn.disabled = false;
                reloadBtn.innerHTML = originalContent;
                lucide.createIcons();
            }
        });
    }

    // --- MULTI CAPTURA ---
    const mStartBtn = document.getElementById('multiStartBtn');
    const mAddBtn = document.getElementById('multiAddBtn');
    const mUndoBtn = document.getElementById('multiUndoBtn');
    const mFinishBtn = document.getElementById('multiFinishBtn');
    const mDiscardBtn = document.getElementById('multiDiscardBtn');
    
    const mInitControls = document.getElementById('multiInitControls');
    const mActiveControls = document.getElementById('multiActiveControls');
    const mCountBadge = document.getElementById('multiCountBadge');
    const mHint = document.getElementById('multiHint');
    
    const pContainer = document.getElementById('multiProgressContainer');
    const pFill = document.getElementById('multiProgressFill');
    const pText = document.getElementById('multiProgressText');

    let isMultiActive = false;
    let checkStatusInterval = null;

    function setMultiState(active, count=0, hint="Gira la cámara ~20° entre fotos.") {
        isMultiActive = active;
        if (active) {
            mInitControls.classList.add('hidden');
            mActiveControls.classList.remove('hidden');
            mCountBadge.textContent = `${count} vistas`;
            mHint.textContent = hint;
            mFinishBtn.disabled = count < 2;
            mUndoBtn.disabled = count === 0;
        } else {
            mInitControls.classList.remove('hidden');
            mActiveControls.classList.add('hidden');
            pContainer.classList.add('hidden');
            if (checkStatusInterval) clearInterval(checkStatusInterval);
        }
    }

    async function checkMultiStatus() {
        try {
            const res = await fetch('/api/multi/status');
            const data = await res.json();
            if (data.active && data.session) {
                if (data.session.state === 'capturing') {
                    const count = data.session.count;
                    const hint = count > 0 ? data.session.views[count-1].hint : "Apunta al objeto y añade la primera foto.";
                    setMultiState(true, count, hint);
                } else if (data.session.state === 'processing') {
                    // Muestra progreso
                    mAddBtn.disabled = true;
                    mFinishBtn.disabled = true;
                    mUndoBtn.disabled = true;
                    mDiscardBtn.disabled = true;
                    pContainer.classList.remove('hidden');
                    pFill.style.width = `${data.session.progress * 100}%`;
                    pText.textContent = data.session.stage;
                } else if (data.session.state === 'done') {
                    showStatus(`¡Mapeo procesado con éxito! Guardado en: ${data.session.dir}`, 'success', 10000);
                    setMultiState(false);
                    mAddBtn.disabled = false;
                    mFinishBtn.disabled = false;
                    mUndoBtn.disabled = false;
                    mDiscardBtn.disabled = false;
                }
            } else {
                setMultiState(false);
            }
        } catch (e) {
            console.error("Status check failed", e);
        }
    }

    // Inicializar comprobando si ya había sesión abierta
    checkMultiStatus();

    mStartBtn.addEventListener('click', async () => {
        mStartBtn.disabled = true;
        try {
            const res = await fetch('/api/multi/start', { method: 'POST' });
            if (res.ok) {
                setMultiState(true, 0, "Apunta al objeto y añade la primera foto.");
                checkStatusInterval = setInterval(checkMultiStatus, 1000);
            } else {
                const data = await res.json();
                showStatus(data.message, 'error');
            }
        } catch (e) {
            showStatus("Error iniciando sesión", "error");
        } finally {
            mStartBtn.disabled = false;
        }
    });

    mAddBtn.addEventListener('click', async () => {
        mAddBtn.disabled = true;
        const orig = mAddBtn.innerHTML;
        mAddBtn.innerHTML = '<i data-lucide="loader" class="spin"></i> Añadiendo...';
        lucide.createIcons();

        try {
            const res = await fetch('/api/multi/add', { method: 'POST' });
            const data = await res.json();
            if (res.ok) {
                showStatus("Vista añadida.", "success", 2000);
                await checkMultiStatus();
            } else {
                showStatus(data.message, "error");
            }
        } catch (e) {
            showStatus("Error añadiendo vista", "error");
        } finally {
            mAddBtn.disabled = false;
            mAddBtn.innerHTML = orig;
            lucide.createIcons();
        }
    });

    mUndoBtn.addEventListener('click', async () => {
        try {
            const res = await fetch('/api/multi/undo', { method: 'POST' });
            if (res.ok) checkMultiStatus();
        } catch (e) {}
    });

    mDiscardBtn.addEventListener('click', async () => {
        if (!confirm("¿Seguro que quieres descartar esta sesión?")) return;
        try {
            const res = await fetch('/api/multi/discard', { method: 'POST' });
            if (res.ok) {
                setMultiState(false);
                showStatus("Sesión descartada.", "success");
            }
        } catch (e) {}
    });

    mFinishBtn.addEventListener('click', async () => {
        try {
            const res = await fetch('/api/multi/finish', { method: 'POST' });
            if (res.ok) {
                // El polling de status actualizará la UI a 'processing'
                checkMultiStatus();
            } else {
                const data = await res.json();
                showStatus(data.message, "error");
            }
        } catch (e) {
            showStatus("Error al finalizar", "error");
        }
    });

    // Check camera status periodically to update the sidebar indicator
    setInterval(async () => {
        try {
            const res = await fetch('/api/camera/status');
            const data = await res.json();
            const indicator = document.querySelector('.status-indicator');
            const statusText = document.querySelector('.system-status span');
            if (data.streaming) {
                indicator.className = 'status-indicator online';
                statusText.textContent = 'Cámara Conectada';
            } else {
                indicator.className = 'status-indicator error';
                indicator.style.background = 'var(--error)';
                indicator.style.boxShadow = '0 0 10px var(--error)';
                statusText.textContent = data.message || 'Desconectada';
            }
        } catch(e) {}
    }, 2000);
});
