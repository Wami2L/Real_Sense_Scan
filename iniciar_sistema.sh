#!/usr/bin/env bash
# ==============================================================================
# INICIAR SISTEMA · RealSenseScans (Linux / macOS)
# ==============================================================================

if [ -f ".venv/bin/activate" ]; then
    source .venv/bin/activate
fi

echo "Iniciando servidor de streaming en puerto 5002..."
python3 -u app.py &
PID_APP=$!

sleep 2

echo "Iniciando estudio de alineación en puerto 5005..."
python3 -u test_fotogrametria/servidor_test.py &
PID_TEST=$!

sleep 1

echo "Abriendo navegador en http://localhost:5005 ..."
if command -v xdg-open &> /dev/null; then
    xdg-open http://localhost:5005
elif command -v open &> /dev/null; then
    open http://localhost:5005
fi

wait $PID_APP $PID_TEST
