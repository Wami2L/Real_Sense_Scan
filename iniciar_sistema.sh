#!/usr/bin/env bash
# ==============================================================================
# INICIAR SISTEMA · RealSenseScans (Linux / macOS)
# Live 3D + escáner de objetos (5002), calibración dual (5005), enlace Unity (5010)
# ==============================================================================

if [ -f ".venv/bin/activate" ]; then
    source .venv/bin/activate
fi

echo "Servidor de cámaras y escáner de objetos en el puerto 5002..."
python3 -u app.py &
PID_APP=$!

sleep 2

echo "Estudio de calibración en el puerto 5005..."
python3 -u test_fotogrametria/servidor_test.py &
PID_TEST=$!

echo "Enlace privado con Unity en el puerto 5010 (solo este equipo)..."
python3 -u -m live3d.unity_link &
PID_UNITY=$!

sleep 1

URL=http://localhost:5002/objetos
echo "Abriendo $URL ..."
if command -v xdg-open &> /dev/null; then
    xdg-open "$URL"
elif command -v open &> /dev/null; then
    open "$URL"
fi

wait $PID_APP $PID_TEST $PID_UNITY
