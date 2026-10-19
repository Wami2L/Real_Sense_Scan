"""
RealSenseScans 3D Studio (Plataforma Unificada)
==============================================
Combina el streaming WebGL interactivo en tiempo real (GPU/Three.js) con
la captura estática de alta resolución y el mapeo 360° multi-vista (Open3D).

Uso común:
    python app.py                  # Inicia con cámara RealSense en http://localhost:5002
    python app.py --synthetic      # Modo simulación animada 3D (sin necesidad de cámara)
    python app.py --port 5002      # Especificar puerto alternativo
    python app.py --host 0.0.0.0   # Permitir acceso desde otros PCs o tablets en la red local

Nota: La versión anterior de Flask está respaldada en 'app_legacy_flask.py'.
"""

import sys
from live3d.server import main

if __name__ == "__main__":
    main()
