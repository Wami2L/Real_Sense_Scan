#!/usr/bin/env bash
# ==============================================================================
# INSTALADOR DE DEPENDENCIAS · RealSenseScans (Linux / macOS)
# ==============================================================================

set -e

echo "=============================================================================="
echo "  INSTALADOR DE DEPENDENCIAS · RealSenseScans"
echo "=============================================================================="

# 1. Comprobar Python 3
if ! command -v python3 &> /dev/null; then
    echo "[ERROR] python3 no está instalado. Instálalo con: sudo apt install python3 python3-pip python3-venv"
    exit 1
fi

echo "[1/3] Creando entorno virtual .venv..."
if [ ! -d ".venv" ]; then
    python3 -m venv .venv
fi

echo "[2/3] Activando entorno virtual..."
source .venv/bin/activate

echo "[3/3] Instalando dependencias desde requirements.txt..."
pip install --upgrade pip
pip install -r requirements.txt

echo "=============================================================================="
echo "  ¡Instalación completada con éxito!"
echo "  Para ejecutar: bash iniciar_sistema.sh"
echo "=============================================================================="
