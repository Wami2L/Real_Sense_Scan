@echo off
chcp 65001 >nul
echo ==============================================================================
echo   INSTALADOR DE DEPENDENCIAS · RealSenseScans (Dual Live 3D)
echo ==============================================================================
echo.

:: 1. Comprobar Python
python --version >nul 2>&1
if %errorlevel% neq 0 (
    echo [ERROR] No se detectó Python en el sistema.
    echo Por favor instala Python 3.10, 3.11 o 3.12 (64 bits) y marca "Add Python to PATH".
    echo Descarga: https://www.python.org/downloads/
    pause
    exit /b 1
)

echo [1/3] Verificando entorno virtual (.venv)...
if not exist ".venv" (
    echo Creando entorno virtual aislado en .venv...
    python -m venv .venv
    if %errorlevel% neq 0 (
        echo [ERROR] Falló la creación del entorno virtual.
        pause
        exit /b 1
    )
) else (
    echo Entorno virtual existente detectado.
)

echo [2/3] Activando entorno virtual...
call .venv\Scripts\activate.bat

echo [3/3] Instalando librerías desde requirements.txt...
python -m pip install --upgrade pip
pip install -r requirements.txt

if %errorlevel% neq 0 (
    echo.
    echo [AVISO] Ocurrió un error al instalar algunas librerías.
    echo Si pyrealsense2 falló, asegúrate de estar en Python de 64 bits.
    pause
    exit /b 1
)

echo.
echo ==============================================================================
echo   ¡INSTALACIÓN COMPLETADA CON ÉXITO!
echo   Ya puedes ejecutar el proyecto con: iniciar_sistema.bat
echo ==============================================================================
echo.
pause
