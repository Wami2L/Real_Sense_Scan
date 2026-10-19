@echo off
chcp 65001 >nul
echo ==============================================================================
echo   INICIANDO RealSenseScans (Dual Live 3D + Estudio de Alineación)
echo ==============================================================================
echo.

if exist ".venv\Scripts\activate.bat" (
    echo Activando entorno virtual .venv...
    call .venv\Scripts\activate.bat
)

echo [1/3] Iniciando Servidor de Streaming de Cámaras (Puerto 5002)...
start "RealSense Live 3D (Puerto 5002)" cmd /k "python -u app.py"

:: Esperar 2 segundos para que el puerto 5002 inicie
timeout /t 2 /nobreak >nul

echo [2/3] Iniciando Estudio de Capturas y Alineación Virtual (Puerto 5005)...
start "Estudio de Alineación (Puerto 5005)" cmd /k "python -u test_fotogrametria/servidor_test.py"

:: Esperar 1 segundo
timeout /t 1 /nobreak >nul

echo [3/3] Abriendo navegador en http://localhost:5005...
start http://localhost:5005

echo.
echo ==============================================================================
echo   Sistema iniciado en segundo plano.
echo   - Entorno Dividido: http://localhost:5005
echo   - Servidor Base 3D: http://localhost:5002
echo ==============================================================================
echo.
