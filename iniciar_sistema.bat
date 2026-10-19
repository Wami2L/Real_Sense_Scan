@echo off
chcp 65001 >nul
echo ==============================================================================
echo   INICIANDO RealSenseScans (Live 3D + Escáner de objetos + Calibración + Unity)
echo ==============================================================================
echo.

if exist ".venv\Scripts\activate.bat" (
    echo Activando entorno virtual .venv...
    call .venv\Scripts\activate.bat
)

echo [1/4] Servidor de cámaras y escáner de objetos (Puerto 5002)...
start "RealSense Live 3D (Puerto 5002)" cmd /k "python -u app.py"

:: Esperar a que el puerto 5002 inicie
timeout /t 2 /nobreak >nul

echo [2/4] Estudio de calibración de las dos cámaras (Puerto 5005)...
start "Calibración (Puerto 5005)" cmd /k "python -u test_fotogrametria/servidor_test.py"

echo [3/4] Enlace privado con Unity (Puerto 5010, solo este PC)...
start "Enlace Unity (Puerto 5010)" cmd /k "python -u -m live3d.unity_link"

timeout /t 1 /nobreak >nul

echo [4/4] Abriendo el escáner de objetos en http://localhost:5002/objetos ...
start http://localhost:5002/objetos

echo.
echo ==============================================================================
echo   Sistema iniciado en segundo plano.
echo   - Escáner de objetos: http://localhost:5002/objetos
echo   - Visor Live 3D:      http://localhost:5002
echo   - Calibración dual:   http://localhost:5005
echo   - Enlace Unity:       http://127.0.0.1:5010  (VR ScanWorld lo consulta solo)
echo ==============================================================================
echo.
