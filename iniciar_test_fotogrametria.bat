@echo off
title Servidor de Prueba - Fotogrametria RGB-D (Puerto 5005)
color 0B
echo ======================================================================
echo   INICIANDO ENTORNO DE PRUEBA: FOTOGRAMETRIA RGB-D EN TIEMPO REAL
echo   Puerto: 5005
echo ======================================================================
echo.

:: Moverse al directorio del script
cd /d "%~dp0"

:: Ejecutar el lanzador en Python
python ejecutar_servidor_test.py

pause
