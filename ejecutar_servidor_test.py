"""
Lanzador del Servidor de Prueba de Fotogrametría RGB-D.
Ejecuta el servidor en el puerto 5005 y abre el navegador automáticamente.

Uso:
    python ejecutar_servidor_test.py
"""

import os
import sys
import time
import webbrowser
import subprocess

BASE_DIR = os.path.dirname(os.path.abspath(__file__))
if BASE_DIR not in sys.path:
    sys.path.insert(0, BASE_DIR)

def is_port_in_use(port):
    import socket
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
        return s.connect_ex(('127.0.0.1', port)) == 0

def kill_port_owner(port):
    try:
        out = subprocess.check_output(f'netstat -ano | findstr :{port} | findstr LISTENING', shell=True).decode()
        for line in out.strip().split('\n'):
            parts = line.strip().split()
            if len(parts) >= 5:
                pid = parts[-1]
                subprocess.run(f'taskkill /F /PID {pid}', shell=True, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    except Exception:
        pass

def main():
    print("=" * 70)
    print("  LANZADOR: SERVIDOR DE PRUEBA DE FOTOGRAMETRÍA RGB-D")
    print("  Puerto Objetivo: http://localhost:5005")
    print("=" * 70)

    # 1. Liberar puerto 5005 si quedó ocupado
    if is_port_in_use(5005):
        print("[Aviso] Puerto 5005 ocupado. Liberando...")
        kill_port_owner(5005)
        time.sleep(1.0)

    # 2. Comprobar si el servidor Live 3D (puerto 5002) está activo
    if not is_port_in_use(5002):
        print("\n[Aviso] El servidor principal (puerto 5002) no está activo.")
        print("[Iniciando] Arrancando 'app.py' en segundo plano...")
        subprocess.Popen([sys.executable, "-u", "app.py"], cwd=BASE_DIR)
        print("  Esperando 3 segundos a que inicien las cámaras RealSense...")
        time.sleep(3.5)
    else:
        print("[OK] Servidor RealSense Live 3D detectado activo en el puerto 5002.")

    # 3. Abrir navegador
    print("\n[Navegador] Abriendo http://localhost:5005 ...")
    webbrowser.open("http://localhost:5005")

    # 4. Ejecutar servidor de prueba
    print("[Servidor] Iniciando servidor de prueba en http://localhost:5005 ...\n")
    from test_fotogrametria.servidor_test import create_app, PORT
    from aiohttp import web

    app = create_app()
    web.run_app(app, host="0.0.0.0", port=PORT)

if __name__ == "__main__":
    main()
