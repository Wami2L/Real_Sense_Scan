# Guía de Empaquetado, Portabilidad y Publicación en GitHub

> **Proyecto:** RealSenseScans (Dual RealSense Live 3D & Estudio de Alineación)  
> **Fecha:** Octubre 2026  
> **Propósito:** Garantizar que el proyecto pueda subirse a GitHub sin errores y descargarse en cualquier otro computador sin problemas de dependencias.

---

## 1. ¿Es Posible Empaquetarlo y Llevarlo a Otro Dispositivo?

**Sí, absolutamente.** El proyecto ha quedado completamente preparado y protegido con:
1. **`.gitignore` estricto:** Evita que se suban a GitHub los gigabytes de escaneos 3D (`Saves/`), fotos temporales o entornos virtuales pesados.
2. **`requirements.txt` universal:** Lista con versiones compatibles para Python 3.10, 3.11 y 3.12 (64 bits).
3. **Instaladores automáticos en 1-Clic:** `instalar_dependencias.bat` (Windows) y `instalar_dependencias.sh` (Linux/macOS).
4. **Lanzadores automáticos en 1-Clic:** `iniciar_sistema.bat` (Windows) e `iniciar_sistema.sh` (Linux/macOS).
5. **Modo Sintético sin Cámaras:** Si ejecutas el proyecto en una laptop sin las cámaras físicas conectadas, el sistema cuenta con `--synthetic` para probar y simular la visualización 3D sin fallar.

---

## 2. Pasos para Subir el Proyecto a GitHub (Por Primera Vez)

### Paso 1: Crear el repositorio en GitHub
1. Inicia sesión en [GitHub](https://github.com).
2. Haz clic en **"New repository"** (Nuevo repositorio).
3. Dale un nombre (por ejemplo: `realsense-dual-scans`).
4. Déjalo como **Público** o **Privado** según prefieras.
5. **IMPORTANTE:** **NO** marques "Add a README file", ni ".gitignore", ni licencia (ya los tenemos creados localmente).
6. Haz clic en **"Create repository"**.

### Paso 2: Subir tu código desde la terminal
Abre una terminal (PowerShell o CMD) en la carpeta raíz del proyecto (`RealSenseScans`):

```bash
# 1. Inicializar el repositorio Git local
git init

# 2. Agregar todos los archivos (el .gitignore protegerá las carpetas pesadas)
git add .

# 3. Crear el primer commit
git commit -m "feat: RealSense Dual Live 3D y Estudio de Alineación Virtual"

# 4. Establecer la rama principal
git branch -M main

# 5. Conectar con tu repositorio de GitHub (reemplaza con tu URL)
git remote add origin https://github.com/TU_USUARIO/realsense-dual-scans.git

# 6. Subir el proyecto
git push -u origin main
```

---

## 3. Cómo Descargar y Ejecutar en OTRA Computadora

Cuando vayas a otro dispositivo (otra PC del laboratorio, tu laptop personal, etc.):

### En Windows (1 Clic)
1. **Clona o descarga el repositorio:**
   ```bash
   git clone https://github.com/TU_USUARIO/realsense-dual-scans.git
   cd realsense-dual-scans
   ```
   *(O simplemente descarga el `.ZIP` desde el botón verde "Code" -> "Download ZIP" en GitHub y extráelo).*
2. **Doble clic en `instalar_dependencias.bat`:**  
   Creará un entorno virtual aislado (`.venv`) e instalará `pyrealsense2`, `open3d`, `opencv`, `aiohttp`, etc., sin alterar el Python global de esa máquina.
3. **Doble clic en `iniciar_sistema.bat`:**  
   Iniciará los servidores en los puertos 5002 y 5005 y abrirá tu navegador automáticamente en `http://localhost:5005`.

### En Linux / macOS
```bash
# 1. Clonar
git clone https://github.com/TU_USUARIO/realsense-dual-scans.git
cd realsense-dual-scans

# 2. Instalar dependencias
bash instalar_dependencias.sh

# 3. Iniciar el sistema
bash iniciar_sistema.sh
```

---

## 4. Requisitos del Nuevo Dispositivo

Para que las librerías compilen e instalen limpiamente en el nuevo equipo:

1. **Versión de Python:**  
   * **Python 3.10, 3.11 o 3.12 (Recomendado 3.11 o 3.12 de 64 bits).**  
   * *Nota:* En el instalador de Python de Windows, siempre marcar la casilla **"Add Python to PATH"**.
2. **Hardware RealSense:**  
   * **Windows:** Los drivers UVC de Intel RealSense D400 son reconocidos automáticamente al conectar el cable USB 3.0.
   * **Linux:** Si se usa Ubuntu/Debian, instalar las reglas udev para permisos USB:
     ```bash
     sudo apt-get install libssl-dev libusb-1.0-0-dev pkg-config libgtk-3-dev
     ```
3. **¿Y si no tengo las cámaras conectadas en ese equipo?**  
   Puedes ejecutar el modo simulación para revisar el visor y las interfaces sin hardware:
   ```bash
   python app.py --synthetic --dual
   ```

---

## 5. Qué SÍ se Sube y Qué NO se Sube a GitHub

| Contenido | ¿Se sube a GitHub? | Razón |
| :--- | :---: | :--- |
| **Código Fuente (`.py`)** | ✅ SÍ | Núcleo de la aplicación |
| **Interfaces Web (`HTML, CSS, JS`)** | ✅ SÍ | Visor Three.js y panel de estudio |
| **Documentación y Planes (`.md`, `.pdf`)** | ✅ SÍ | Guías técnicas y manuales |
| **Scripts Instaladores (`.bat`, `.sh`)** | ✅ SÍ | Permite la instalación automática en 1 clic |
| **Entornos virtuales (`.venv/`)** | ❌ NO | Se crean automáticamente en cada PC con el instalador |
| **Nubes de Puntos y Escaneos (`Saves/`)** | ❌ NO | Pesarían gigabytes y bloquearían el límite de GitHub |
| **Caché y Temporales (`__pycache__/`)** | ❌ NO | Archivos binarios efímeros |
