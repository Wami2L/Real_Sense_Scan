# Respaldo del servidor Flask original antes de la unificación
# Para ejecutar la versión unificada moderna use: python app.py o python -m live3d

import os
import time
import json
import threading
import numpy as np
import cv2
import open3d as o3d
from flask import Flask, render_template, Response, jsonify, request

from camera_manager import CameraManager
from multiscan import MultiScanSession

app = Flask(__name__)

# Instancia global del manager de la cámara
cam = CameraManager()
cam.start()

# Sesión global de multi-captura
multi_session = None
multi_lock = threading.Lock()

def generate_video_stream(kind):
    """Generador para el feed de video (color o mapa de calor)"""
    gen = cam._generation
    seq = 0
    while True:
        # Si la cámara se recarga, hay que esperar a la nueva generación
        if not cam.is_streaming() or gen != cam._generation:
            pass
            
        jpeg, seq, state = cam.wait_jpeg(kind, seq, timeout=0.5)
        
        if jpeg is None:
            # Enviar un placeholder si no hay video
            jpeg = cam.placeholder_jpeg(kind)
            time.sleep(0.1) # No saturar si estamos caídos
            
        yield (b'--frame\r\n'
               b'Content-Type: image/jpeg\r\n\r\n' + jpeg + b'\r\n')

@app.route('/')
def index():
    return render_template('index.html')

@app.route('/video_feed')
def video_feed():
    return Response(generate_video_stream("color"), mimetype='multipart/x-mixed-replace; boundary=frame')

@app.route('/depth_feed')
def depth_feed():
    return Response(generate_video_stream("depth"), mimetype='multipart/x-mixed-replace; boundary=frame')

@app.route('/api/camera/status')
def camera_status():
    return jsonify(cam.status())

@app.route('/api/reload_camera', methods=['POST'])
def reload_camera():
    hard = request.json.get('hard', False) if request.is_json else False
    cam.request_reload(hard=hard)
    return jsonify({"status": "success", "message": "Reiniciando cámara..."})

@app.route('/api/capture', methods=['POST'])
def capture_scan():
    """Captura única (el comportamiento original)"""
    snap = cam.wait_fresh_snapshot(timeout=3.0)
    if snap is None:
        return jsonify({"status": "error", "message": "No se pudo obtener un cuadro de la cámara"}), 500
        
    try:
        color_image = snap["color"]
        depth_image = snap["depth"]
        heat_image = snap["heat"]
        intrinsics = snap["intrinsics"]
        depth_scale = snap["depth_scale"]
        
        o3d_color = o3d.geometry.Image(color_image)
        o3d_heat = o3d.geometry.Image(heat_image)
        o3d_depth = o3d.geometry.Image(depth_image)
        
        pinhole = o3d.camera.PinholeCameraIntrinsic(
            intrinsics["width"], intrinsics["height"], 
            intrinsics["fx"], intrinsics["fy"], 
            intrinsics["ppx"], intrinsics["ppy"]
        )
        
        rgbd_normal = o3d.geometry.RGBDImage.create_from_color_and_depth(
            o3d_color, o3d_depth, depth_scale=1.0/depth_scale, depth_trunc=3.0, convert_rgb_to_intensity=False
        )
        pcd_normal = o3d.geometry.PointCloud.create_from_rgbd_image(rgbd_normal, pinhole)
        
        rgbd_calor = o3d.geometry.RGBDImage.create_from_color_and_depth(
            o3d_heat, o3d_depth, depth_scale=1.0/depth_scale, depth_trunc=3.0, convert_rgb_to_intensity=False
        )
        pcd_calor = o3d.geometry.PointCloud.create_from_rgbd_image(rgbd_calor, pinhole)
        
        date_folder = time.strftime("%Y-%m-%d")
        time_folder = time.strftime("scan_%H%M%S")
        base_dir = os.path.dirname(os.path.abspath(__file__))
        scan_dir = os.path.join(base_dir, "Saves", date_folder, time_folder)
        os.makedirs(scan_dir, exist_ok=True)
        
        o3d.io.write_point_cloud(os.path.join(scan_dir, "nube_puntos.ply"), pcd_normal)
        o3d.io.write_point_cloud(os.path.join(scan_dir, "nube_calor_profundidad.ply"), pcd_calor)
        
        cv2.imwrite(os.path.join(scan_dir, "captura_color.jpg"), cv2.cvtColor(color_image, cv2.COLOR_RGB2BGR))
        cv2.imwrite(os.path.join(scan_dir, "mapa_profundidad.jpg"), cv2.cvtColor(heat_image, cv2.COLOR_RGB2BGR))
        
        metadata = {
            "intrinsics": intrinsics,
            "depth_scale": depth_scale
        }
        with open(os.path.join(scan_dir, "metadata_camara.json"), 'w') as f:
            json.dump(metadata, f, indent=4)
            
        return jsonify({
            "status": "success", 
            "message": f"Escaneo dual guardado en Saves/{date_folder}/{time_folder}/",
            "filename": time_folder
        })
        
    except Exception as e:
        import traceback
        traceback.print_exc()
        return jsonify({"status": "error", "message": str(e)}), 500

# ----------------- MULTI-CAPTURA -----------------

@app.route('/api/multi/start', methods=['POST'])
def multi_start():
    global multi_session
    with multi_lock:
        if multi_session and multi_session.state == "capturing":
            multi_session.discard()
        base_dir = os.path.dirname(os.path.abspath(__file__))
        multi_session = MultiScanSession(base_dir)
        return jsonify({"status": "success", "session_id": multi_session.id})

@app.route('/api/multi/add', methods=['POST'])
def multi_add():
    global multi_session
    snap = cam.wait_fresh_snapshot(timeout=3.0)
    if snap is None:
        return jsonify({"status": "error", "message": "Cámara no lista"}), 500
        
    with multi_lock:
        if not multi_session or multi_session.state != "capturing":
            return jsonify({"status": "error", "message": "No hay sesión multi-captura activa"}), 400
        try:
            res = multi_session.add_view(snap)
            return jsonify({"status": "success", "view": res})
        except Exception as e:
            import traceback
            traceback.print_exc()
            return jsonify({"status": "error", "message": str(e)}), 500

@app.route('/api/multi/undo', methods=['POST'])
def multi_undo():
    global multi_session
    with multi_lock:
        if not multi_session: return jsonify({"status": "error", "message": "Sin sesión"}), 400
        try:
            multi_session.undo()
            return jsonify({"status": "success"})
        except Exception as e:
            return jsonify({"status": "error", "message": str(e)}), 500

@app.route('/api/multi/discard', methods=['POST'])
def multi_discard():
    global multi_session
    with multi_lock:
        if multi_session:
            multi_session.discard()
            multi_session = None
        return jsonify({"status": "success"})

@app.route('/api/multi/finish', methods=['POST'])
def multi_finish():
    global multi_session
    with multi_lock:
        if not multi_session: return jsonify({"status": "error", "message": "Sin sesión"}), 400
        try:
            multi_session.finish()
            return jsonify({"status": "success"})
        except Exception as e:
            return jsonify({"status": "error", "message": str(e)}), 500

@app.route('/api/multi/status')
def multi_status():
    global multi_session
    if not multi_session:
        return jsonify({"active": False})
    return jsonify({"active": True, "session": multi_session.summary(with_thumbs=True)})

if __name__ == '__main__':
    try:
        print("Iniciando servidor Flask con pyrealsense2...")
        app.run(host='127.0.0.1', port=5001, debug=False, threaded=True)
    finally:
        cam.shutdown()
