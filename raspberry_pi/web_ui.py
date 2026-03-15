#!/usr/bin/env python3
# web boshqaruv paneli - flask + socketio

import os
import sys
import json
import glob as globmod
import time
import signal
import argparse
import threading

from flask import Flask, render_template, request, jsonify
from flask_socketio import SocketIO, emit

from config import SERIAL_PORT, SERIAL_BAUD, SERIAL_TIMEOUT, BASE_SPEED
from communicator import Communicator
from map_manager import MapManager
from navigator import Navigator

app = Flask(__name__)
app.config['SECRET_KEY'] = 'avtonom-mashina-2026'
socketio = SocketIO(app, cors_allowed_origins="*", async_mode='threading')

# Global State
state = {
    "status": "idle",
    "map_loaded": False,
    "map_name": "",
    "path_found": False,
    "connected": False,
    "simulate": False,
    "waypoints_total": 0,
    "waypoints_done": 0,
    "logs": [],
}

comm = None
map_mgr = MapManager()
navigator = None
nav_thread = None
telemetry_thread = None
running = True

MAPS_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'maps')


def add_log(msg, level="info"):
    entry = {
        "time": time.strftime("%H:%M:%S"),
        "msg": msg,
        "level": level,
    }
    state["logs"].append(entry)
    if len(state["logs"]) > 200:
        state["logs"] = state["logs"][-200:]
    socketio.emit('log', entry)


# ---- Routes ----

@app.route('/')
def index():
    return render_template('index.html')


@app.route('/api/maps', methods=['GET'])
def list_maps():
    maps = []
    for f in sorted(globmod.glob(os.path.join(MAPS_DIR, '*.json'))):
        try:
            with open(f, 'r') as fh:
                data = json.load(fh)
            maps.append({
                "filename": os.path.basename(f),
                "name": data.get("name", os.path.basename(f)),
                "size": f"{len(data.get('map', []))}x{len(data.get('map', [[]])[0])}",
            })
        except Exception:
            pass
    return jsonify(maps)


@app.route('/api/map/upload', methods=['POST'])
def upload_map():
    if 'file' not in request.files:
        return jsonify({"error": "Fayl tanlanmadi"}), 400
    file = request.files['file']
    if file.filename == '':
        return jsonify({"error": "Fayl nomi bo'sh"}), 400
    if not file.filename.endswith('.json'):
        return jsonify({"error": "Faqat .json fayllar"}), 400
    filepath = os.path.join(MAPS_DIR, file.filename)
    file.save(filepath)
    try:
        with open(filepath, 'r') as f:
            json.load(f)
        add_log(f"Xarita yuklandi: {file.filename}", "success")
        return jsonify({"success": True, "filename": file.filename})
    except json.JSONDecodeError:
        os.remove(filepath)
        return jsonify({"error": "Noto'g'ri JSON format"}), 400


# ---- Socket.IO Events ----

@socketio.on('connect')
def handle_connect():
    emit('state', get_full_state())
    emit('log_history', state["logs"][-50:])


@socketio.on('load_map')
def handle_load_map(data):
    global map_mgr
    filename = data.get('filename', '')
    filepath = os.path.join(MAPS_DIR, filename)

    if not os.path.exists(filepath):
        emit('error', {"msg": f"Xarita topilmadi: {filename}"})
        return

    map_mgr = MapManager()
    if map_mgr.load_from_file(filepath):
        state["map_loaded"] = True
        state["map_name"] = filename
        state["path_found"] = False
        add_log(f"Xarita yuklandi: {filename}", "success")
        emit('map_data', get_map_state())
        emit('state', get_full_state())
    else:
        emit('error', {"msg": "Xarita yuklanmadi"})


@socketio.on('find_path')
def handle_find_path():
    if not state["map_loaded"]:
        emit('error', {"msg": "Avval xarita yuklang"})
        return

    path = map_mgr.find_path()
    if path:
        waypoints = map_mgr.generate_waypoints()
        state["path_found"] = True
        state["waypoints_total"] = len(waypoints)
        state["waypoints_done"] = 0
        add_log(f"Yo'l topildi: {len(path)} qadam, {len(waypoints)} waypoint", "success")
        emit('map_data', get_map_state())
        emit('state', get_full_state())
    else:
        state["path_found"] = False
        add_log("Yo'l topilmadi!", "error")
        emit('map_data', get_map_state())
        emit('state', get_full_state())


@socketio.on('start_navigation')
def handle_start_nav():
    global nav_thread, navigator
    if state["status"] == "navigating":
        add_log("Navigatsiya allaqachon ishlayapti", "warn")
        return
    if not state["path_found"]:
        add_log("Avval yo'l toping", "warn")
        return

    navigator = Navigator(comm)
    state["status"] = "navigating"
    state["waypoints_done"] = 0
    socketio.emit('state', get_full_state())
    add_log("Navigatsiya boshlandi!", "success")

    nav_thread = threading.Thread(target=run_navigation, daemon=True)
    nav_thread.start()


@socketio.on('stop_navigation')
def handle_stop_nav():
    global navigator
    if navigator:
        navigator.stop()
    state["status"] = "idle"
    socketio.emit('state', get_full_state())
    add_log("Navigatsiya to'xtatildi", "warn")


@socketio.on('emergency_stop')
def handle_emergency():
    global navigator
    if navigator:
        navigator.emergency_stop()
    if comm:
        comm.send_stop()
    state["status"] = "idle"
    socketio.emit('state', get_full_state())
    add_log("FAVQULODDA TO'XTATISH!", "error")


@socketio.on('manual_control')
def handle_manual(data):
    if state["status"] == "navigating":
        return
    state["status"] = "manual"
    speed = int(data.get('speed', 0))
    angle = int(data.get('angle', 0))
    if comm:
        comm.send_drive(speed, angle)
    socketio.emit('state', get_full_state())


@socketio.on('manual_stop')
def handle_manual_stop():
    if comm:
        comm.send_stop()
    if state["status"] == "manual":
        state["status"] = "idle"
    socketio.emit('state', get_full_state())


@socketio.on('reset_encoders')
def handle_reset_enc():
    if comm:
        comm.reset_encoders()
    add_log("Encoderlar nollandi", "info")


# ---- Background Tasks ----

def run_navigation():
    global navigator
    try:
        waypoints = map_mgr.waypoints
        original_nav = navigator.navigate_waypoints

        # Waypoint progress tracking wrapper
        def tracked_navigate(wps):
            navigator.running = True
            navigator.current_waypoint_idx = 0
            total = len(wps)

            for idx, wp in enumerate(wps):
                if not navigator.running:
                    return False
                navigator.current_waypoint_idx = idx
                state["waypoints_done"] = idx
                socketio.emit('state', get_full_state())
                socketio.emit('nav_progress', {
                    "current": idx,
                    "total": total,
                    "waypoint": wp,
                })
                add_log(f"Waypoint {idx+1}/{total}: {wp['heading']}°, {wp['distance']}m")

                if not navigator._turn_to_heading(wp["heading"]):
                    return False
                if not navigator._drive_distance(wp["distance"], wp["heading"]):
                    return False

            return True

        success = tracked_navigate(waypoints)

        if success:
            state["status"] = "idle"
            state["waypoints_done"] = state["waypoints_total"]
            add_log("MANZILGA YETILDI!", "success")
        else:
            state["status"] = "idle"
            add_log("Navigatsiya bekor qilindi", "warn")
    except Exception as e:
        state["status"] = "error"
        add_log(f"Navigatsiya xatosi: {str(e)}", "error")
    finally:
        if comm:
            comm.send_stop()
        socketio.emit('state', get_full_state())


def telemetry_loop():
    while running:
        if comm:
            try:
                t = comm.get_telemetry()
                telemetry_data = {
                    "encoder_left": t.encoder_left,
                    "encoder_right": t.encoder_right,
                    "heading": round(t.heading, 1),
                    "rpm_left": round(t.rpm_left, 1),
                    "rpm_right": round(t.rpm_right, 1),
                    "distance": round(t.distance, 2),
                    "dist_old_ong": round(t.dist_old_ong, 2),
                    "dist_old_chap": round(t.dist_old_chap, 2),
                    "dist_ong": round(t.dist_ong, 2),
                    "dist_chap": round(t.dist_chap, 2),
                    "dist_orqa": round(t.dist_orqa, 2),
                    "obstacle": t.obstacle_warning,
                }
                socketio.emit('telemetry', telemetry_data)

                warnings = comm.get_warnings()
                for w in warnings:
                    add_log(f"ESP32: {w}", "warn")
            except Exception:
                pass
        time.sleep(0.15)


# ---- Helpers ----

def get_full_state():
    return {
        "status": state["status"],
        "map_loaded": state["map_loaded"],
        "map_name": state["map_name"],
        "path_found": state["path_found"],
        "connected": state["connected"],
        "simulate": state["simulate"],
        "waypoints_total": state["waypoints_total"],
        "waypoints_done": state["waypoints_done"],
    }


def get_map_state():
    result = {
        "grid": map_mgr.grid,
        "rows": map_mgr.rows,
        "cols": map_mgr.cols,
        "start": list(map_mgr.start),
        "end": list(map_mgr.end),
        "path": [list(p) for p in map_mgr.path],
        "waypoints": map_mgr.waypoints,
    }
    return result


# ---- Main ----

def main():
    global comm, running, state, telemetry_thread

    parser = argparse.ArgumentParser(description="Avtonom Mashina - Web UI")
    parser.add_argument("--port", default=SERIAL_PORT, help="Serial port")
    parser.add_argument("--baud", type=int, default=SERIAL_BAUD, help="Baud rate")
    parser.add_argument("--simulate", action="store_true", help="Simulyatsiya rejimi")
    parser.add_argument("--host", default="0.0.0.0", help="Web server host")
    parser.add_argument("--web-port", type=int, default=5000, help="Web server port")
    args = parser.parse_args()

    state["simulate"] = args.simulate

    if args.simulate:
        from main import SimulatedCommunicator
        comm = SimulatedCommunicator()
        state["connected"] = True
        add_log("Simulyatsiya rejimi faol", "info")
    else:
        comm = Communicator(args.port, args.baud, SERIAL_TIMEOUT)
        if comm.connect():
            state["connected"] = True
            add_log(f"ESP32 ga ulandi: {args.port}", "success")
        else:
            state["connected"] = False
            add_log("ESP32 ga ulanib bo'lmadi!", "error")

    telemetry_thread = threading.Thread(target=telemetry_loop, daemon=True)
    telemetry_thread.start()

    def shutdown(sig, frame):
        global running
        running = False
        if comm and not args.simulate:
            comm.send_stop()
            comm.disconnect()
        sys.exit(0)

    signal.signal(signal.SIGINT, shutdown)

    print(f"\n  Web UI: http://localhost:{args.web_port}")
    print(f"  Rejim: {'Simulyatsiya' if args.simulate else 'Haqiqiy'}\n")

    socketio.run(app, host=args.host, port=args.web_port,
                 allow_unsafe_werkzeug=True, debug=False)


if __name__ == "__main__":
    main()
