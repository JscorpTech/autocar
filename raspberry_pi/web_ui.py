#!/usr/bin/env python3
# web control panel - flask + socketio

import os
import sys
import json
import glob as globmod
import time
import signal
import argparse
import threading
from collections import deque

from flask import Flask, render_template, request, jsonify
from flask_socketio import SocketIO, emit
from werkzeug.utils import secure_filename

from logger import setup_logging, get_logger
from config import SERIAL_PORT, SERIAL_BAUD, SERIAL_TIMEOUT, BASE_SPEED
from communicator import Communicator
from map_manager import MapManager
from navigator import Navigator

app = Flask(__name__)
# SECRET_KEY from environment variable; random on each restart if not set
app.config['SECRET_KEY'] = os.environ.get('SECRET_KEY', os.urandom(24).hex())
socketio = SocketIO(app, cors_allowed_origins="*", async_mode='threading')

# Global state
state = {
    "status": "idle",
    "map_loaded": False,
    "map_name": "",
    "path_found": False,
    "connected": False,
    "simulate": False,
    "waypoints_total": 0,
    "waypoints_done": 0,
    "logs": deque(maxlen=200),  # auto-bounded
}

comm = None
map_mgr = MapManager()
navigator = None
nav_thread = None
telemetry_thread = None
running = True

MAPS_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'maps')
os.makedirs(MAPS_DIR, exist_ok=True)


def add_log(msg, level="info"):
    entry = {
        "time": time.strftime("%H:%M:%S"),
        "msg": msg,
        "level": level,
    }
    state["logs"].append(entry)
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
        return jsonify({"error": "No file selected"}), 400
    file = request.files['file']
    if not file.filename:
        return jsonify({"error": "Empty filename"}), 400
    if not file.filename.endswith('.json'):
        return jsonify({"error": "Only .json files allowed"}), 400

    # Path traversal protection
    safe_name = secure_filename(file.filename)
    if not safe_name:
        return jsonify({"error": "Invalid filename"}), 400

    filepath = os.path.join(MAPS_DIR, safe_name)
    file.save(filepath)

    try:
        with open(filepath, 'r') as f:
            data = json.load(f)

        # Semantic validation
        for field in ("map", "start", "end"):
            if field not in data:
                os.remove(filepath)
                return jsonify({"error": f"'{field}' field missing"}), 400

        grid = data["map"]
        if not isinstance(grid, list) or len(grid) == 0 or len(grid) > 1000:
            os.remove(filepath)
            return jsonify({"error": "Invalid map size (max 1000x1000)"}), 400

        add_log(f"Map uploaded: {safe_name}", "success")
        return jsonify({"success": True, "filename": safe_name})

    except json.JSONDecodeError:
        if os.path.exists(filepath):
            os.remove(filepath)
        return jsonify({"error": "Invalid JSON format"}), 400
    except Exception as e:
        if os.path.exists(filepath):
            os.remove(filepath)
        return jsonify({"error": str(e)}), 500


# ---- Socket.IO Events ----

@socketio.on('connect')
def handle_connect():
    emit('state', get_full_state())
    emit('log_history', list(state["logs"])[-50:])


@socketio.on('load_map')
def handle_load_map(data):
    global map_mgr
    filename = secure_filename(data.get('filename', ''))
    if not filename:
        emit('error', {"msg": "Invalid filename"})
        return

    filepath = os.path.join(MAPS_DIR, filename)
    if not os.path.exists(filepath):
        emit('error', {"msg": f"Map not found: {filename}"})
        return

    map_mgr = MapManager()
    if map_mgr.load_from_file(filepath):
        state["map_loaded"] = True
        state["map_name"] = filename
        state["path_found"] = False
        add_log(f"Map loaded: {filename}", "success")
        emit('map_data', get_map_state())
        emit('state', get_full_state())
    else:
        emit('error', {"msg": "Failed to load map (format error)"})


@socketio.on('find_path')
def handle_find_path():
    if not state["map_loaded"]:
        emit('error', {"msg": "Load a map first"})
        return

    path = map_mgr.find_path()
    if path:
        waypoints = map_mgr.generate_waypoints()
        state["path_found"] = True
        state["waypoints_total"] = len(waypoints)
        state["waypoints_done"] = 0
        add_log(f"Path found: {len(path)} steps, {len(waypoints)} waypoints", "success")
        emit('map_data', get_map_state())
        emit('state', get_full_state())
    else:
        state["path_found"] = False
        add_log("No path found!", "error")
        emit('map_data', get_map_state())
        emit('state', get_full_state())


@socketio.on('start_navigation')
def handle_start_nav():
    global nav_thread, navigator

    if state["status"] == "navigating":
        add_log("Navigation already running", "warn")
        return
    if not state["path_found"]:
        add_log("Find a path first", "warn")
        return
    if not state["connected"]:
        add_log("ESP32 not connected! Connect first.", "error")
        return

    c = comm
    if not c:
        add_log("No communicator object!", "error")
        return

    navigator = Navigator(c)
    state["status"] = "navigating"
    state["waypoints_done"] = 0
    socketio.emit('state', get_full_state())
    add_log("Navigation started!", "success")

    nav_thread = threading.Thread(target=run_navigation, daemon=True)
    nav_thread.start()


@socketio.on('stop_navigation')
def handle_stop_nav():
    global navigator
    if navigator:
        navigator.stop()
    state["status"] = "idle"
    socketio.emit('state', get_full_state())
    add_log("Navigation stopped", "warn")


@socketio.on('emergency_stop')
def handle_emergency():
    global navigator
    if navigator:
        navigator.emergency_stop()
    c = comm
    if c:
        c.send_stop()
    state["status"] = "idle"
    socketio.emit('state', get_full_state())
    add_log("EMERGENCY STOP!", "error")


@socketio.on('manual_control')
def handle_manual(data):
    _handle_manual_input(data)


@socketio.on('manual_drive')
def handle_manual_drive(data):
    _handle_manual_input(data)


def _handle_manual_input(data):
    if state["status"] == "navigating":
        return
    state["status"] = "manual"
    speed = max(-255, min(255, int(data.get('speed', 0))))
    angle = max(-20, min(20, int(data.get('angle', 0))))
    c = comm
    if c:
        c.send_drive(speed, angle)
    socketio.emit('state', get_full_state())


@socketio.on('manual_stop')
def handle_manual_stop():
    c = comm
    if c:
        c.send_stop()
    if state["status"] == "manual":
        state["status"] = "idle"
    socketio.emit('state', get_full_state())


@socketio.on('reset_encoders')
def handle_reset_enc():
    c = comm
    if c:
        c.reset_encoders()
    add_log("Encoders reset", "info")


# ---- Background Tasks ----

def run_navigation():
    global navigator
    try:
        waypoints = map_mgr.waypoints
        total = len(waypoints)
        navigator.running = True
        navigator.current_waypoint_idx = 0

        for idx, wp in enumerate(waypoints):
            if not navigator.running:
                break

            navigator.current_waypoint_idx = idx
            state["waypoints_done"] = idx
            socketio.emit('state', get_full_state())
            socketio.emit('nav_progress', {
                "current": idx,
                "total": total,
                "waypoint": wp,
            })
            add_log(f"Waypoint {idx + 1}/{total}: {wp['heading']}deg, {wp['distance']}m")

            if not navigator._turn_to_heading(wp["heading"]):
                add_log(f"Turn failed ({wp['heading']}deg)", "error")
                state["status"] = "idle"
                socketio.emit('state', get_full_state())
                return

            if not navigator._drive_distance(wp["distance"], wp["heading"]):
                add_log(f"Drive failed ({wp['distance']}m)", "error")
                state["status"] = "idle"
                socketio.emit('state', get_full_state())
                return

        if navigator.running:
            state["status"] = "idle"
            state["waypoints_done"] = total
            add_log("DESTINATION REACHED!", "success")
        else:
            state["status"] = "idle"
            add_log("Navigation cancelled", "warn")

    except Exception as e:
        state["status"] = "error"
        add_log(f"Navigation error: {str(e)}", "error")
    finally:
        c = comm
        if c:
            c.send_stop()
        socketio.emit('state', get_full_state())


def telemetry_loop():
    while running:
        c = comm  # thread-safe snapshot
        if c:
            try:
                t = c.get_telemetry()
                telemetry_data = {
                    "encoder_left":  t.encoder_left,
                    "encoder_right": t.encoder_right,
                    "heading":       round(t.heading, 1),
                    "rpm_left":      round(t.rpm_left, 1),
                    "rpm_right":     round(t.rpm_right, 1),
                    "distance":      round(t.distance, 2),
                    "dist_front_right": round(t.dist_front_right, 2),
                    "dist_front_left":  round(t.dist_front_left, 2),
                    "dist_right":    round(t.dist_right, 2),
                    "dist_left":     round(t.dist_left, 2),
                    "dist_rear":     round(t.dist_rear, 2),
                    "obstacle":      t.obstacle_warning,
                }
                socketio.emit('telemetry', telemetry_data)

                warnings = c.get_warnings()
                for w in warnings:
                    add_log(f"ESP32: {w}", "warn")
            except Exception:
                pass
        time.sleep(0.05)  # 20Hz


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
    return {
        "grid": map_mgr.grid,
        "rows": map_mgr.rows,
        "cols": map_mgr.cols,
        "start": list(map_mgr.start),
        "end": list(map_mgr.end),
        "path": [list(p) for p in map_mgr.path],
        "waypoints": map_mgr.waypoints,
    }


# ---- Main ----

def main():
    global comm, running, telemetry_thread

    setup_logging()
    parser = argparse.ArgumentParser(description="Autonomous Car - Web UI")
    parser.add_argument("--port", default=SERIAL_PORT, help="Serial port")
    parser.add_argument("--baud", type=int, default=SERIAL_BAUD, help="Baud rate")
    parser.add_argument("--simulate", action="store_true", help="Simulation mode")
    parser.add_argument("--host", default="0.0.0.0", help="Web server host")
    parser.add_argument("--web-port", type=int, default=5000, help="Web server port")
    args = parser.parse_args()

    state["simulate"] = args.simulate

    if args.simulate:
        from main import SimulatedCommunicator
        comm = SimulatedCommunicator()
        state["connected"] = True
        add_log("Simulation mode active", "info")
    else:
        comm = Communicator(args.port, args.baud, SERIAL_TIMEOUT)
        if comm.connect():
            state["connected"] = True
            add_log(f"Connected to ESP32: {args.port}", "success")
        else:
            state["connected"] = False
            add_log("Failed to connect to ESP32!", "error")

    telemetry_thread = threading.Thread(target=telemetry_loop, daemon=True)
    telemetry_thread.start()

    def shutdown(sig, frame):
        global running
        running = False
        c = comm
        if c and not args.simulate:
            try:
                c.send_stop()
                c.disconnect()
            except Exception:
                pass
        sys.exit(0)

    signal.signal(signal.SIGINT, shutdown)

    print(f"\n  Web UI: http://localhost:{args.web_port}")
    print(f"  Mode: {'Simulation' if args.simulate else 'Real'}\n")

    socketio.run(app, host=args.host, port=args.web_port,
                 allow_unsafe_werkzeug=True, debug=False)


if __name__ == "__main__":
    main()
