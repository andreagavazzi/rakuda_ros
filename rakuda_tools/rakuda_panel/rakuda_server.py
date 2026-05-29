#!/usr/bin/env python3
"""
Rakuda Control Panel — Backend
Flask + SocketIO per controllo remoto via browser
"""

import subprocess
import threading
import os
from flask import Flask, send_from_directory

BASE_DIR = os.path.dirname(os.path.abspath(__file__))
from flask_socketio import SocketIO, emit

app = Flask(__name__)
app.config["SECRET_KEY"] = "rakuda-secret"
socketio = SocketIO(app, cors_allowed_origins="*", async_mode="threading")

# ─── Configurazione moduli ────────────────────────────────────────────────────
MODULES = [
    {
        "group": "Camera",
        "color": "blue",
        "items": [
            {
                "id": "orbbec",
                "label": "Orbbec 335",
                "cmd": "ros2 launch orbbec_camera gemini2.launch.py",
                "type": "launch",
            },
        ]
    },
    {
        "group": "Percezione",
        "color": "purple",
        "items": [
            {
                "id": "insightface",
                "label": "InsightFace",
                "cmd": "ros2 run rakuda_perception face_recognition_node",
                "type": "node",
            },
            {
                "id": "yolo",
                "label": "YOLO Detection",
                "cmd": "ros2 launch isaac_ros_yolov8 isaac_ros_yolov8.launch.py",
                "type": "launch",
            },
        ]
    },
    {
        "group": "Isaac ROS",
        "color": "teal",
        "items": [
            {
                "id": "nvblox",
                "label": "nvblox",
                "cmd": "ros2 launch nvblox_examples_bringup orbbec_example.launch.py",
                "type": "launch",
            },
        ]
    },
    {
        "group": "Manipolazione",
        "color": "amber",
        "items": [
            {
                "id": "moveit",
                "label": "MoveIt2",
                "cmd": "ros2 launch rakuda_moveit moveit.launch.py",
                "type": "launch",
            },
        ]
    },
    {
        "group": "Intelligenza",
        "color": "coral",
        "items": [
            {
                "id": "ollama",
                "label": "Ollama / Gemma 4",
                "cmd": "ollama serve",
                "type": "service",
            },
            {
                "id": "orchestrator",
                "label": "Orchestratore",
                "cmd": "ros2 run rakuda_brain orchestrator_node",
                "type": "node",
            },
        ]
    },
    {
        "group": "Voice",
        "color": "green",
        "items": [
            {
                "id": "voice",
                "label": "GPT-4o Realtime",
                "cmd": "source ~/my_code/ag_echo.sh",
                "type": "shell",
            },
        ]
    },
]

# ─── Configurazione motions ──────────────────────────────────────────────────
MOTIONS = [
    {"id": "motion1", "label": "Motion 1", "cmd": "ros2 action send_goal /rakuda/motion rakuda_msgs/action/Motion \"{name: 'motion1'}\""},
    {"id": "motion2", "label": "Motion 2", "cmd": "ros2 action send_goal /rakuda/motion rakuda_msgs/action/Motion \"{name: 'motion2'}\""},
    {"id": "motion3", "label": "Motion 3", "cmd": "ros2 action send_goal /rakuda/motion rakuda_msgs/action/Motion \"{name: 'motion3'}\""},
    {"id": "motion4", "label": "Motion 4", "cmd": "ros2 action send_goal /rakuda/motion rakuda_msgs/action/Motion \"{name: 'motion4'}\""},
    {"id": "motion5", "label": "Motion 5", "cmd": "ros2 action send_goal /rakuda/motion rakuda_msgs/action/Motion \"{name: 'motion5'}\""},
]

# ─── Gestione processi ────────────────────────────────────────────────────────
processes = {}  # id -> subprocess.Popen

ROS_SETUP = (
    "source /opt/ros/humble/setup.bash && "
    "source ~/ros2_ws/install/setup.bash 2>/dev/null"
)

def stream_output(module_id, process):
    try:
        for line in process.stdout:
            line = line.rstrip()
            if line:
                socketio.emit("log", {"id": module_id, "msg": line})
        process.wait()
    except Exception as e:
        socketio.emit("log", {"id": module_id, "msg": f"[ERRORE] {e}"})
    finally:
        processes.pop(module_id, None)
        socketio.emit("status", {"id": module_id, "running": False})

def start_process(module_id, cmd, item_type):
    if module_id in processes:
        return False
    try:
        full_cmd = cmd if item_type == "shell" else f"{ROS_SETUP} && {cmd}"
        proc = subprocess.Popen(
            full_cmd, shell=True, executable="/bin/bash",
            stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
            text=True,
            env={**os.environ, "PYTHONUNBUFFERED": "1"}
        )
        processes[module_id] = proc
        t = threading.Thread(target=stream_output, args=(module_id, proc), daemon=True)
        t.start()
        return True
    except Exception as e:
        socketio.emit("log", {"id": module_id, "msg": f"[AVVIO FALLITO] {e}"})
        return False

def stop_process(module_id):
    proc = processes.pop(module_id, None)
    if proc:
        proc.terminate()
        try:
            proc.wait(timeout=4)
        except subprocess.TimeoutExpired:
            proc.kill()
        return True
    return False

# ─── SocketIO events ──────────────────────────────────────────────────────────
@socketio.on("start")
def handle_start(data):
    mid = data["id"]
    cmd = data["cmd"]
    itype = data.get("type", "node")
    ok = start_process(mid, cmd, itype)
    emit("status", {"id": mid, "running": ok})
    emit("log", {"id": mid, "msg": f"▶ Avvio: {cmd}" if ok else "✗ Già in esecuzione"})

@socketio.on("stop")
def handle_stop(data):
    mid = data["id"]
    ok = stop_process(mid)
    emit("status", {"id": mid, "running": False})
    emit("log", {"id": mid, "msg": "■ Fermato" if ok else "✗ Non in esecuzione"})

@socketio.on("nodes")
def handle_nodes():
    try:
        result = subprocess.run(
            f"{ROS_SETUP} && ros2 node list",
            shell=True, executable="/bin/bash",
            capture_output=True, text=True, timeout=5
        )
        nodes = result.stdout.strip().split("\n") if result.stdout.strip() else []
        emit("nodes", {"nodes": nodes})
    except Exception:
        emit("nodes", {"nodes": []})

@socketio.on("run_motion")
def handle_motion(data):
    mid = data["id"]
    cmd = data["cmd"]
    emit("log", {"id": "motion", "msg": f"▶ Esecuzione: {data['label']}"})
    try:
        result = subprocess.run(
            f"{ROS_SETUP} && {cmd}",
            shell=True, executable="/bin/bash",
            capture_output=True, text=True, timeout=15
        )
        out = result.stdout.strip() or result.stderr.strip() or "completato"
        emit("log", {"id": "motion", "msg": out})
        emit("motion_done", {"id": mid, "ok": result.returncode == 0})
    except subprocess.TimeoutExpired:
        emit("log", {"id": "motion", "msg": "[TIMEOUT] Motion non completata in tempo"})
        emit("motion_done", {"id": mid, "ok": False})
    except Exception as e:
        emit("log", {"id": "motion", "msg": f"[ERRORE] {e}"})
        emit("motion_done", {"id": mid, "ok": False})

@socketio.on("connect")
def handle_connect():
    running_ids = list(processes.keys())
    emit("init", {"running": running_ids, "modules": MODULES, "motions": MOTIONS})

# ─── Route HTML ───────────────────────────────────────────────────────────────
@app.route("/")
def index():
    with open(os.path.join(BASE_DIR, "rakuda_dashboard.html"), "r") as f:
        return f.read()

@app.route("/socket.io.min.js")
def serve_socketio_js():
    return send_from_directory(BASE_DIR, "socket.io.min.js")

if __name__ == "__main__":
    print("Rakuda Control Panel avviato su http://0.0.0.0:5000")
    socketio.run(app, host="0.0.0.0", port=5000, debug=False, allow_unsafe_werkzeug=True)
