import ctypes
import socket
import threading
import time
import json
from http.server import HTTPServer, BaseHTTPRequestHandler
import urllib.parse

class Pose(ctypes.Structure):
    _pack_ = 1
    _fields_ = [("x", ctypes.c_double), ("y", ctypes.c_double), ("z", ctypes.c_double), ("roll", ctypes.c_double), ("pitch", ctypes.c_double), ("yaw", ctypes.c_double), ("timestamp", ctypes.c_double), ("confidence", ctypes.c_double)]

class FlightCommand(ctypes.Structure):
    _pack_ = 1
    _pack_ = 1
    _fields_ = [("tx", ctypes.c_double), ("ty", ctypes.c_double), ("tz", ctypes.c_double), ("tyaw", ctypes.c_double), ("en", ctypes.c_bool), ("est", ctypes.c_bool)]

class Telemetry(ctypes.Structure):
    _pack_ = 1
    _fields_ = [("p", Pose), ("bat", ctypes.c_int), ("err", ctypes.c_bool)]

class UIGoalSequence(ctypes.Structure):
    _pack_ = 1
    _fields_ = [("g", Pose * 16), ("n", ctypes.c_size_t), ("start", ctypes.c_bool), ("stop", ctypes.c_bool), ("estop", ctypes.c_bool)]

class UIAckPacket(ctypes.Structure):
    _fields_ = [("received", ctypes.c_bool)]

class VisionGoalEstimate(ctypes.Structure):
    _fields_ = [("p", Pose), ("id", ctypes.c_int), ("det", ctypes.c_bool)]

state_lock = threading.Lock()
drone_pos = {"x": 0, "y": 0, "z": 1, "yaw": 0}
vision_goals = [
    {"id": 1, "x": 1.0, "y": 0.0, "z": 1.0, "yaw": 0.0},
    {"id": 2, "x": 1.0, "y": 1.0, "z": 1.0, "yaw": 0.0}
]
vision_enabled = True

import math

def fcu_thread():
    try:
        _fcu_thread()
    except Exception as e:
        import traceback; traceback.print_exc()

def _fcu_thread():
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind(("0.0.0.0", 14551))
    sock.setblocking(False)

    p = Pose(0,0,1,0,0,0,0,1)
    
    vx, vy, vz = 0.0, 0.0, 0.0
    last_cmd = None
    
    while True:
        try:
            data, addr = sock.recvfrom(1024)
            if len(data) == ctypes.sizeof(FlightCommand):
                last_cmd = FlightCommand.from_buffer_copy(data)
            else:
                print(f"Data length mismatch: {len(data)} != {ctypes.sizeof(FlightCommand)}")
        except BlockingIOError:
            pass
        except Exception as e:
            print(f"FCU error: {e}")

        dt = 0.05
        if last_cmd and last_cmd.en:
            fixed_speed = 2.0
            
            dx = last_cmd.tx - p.x
            dy = last_cmd.ty - p.y
            dz = last_cmd.tz - p.z
            dist = math.sqrt(dx*dx + dy*dy + dz*dz)
            
            if dist > fixed_speed * dt:
                p.x += (dx / dist) * fixed_speed * dt
                p.y += (dy / dist) * fixed_speed * dt
                p.z += (dz / dist) * fixed_speed * dt
            else:
                p.x = last_cmd.tx
                p.y = last_cmd.ty
                p.z = last_cmd.tz
            
            dyaw = last_cmd.tyaw - p.yaw
            while dyaw > math.pi: dyaw -= 2*math.pi
            while dyaw < -math.pi: dyaw += 2*math.pi
            p.yaw += dyaw
            
            p.pitch = 0.0
            p.roll = 0.0
            
            with state_lock:
                drone_pos["x"] = p.x
                drone_pos["y"] = p.y
                drone_pos["z"] = p.z
                drone_pos["yaw"] = p.yaw
                drone_pos["pitch"] = p.pitch
                drone_pos["roll"] = p.roll

        telem = Telemetry(p, 90, False)
        sock.sendto(bytes(telem), ("127.0.0.1", 14550))
        
        time.sleep(dt)



import os

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
INDEX_PATH = os.path.join(SCRIPT_DIR, "index.html")

global_detections = []

def vision_thread():
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    while True:
        with state_lock:
            enabled = vision_enabled
            dets = list(global_detections)
        
        if enabled:
            if not dets:
                est = VisionGoalEstimate()
                est.id = -1
                est.det = False
                sock.sendto(bytes(est), ("127.0.0.1", 14553))
            else:
                for det in dets:
                    est = VisionGoalEstimate()
                    est.p = Pose(det["x"], det["y"], det["z"], 0, 0, det["yaw"], time.time(), 1.0)
                    est.id = det["id"]
                    est.det = True
                    sock.sendto(bytes(est), ("127.0.0.1", 14553))
        time.sleep(0.05)

import asyncio
import websockets
from websockets.server import serve
from http import HTTPStatus
import urllib.parse

async def process_request(path, request_headers):
    parsed_path = urllib.parse.urlparse(path).path
    if parsed_path == "/" or parsed_path.endswith("index.html") or parsed_path == "":
        try:
            with open(INDEX_PATH, "rb") as f:
                content = f.read()
            return HTTPStatus.OK, [("Content-Type", "text/html")], content
        except FileNotFoundError:
            return HTTPStatus.NOT_FOUND, [], b"Not found"
    
    # Let normal websocket handshake proceed for other paths (like /ws)
    return None

async def handler(websocket):
    global vision_enabled, vision_goals, global_detections
    
    async def send_state():
        while True:
            with state_lock:
                state = {"drone": drone_pos, "goals": vision_goals, "vision_enabled": vision_enabled}
            try:
                await websocket.send(json.dumps(state))
            except Exception:
                break
            await asyncio.sleep(0.1) # 10Hz telemetry
            
    task = asyncio.create_task(send_state())
    
    try:
        async for message in websocket:
            data = json.loads(message)
            if data["type"] == "ui":
                seq = UIGoalSequence()
                seq.start = True
                seq.n = len(data["goals"])
                for i, g in enumerate(data["goals"]):
                    seq.g[i] = Pose(g["x"], g["y"], g["z"], 0, 0, g["yaw"], 0, 1.0)
                
                sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
                sock.sendto(bytes(seq), ("127.0.0.1", 14552))
                sock.close()
            elif data["type"] == "vision":
                with state_lock:
                    global_detections = data.get("detections", [])
            elif data["type"] == "toggle":
                with state_lock:
                    vision_enabled = data["enabled"]
            elif data["type"] == "update_goals":
                with state_lock:
                    vision_goals = data["goals"]
    except Exception:
        pass
    finally:
        task.cancel()

async def main():
    print("Serving at http://localhost:5000")
    async with serve(handler, "", 5000, process_request=process_request):
        await asyncio.Future()

if __name__ == '__main__':
    t1 = threading.Thread(target=fcu_thread, daemon=True)
    t2 = threading.Thread(target=vision_thread, daemon=True)
    t1.start()
    t2.start()
    try:
        asyncio.run(main())
    except Exception as e:
        print(f"MAIN FATAL ERROR: {repr(e)}")
