import ctypes
import socket
import threading
import time
import json
import math
import os
import asyncio
import websockets
from websockets.server import serve
from http import HTTPStatus
import urllib.parse

class Pose(ctypes.Structure):
    _pack_ = 1
    _fields_ = [("x", ctypes.c_double), ("y", ctypes.c_double), ("z", ctypes.c_double), 
                ("roll", ctypes.c_double), ("pitch", ctypes.c_double), ("yaw", ctypes.c_double), 
                ("timestamp", ctypes.c_double), ("confidence", ctypes.c_double)]

class FlightCommand(ctypes.Structure):
    _pack_ = 1
    _fields_ = [("tx", ctypes.c_double), ("ty", ctypes.c_double), ("tz", ctypes.c_double), 
                ("tyaw", ctypes.c_double), ("en", ctypes.c_bool), ("est", ctypes.c_bool)]

class Telemetry(ctypes.Structure):
    _pack_ = 1
    _fields_ = [("p", Pose), ("bat", ctypes.c_int), ("err", ctypes.c_bool)]

class Vec3f(ctypes.Structure):
    _pack_ = 1
    _fields_ = [("x", ctypes.c_float), ("y", ctypes.c_float), ("z", ctypes.c_float)]

class Quatf(ctypes.Structure):
    _pack_ = 1
    _fields_ = [("w", ctypes.c_float), ("x", ctypes.c_float), ("y", ctypes.c_float), ("z", ctypes.c_float)]

class Point2f(ctypes.Structure):
    _pack_ = 1
    _fields_ = [("u", ctypes.c_float), ("v", ctypes.c_float)]

class TagMeasurement(ctypes.Structure):
    _pack_ = 1
    _fields_ = [
        ("tag_id", ctypes.c_uint16),
        ("is_valid", ctypes.c_uint8),
        ("camera_pos_world", Vec3f),
        ("camera_att_world", Quatf),
        ("confidence", ctypes.c_float),
    ]

class PoseHypothesis(ctypes.Structure):
    _pack_ = 1
    _fields_ = [
        ("position_cam", Vec3f),
        ("orientation_cam", Quatf),
        ("confidence", ctypes.c_float),
    ]

class GoalMeasurement(ctypes.Structure):
    _pack_ = 1
    _fields_ = [
        ("shape_type", ctypes.c_uint8),
        ("is_detected", ctypes.c_uint8),
        ("centroid", Point2f),
        ("contour_area", ctypes.c_float),
        ("num_hypotheses", ctypes.c_uint8),
        ("hypotheses", PoseHypothesis * 4),
    ]

class VisionPacket(ctypes.Structure):
    _pack_ = 1
    _fields_ = [
        ("msg_type", ctypes.c_uint8),
        ("sequence_id", ctypes.c_uint32),
        ("timestamp_ns", ctypes.c_uint64),
        ("error_state", ctypes.c_uint32),
        ("num_tags_detected", ctypes.c_uint8),
        ("tags", TagMeasurement * 4),
        ("num_goals_reported", ctypes.c_uint8),
        ("goals", GoalMeasurement * 3),
    ]

class UICommandPacket(ctypes.Structure):
    _pack_ = 1
    _fields_ = [
        ("msg_type", ctypes.c_uint8),
        ("command_id", ctypes.c_uint32),
        ("timestamp_ns", ctypes.c_uint64),
        ("command", ctypes.c_uint8),
        ("sequence_length", ctypes.c_uint8),
        ("goal_sequence", ctypes.c_uint8 * 16),
        ("end_action", ctypes.c_uint8),
        ("hover_timeout_s", ctypes.c_float),
        ("use_tube_constraint", ctypes.c_uint8),
        ("tube_rail_y", ctypes.c_float),
        ("tube_rail_z", ctypes.c_float),
    ]

class UIStatusPacket(ctypes.Structure):
    _pack_ = 1
    _fields_ = [
        ("msg_type", ctypes.c_uint8),
        ("sequence_id", ctypes.c_uint32),
        ("timestamp_ns", ctypes.c_uint64),
        ("flight_state", ctypes.c_uint8),
        ("is_armed", ctypes.c_uint8),
        ("estop_active", ctypes.c_uint8),
        ("active_sequence_len", ctypes.c_uint8),
        ("current_seq_index", ctypes.c_uint8),
        ("current_target_shape", ctypes.c_uint8),
        ("configured_end_action", ctypes.c_uint8),
        ("battery_voltage", ctypes.c_float),
        ("battery_remaining", ctypes.c_float),
        ("active_error_flags", ctypes.c_uint32),
        ("last_acked_cmd_id", ctypes.c_uint32),
        ("cmd_ack_result", ctypes.c_uint8),
    ]

state_lock = threading.Lock()
drone_pos = {"x": 0, "y": 0, "z": 1, "yaw": 0}
vision_goals = [
    {"id": 1, "x": 1.0, "y": 0.0, "z": 1.0, "yaw": 0.0},
    {"id": 2, "x": 1.0, "y": 1.0, "z": 1.0, "yaw": 0.0}
]
vision_enabled = True
global_fixed_speed = 2.0
global_detections = []

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
            with state_lock:
                fixed_speed = global_fixed_speed
            
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

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
INDEX_PATH = os.path.join(SCRIPT_DIR, "index.html")

def vision_thread():
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    seq_id = 0
    while True:
        with state_lock:
            enabled = vision_enabled
            dets = list(global_detections)
        
        if enabled:
            pkt = VisionPacket()
            pkt.msg_type = 1 # MSG_TYPE_VISION
            pkt.sequence_id = seq_id
            seq_id += 1
            pkt.timestamp_ns = int(time.time() * 1e9)
            pkt.num_goals_reported = min(len(dets), 3)
            for i, det in enumerate(dets[:3]):
                gm = pkt.goals[i]
                gm.shape_type = det.get("shape", det.get("id", i + 1))
                gm.is_detected = 1
                gm.num_hypotheses = 1
                gm.hypotheses[0].position_cam = Vec3f(float(det["x"]), float(det["y"]), float(det["z"]))
                yaw = float(det.get("yaw", 0.0))
                gm.hypotheses[0].orientation_cam = Quatf(math.cos(yaw / 2.0), 0.0, 0.0, math.sin(yaw / 2.0))
                gm.hypotheses[0].confidence = 1.0
            sock.sendto(bytes(pkt), ("127.0.0.1", 14553))
        time.sleep(0.05)

async def process_request(path, request_headers):
    parsed_path = urllib.parse.urlparse(path).path
    if parsed_path == "/" or parsed_path.endswith("index.html") or parsed_path == "":
        try:
            with open(INDEX_PATH, "rb") as f:
                content = f.read()
            return HTTPStatus.OK, [("Content-Type", "text/html")], content
        except FileNotFoundError:
            return HTTPStatus.NOT_FOUND, [], b"Not found"
    return None

cmd_counter = 0

async def handler(websocket):
    global vision_enabled, vision_goals, global_detections, global_fixed_speed, cmd_counter
    
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
                cmd = UICommandPacket()
                cmd.msg_type = 2 # MSG_TYPE_UI_CMD
                cmd_counter += 1
                cmd.command_id = cmd_counter
                cmd.timestamp_ns = int(time.time() * 1e9)
                cmd.command = 3 # UI_CMD_START_MISSION
                cmd.sequence_length = min(len(data["goals"]), 16)
                for i, g in enumerate(data["goals"][:16]):
                    cmd.goal_sequence[i] = g.get("shape", g.get("id", i + 1))
                cmd.end_action = 0 # MISSION_END_HOVER
                
                sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
                sock.settimeout(0.5)
                sock.sendto(bytes(cmd), ("127.0.0.1", 14552))
                try:
                    sock.recvfrom(1024)
                except socket.timeout:
                    pass
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
            elif data["type"] == "config":
                with state_lock:
                    global_fixed_speed = data["max_velocity"]
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
