import ctypes
import socket
import threading
import time
import json
import math
import os
import random
import asyncio
import websockets
from websockets.server import serve
from http import HTTPStatus
import urllib.parse

# ---------------------------------------------------------------------------
# C-Compatible Data Contracts
# ---------------------------------------------------------------------------
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

# ---------------------------------------------------------------------------
# Kinematic Follower with Configurable Imputed Noise
# ---------------------------------------------------------------------------
class FollowerSimulator:
    def __init__(self):
        self.lock = threading.Lock()
        self.tau_xy = 0.15
        self.tau_z = 0.20
        self.tau_yaw = 0.10
        self.max_velocity_xy = 2.5
        self.max_velocity_z = 1.5
        self.max_acceleration = 8.0
        self.max_yaw_rate = 2.0

        self.noise_enabled = True
        self.pos_noise_std_xy = 0.03
        self.pos_noise_std_z = 0.015
        self.yaw_noise_std = 0.02

        self.drift_vel_x = 0.0
        self.drift_vel_y = 0.0
        self.drift_vel_z = 0.0

        self.true_x = 0.0
        self.true_y = 0.0
        self.true_z = 1.0
        self.true_yaw = 0.0
        self.vx = 0.0
        self.vy = 0.0
        self.vz = 0.0

        self.feedback_x = 0.0
        self.feedback_y = 0.0
        self.feedback_z = 1.0
        self.feedback_yaw = 0.0

        self.total_samples = 0
        self.sum_sq_err = 0.0
        self.max_error = 0.0
        self.current_error = 0.0
        self.total_dist = 0.0
        self.tag_corrections = 0
        self.last_target = {"x": 0.0, "y": 0.0, "z": 1.0, "yaw": 0.0}

    def correct_drift(self, x, y, z, yaw, confidence=0.95):
        with self.lock:
            k = min(0.95, max(0.20, confidence))
            self.feedback_x += k * (x - self.feedback_x)
            self.feedback_y += k * (y - self.feedback_y)
            self.feedback_z += k * (z - self.feedback_z)
            dyaw = yaw - self.feedback_yaw
            while dyaw > math.pi: dyaw -= 2 * math.pi
            while dyaw < -math.pi: dyaw += 2 * math.pi
            self.feedback_yaw += k * dyaw
            self.tag_corrections += 1

    def step(self, cmd, dt):
        with self.lock:
            if not cmd or not cmd.en:
                self.vx = 0.0
                self.vy = 0.0
                self.vz = 0.0
                self._apply_noise()
                return

            if cmd.est:
                self.true_z = max(0.0, self.true_z - 9.81 * dt * dt)
                self._apply_noise()
                return

            self.last_target = {"x": cmd.tx, "y": cmd.ty, "z": cmd.tz, "yaw": cmd.tyaw}

            des_vx = (cmd.tx - self.true_x) / max(0.01, self.tau_xy)
            des_vy = (cmd.ty - self.true_y) / max(0.01, self.tau_xy)
            des_vz = (cmd.tz - self.true_z) / max(0.01, self.tau_z)

            h_speed = math.sqrt(des_vx*des_vx + des_vy*des_vy)
            if h_speed > self.max_velocity_xy and h_speed > 1e-6:
                scale = self.max_velocity_xy / h_speed
                des_vx *= scale
                des_vy *= scale
            des_vz = max(-self.max_velocity_z, min(self.max_velocity_z, des_vz))

            max_dv = self.max_acceleration * dt
            self.vx += max(-max_dv, min(max_dv, des_vx - self.vx))
            self.vy += max(-max_dv, min(max_dv, des_vy - self.vy))
            self.vz += max(-max_dv, min(max_dv, des_vz - self.vz))

            dx = (self.vx + self.drift_vel_x) * dt
            dy = (self.vy + self.drift_vel_y) * dt
            dz = (self.vz + self.drift_vel_z) * dt
            self.true_x += dx
            self.true_y += dy
            self.true_z = max(0.0, self.true_z + dz)
            self.total_dist += math.sqrt(dx*dx + dy*dy + dz*dz)

            dyaw = cmd.tyaw - self.true_yaw
            while dyaw > math.pi: dyaw -= 2 * math.pi
            while dyaw < -math.pi: dyaw += 2 * math.pi
            yaw_rate = max(-self.max_yaw_rate, min(self.max_yaw_rate, dyaw / max(0.01, self.tau_yaw)))
            self.true_yaw += yaw_rate * dt
            while self.true_yaw > math.pi: self.true_yaw -= 2 * math.pi
            while self.true_yaw < -math.pi: self.true_yaw += 2 * math.pi

            self._apply_noise()

            err_x = cmd.tx - self.true_x
            err_y = cmd.ty - self.true_y
            err_z = cmd.tz - self.true_z
            self.current_error = math.sqrt(err_x*err_x + err_y*err_y + err_z*err_z)
            if self.current_error > self.max_error:
                self.max_error = self.current_error
            self.sum_sq_err += self.current_error * self.current_error
            self.total_samples += 1

    def _apply_noise(self):
        nx = random.gauss(0, self.pos_noise_std_xy) if self.noise_enabled else 0.0
        ny = random.gauss(0, self.pos_noise_std_xy) if self.noise_enabled else 0.0
        nz = random.gauss(0, self.pos_noise_std_z) if self.noise_enabled else 0.0
        nyaw = random.gauss(0, self.yaw_noise_std) if self.noise_enabled else 0.0

        self.feedback_x = self.true_x + nx
        self.feedback_y = self.true_y + ny
        self.feedback_z = max(0.0, self.true_z + nz)
        self.feedback_yaw = self.true_yaw + nyaw

    def get_state(self):
        with self.lock:
            rmse = math.sqrt(self.sum_sq_err / max(1, self.total_samples))
            return {
                "drone": {
                    "x": self.true_x, "y": self.true_y, "z": self.true_z,
                    "yaw": self.true_yaw, "pitch": max(-0.35, min(0.35, self.vx / 9.81)),
                    "roll": max(-0.35, min(0.35, -self.vy / 9.81))
                },
                "feedback": {
                    "x": self.feedback_x, "y": self.feedback_y, "z": self.feedback_z,
                    "yaw": self.feedback_yaw
                },
                "command": self.last_target,
                "metrics": {
                    "current_error": round(self.current_error, 4),
                    "rmse": round(rmse, 4),
                    "max_error": round(self.max_error, 4),
                    "distance": round(self.total_dist, 2)
                },
                "config": {
                    "noise_enabled": self.noise_enabled,
                    "pos_noise_std": self.pos_noise_std_xy,
                    "yaw_noise_std": self.yaw_noise_std,
                    "drift_x": self.drift_vel_x,
                    "drift_y": self.drift_vel_y,
                    "tau": self.tau_xy
                }
            }

follower = FollowerSimulator()
state_lock = threading.Lock()
vision_goals = [
    {"id": 1, "shape": 3, "x": 1.5, "y": -0.5, "z": 1.0, "yaw": 0.0},
    {"id": 2, "shape": 2, "x": 3.0, "y": 0.0, "z": 1.2, "yaw": 0.1},
    {"id": 3, "shape": 1, "x": 4.5, "y": 0.5, "z": 1.0, "yaw": 0.0}
]
vision_tags = [
    {"id": 1, "x": 1.5, "y": -0.5, "z": 1.0, "yaw": 0.0, "confidence": 0.95},
    {"id": 2, "x": 3.0, "y": 0.0, "z": 1.2, "yaw": 0.1, "confidence": 0.95},
    {"id": 3, "x": 4.5, "y": 0.5, "z": 1.0, "yaw": 0.0, "confidence": 0.95}
]
vision_enabled = True
global_detections = []

sim_mission_active = False
sim_mission_goals = []
sim_mission_idx = 0
sim_mission_substep = 0
last_cmd_time = 0.0

def fcu_thread():
    try:
        _fcu_thread()
    except Exception as e:
        import traceback; traceback.print_exc()

def _fcu_thread():
    global last_cmd_time, sim_mission_active, sim_mission_idx, sim_mission_substep
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind(("0.0.0.0", 14551))
    sock.setblocking(False)

    last_cmd = None
    dt = 0.05

    while True:
        try:
            data, addr = sock.recvfrom(2048)
            if len(data) == ctypes.sizeof(FlightCommand):
                last_cmd = FlightCommand.from_buffer_copy(data)
                last_cmd_time = time.time()
            elif len(data) >= 12 and data[0] == 0xFD:
                # MAVLink v2 packet
                payload_len = data[1]
                msgid = data[7] | (data[8] << 8) | (data[9] << 16)
                if msgid == 102 and len(data) >= 10 + 32: # VISION_POSITION_ESTIMATE
                    import struct
                    usec, x, y, z, roll, pitch, yaw = struct.unpack_from("<Qffffff", data[10:])
                    follower.correct_drift(x, y, z, yaw)
        except BlockingIOError:
            pass
        except Exception as e:
            print(f"FCU error: {e}")

        now = time.time()
        active_cmd = None

        if last_cmd is not None and (now - last_cmd_time) < 1.0 and last_cmd.en:
            active_cmd = last_cmd
        elif sim_mission_active and sim_mission_goals:
            with state_lock:
                if sim_mission_idx < len(sim_mission_goals):
                    g = sim_mission_goals[sim_mission_idx]
                    gx, gy, gz = float(g["x"]), float(g["y"]), float(g["z"])
                    gyaw = float(g.get("yaw", 0.0))

                    # Orientation vector perpendicular to hoop plane
                    nx = math.cos(gyaw)
                    ny = math.sin(gyaw)

                    # Substep 0: Approach waypoint (0.50m in front of gate)
                    # Substep 1: Traversal exit waypoint (0.50m behind gate)
                    if sim_mission_substep == 0:
                        tx = gx - 0.50 * nx
                        ty = gy - 0.50 * ny
                        tz = gz
                        dist_2d = math.hypot(tx - follower.true_x, ty - follower.true_y)
                        target_yaw = math.atan2(ty - follower.true_y, tx - follower.true_x) if dist_2d > 0.15 else gyaw
                    else:
                        tx = gx + 0.50 * nx
                        ty = gy + 0.50 * ny
                        tz = gz
                        target_yaw = gyaw

                    dist = math.sqrt((tx - follower.true_x)**2 + (ty - follower.true_y)**2 + (tz - follower.true_z)**2)
                    if dist < 0.22:
                        if sim_mission_substep == 0:
                            sim_mission_substep = 1
                        else:
                            sim_mission_substep = 0
                            sim_mission_idx += 1
                            if sim_mission_idx >= len(sim_mission_goals):
                                sim_mission_active = False

                    active_cmd = FlightCommand(tx, ty, tz, target_yaw, True, False)
                else:
                    sim_mission_active = False
                    active_cmd = FlightCommand(follower.true_x, follower.true_y, follower.true_z, follower.true_yaw, True, False)

        follower.step(active_cmd, dt)

        # Transmit feedback pose in Telemetry datagram to Autopilot
        with follower.lock:
            p = Pose(follower.feedback_x, follower.feedback_y, follower.feedback_z,
                     0.0, 0.0, follower.feedback_yaw, time.time(), 1.0)
        telem = Telemetry(p, 92, False)
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
            dets = list(global_detections) if global_detections else list(vision_goals)
            tags = list(vision_tags)

        if enabled:
            pkt = VisionPacket()
            pkt.msg_type = 1 # MSG_TYPE_VISION
            pkt.sequence_id = seq_id
            seq_id += 1
            pkt.timestamp_ns = int(time.time() * 1e9)

            # Goals
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

            # Global Tag Targets
            pkt.num_tags_detected = min(len(tags), 4)
            for i, tag in enumerate(tags[:4]):
                tm = pkt.tags[i]
                tm.tag_id = int(tag.get("id", i + 1))
                tm.is_valid = 1
                tm.camera_pos_world = Vec3f(float(tag["x"]), float(tag["y"]), float(tag["z"]))
                yaw = float(tag.get("yaw", 0.0))
                tm.camera_att_world = Quatf(math.cos(yaw / 2.0), 0.0, 0.0, math.sin(yaw / 2.0))
                tm.confidence = float(tag.get("confidence", 0.95))

            sock.sendto(bytes(pkt), ("127.0.0.1", 14553))
        time.sleep(0.05)

async def process_request(path, request_headers):
    # If this is a WebSocket upgrade handshake, proceed directly
    if request_headers.get("Upgrade", "").lower() == "websocket":
        return None

    # For HTTP requests, serve the simulation dashboard
    try:
        with open(INDEX_PATH, "rb") as f:
            content = f.read()
        return HTTPStatus.OK, [("Content-Type", "text/html; charset=utf-8")], content
    except FileNotFoundError:
        return HTTPStatus.NOT_FOUND, [], b"Not found"

cmd_counter = 0

async def handler(websocket):
    global vision_enabled, vision_goals, global_detections, cmd_counter
    global sim_mission_active, sim_mission_goals, sim_mission_idx, sim_mission_substep

    async def send_state():
        while True:
            try:
                sim_state = follower.get_state()
                with state_lock:
                    sim_state["goals"] = vision_goals
                    sim_state["vision_enabled"] = vision_enabled
                    sim_state["mission_active"] = sim_mission_active
                    sim_state["mission_idx"] = sim_mission_idx
                    sim_state["autopilot_connected"] = (time.time() - last_cmd_time < 1.0)
                await websocket.send(json.dumps(sim_state))
            except Exception:
                break
            await asyncio.sleep(0.05) # 20Hz telemetry

    task = asyncio.create_task(send_state())

    try:
        async for message in websocket:
            data = json.loads(message)
            msg_type = data.get("type")
            if msg_type == "ui":
                cmd_val = data.get("command", 3)
                goals_to_send = data.get("goals", vision_goals)
                if cmd_val == 3: # START_MISSION
                    with state_lock:
                        sim_mission_active = True
                        sim_mission_goals = list(goals_to_send)
                        sim_mission_idx = 0
                        sim_mission_substep = 0
                elif cmd_val in (2, 7): # DISARM or ESTOP
                    with state_lock:
                        sim_mission_active = False
                    with follower.lock:
                        follower.vx = 0.0; follower.vy = 0.0; follower.vz = 0.0
                        if cmd_val == 7: # ESTOP immediately cuts power
                            follower.true_z = max(0.0, follower.true_z - 0.5)

                cmd = UICommandPacket()
                cmd.msg_type = 2 # MSG_TYPE_UI_CMD
                cmd_counter += 1
                cmd.command_id = cmd_counter
                cmd.timestamp_ns = int(time.time() * 1e9)
                cmd.command = cmd_val
                cmd.sequence_length = min(len(goals_to_send), 16)
                for i, g in enumerate(goals_to_send[:16]):
                    cmd.goal_sequence[i] = g.get("shape", g.get("id", i + 1))
                cmd.end_action = data.get("end_action", 0) # MISSION_END_HOVER

                sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
                sock.settimeout(0.1)
                try:
                    sock.sendto(bytes(cmd), ("127.0.0.1", 14552))
                except Exception:
                    pass
                sock.close()
            elif msg_type == "follower_config":
                with follower.lock:
                    if "pos_noise_std" in data:
                        follower.pos_noise_std_xy = float(data["pos_noise_std"])
                        follower.pos_noise_std_z = follower.pos_noise_std_xy * 0.5
                    if "yaw_noise_std" in data:
                        follower.yaw_noise_std = float(data["yaw_noise_std"])
                    if "drift_x" in data:
                        follower.drift_vel_x = float(data["drift_x"])
                    if "drift_y" in data:
                        follower.drift_vel_y = float(data["drift_y"])
                    if "tau" in data:
                        follower.tau_xy = float(data["tau"])
                        follower.tau_z = follower.tau_xy * 1.33
                    if "noise_enabled" in data:
                        follower.noise_enabled = bool(data["noise_enabled"])
            elif msg_type == "reset_sim":
                with state_lock:
                    sim_mission_active = False
                    sim_mission_idx = 0
                    sim_mission_substep = 0
                with follower.lock:
                    follower.true_x = 0.0
                    follower.true_y = 0.0
                    follower.true_z = 1.0
                    follower.true_yaw = 0.0
                    follower.vx = 0.0; follower.vy = 0.0; follower.vz = 0.0
                    follower.total_samples = 0
                    follower.sum_sq_err = 0.0
                    follower.max_error = 0.0
                    follower.current_error = 0.0
                    follower.total_dist = 0.0
                    follower.last_target = None
            elif msg_type == "toggle":
                with state_lock:
                    vision_enabled = data["enabled"]
            elif msg_type == "update_goals":
                with state_lock:
                    vision_goals = data["goals"]
    except Exception:
        pass
    finally:
        task.cancel()

async def main():
    print("Autopilot Simulation Server running at http://localhost:5000")
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
