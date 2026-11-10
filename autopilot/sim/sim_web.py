import ctypes
import socket
import threading
import time
import json
import math
import os
import random
import asyncio
import sys
import glob
import base64
import subprocess
import atexit
import signal

try:
    import websockets
    from websockets.server import serve
except ImportError:
    for root, dirs, _ in os.walk(os.path.expanduser("~/.cache/uv")):
        if any(d.startswith("websockets-") for d in dirs):
            if "websockets" in dirs:
                sys.path.insert(0, root)
                break
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

    @property
    def x(self): return self.u
    @x.setter
    def x(self, val): self.u = float(val)

    @property
    def y(self): return self.v
    @y.setter
    def y(self, val): self.v = float(val)

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

class GoalTrackTelemetry(ctypes.Structure):
    _pack_ = 1
    _fields_ = [
        ("shape_type", ctypes.c_uint8),
        ("state", ctypes.c_uint8),
        ("world_position", Vec3f),
        ("spatial_variance", ctypes.c_float),
    ]

class TelemetryPacket(ctypes.Structure):
    _pack_ = 1
    _fields_ = [
        ("msg_type", ctypes.c_uint8),
        ("sequence_id", ctypes.c_uint32),
        ("timestamp_ns", ctypes.c_uint64),
        ("drone_pos_ned", Vec3f),
        ("drone_vel_ned", Vec3f),
        ("drone_att_ned", Quatf),
        ("target_pos_ned", Vec3f),
        ("target_vel_ned", Vec3f),
        ("goals", GoalTrackTelemetry * 3),
        ("num_tags_in_view", ctypes.c_uint8),
        ("tag_ids_in_view", ctypes.c_uint16 * 4),
        ("optical_flow_quality", ctypes.c_float),
        ("ekf_pos_variance", ctypes.c_float),
        ("cpu_temperature_c", ctypes.c_float),
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
            c_yaw = math.cos(self.true_yaw)
            s_yaw = math.sin(self.true_yaw)
            v_fwd = self.vx * c_yaw + self.vy * s_yaw
            v_right = self.vx * s_yaw - self.vy * c_yaw
            return {
                "drone": {
                    "x": self.true_x, "y": self.true_y, "z": self.true_z,
                    "yaw": self.true_yaw,
                    "pitch": max(-0.35, min(0.35, v_fwd / 9.81)),
                    "roll": max(-0.35, min(0.35, v_right / 9.81))
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
    {"id": 1, "shape": 1, "x": 1.5, "y": -0.4, "z": 1.0, "yaw": 0.0},
    {"id": 2, "shape": 2, "x": 3.0, "y": 0.0, "z": 1.2, "yaw": 0.0},
    {"id": 3, "shape": 3, "x": 4.5, "y": 0.4, "z": 1.0, "yaw": 0.0}
]

TAGS_JSON_PATH = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "config", "surveyed_tags.json")

def load_default_surveyed_tags():
    tags = []
    if os.path.exists(TAGS_JSON_PATH):
        try:
            with open(TAGS_JSON_PATH, "r") as f:
                data = json.load(f)
            for t in data.get("tags", []):
                pos = t.get("position_ned", [0.0, 0.0, 0.0])
                rpy = t.get("attitude_rpy_deg", [0.0, 0.0, 0.0])
                tags.append({
                    "id": int(t["id"]),
                    "label": str(t.get("label", f"Tag {t['id']}")),
                    "x": float(pos[0]),
                    "y": float(pos[1]),
                    "z": float(pos[2]),
                    "yaw": math.radians(float(rpy[2])),
                    "pitch": math.radians(float(rpy[1])),
                    "roll": math.radians(float(rpy[0])),
                    "size": float(t.get("size_m", 0.16)),
                    "confidence": 0.95
                })
        except Exception as e:
            print(f"Error loading {TAGS_JSON_PATH}: {e}")
    if not tags:
        tags = [
            {"id": 1, "label": "Gate 1 Entrance Marker", "x": 1.5, "y": -0.3, "z": 1.0, "yaw": 0.0, "pitch": 0.0, "roll": 0.0, "size": 0.16, "confidence": 0.95},
            {"id": 2, "label": "Gate 2 Center Pylon", "x": 3.0, "y": 0.2, "z": 1.2, "yaw": math.radians(15.0), "pitch": 0.0, "roll": 0.0, "size": 0.16, "confidence": 0.95},
            {"id": 3, "label": "Gate 3 Exit Marker", "x": 4.5, "y": -0.1, "z": 1.0, "yaw": math.radians(-10.0), "pitch": 0.0, "roll": 0.0, "size": 0.16, "confidence": 0.95},
            {"id": 4, "label": "Arena Perimeter Anchor", "x": 0.0, "y": 2.0, "z": 0.5, "yaw": math.radians(90.0), "pitch": 0.0, "roll": 0.0, "size": 0.20, "confidence": 0.95}
        ]
    return tags

vision_tags = load_default_surveyed_tags()
current_tag_detections = []
vision_enabled = True
global_detections = []

# ---------------------------------------------------------------------------
# Vision Perception & Collision Modeling
# ---------------------------------------------------------------------------
current_detections = [] # Active camera frame detections for HUD
current_tag_detections = []
collision_state = {"collision": False, "object": "", "time": 0.0}
mission_status_text = "STANDBY (Ready)"
last_cmd_time = 0.0

def evaluate_camera_vision(true_x, true_y, true_z, true_yaw, goals):
    cam_x = true_x + 0.18 * math.cos(true_yaw)
    cam_y = true_y + 0.18 * math.sin(true_yaw)
    cam_z = true_z + 0.04

    cos_yaw = math.cos(true_yaw)
    sin_yaw = math.sin(true_yaw)

    # 70 deg horiz FOV (+-35 deg), 50 deg vert FOV (+-25 deg)
    fx = 160.0 / math.tan(math.radians(35.0))
    fy = 120.0 / math.tan(math.radians(25.0))

    detections = []

    for g in goals:
        gx, gy, gz = float(g["x"]), float(g["y"]), float(g["z"])
        gyaw = float(g.get("yaw", 0.0))
        shape_type = int(g.get("shape", g.get("id", 1)))

        dx = gx - cam_x
        dy = gy - cam_y
        dz = gz - cam_z
        dist = math.sqrt(dx*dx + dy*dy + dz*dz)

        # Range filter [0.15m, 6.00m]
        if dist < 0.15 or dist > 6.00:
            continue

        fwd = dx * cos_yaw + dy * sin_yaw
        lat = dx * sin_yaw - dy * cos_yaw
        vert = dz

        # Must be in front of camera
        if fwd <= 0.15:
            continue

        h_angle = math.atan2(lat, fwd)
        v_angle = math.atan2(vert, fwd)
        if abs(h_angle) > math.radians(35.0) or abs(v_angle) > math.radians(25.0):
            continue

        # Line of Sight / Occlusion check against closer hoops
        occluded = False
        ray_x, ray_y, ray_z = dx / dist, dy / dist, dz / dist
        for other in goals:
            if other.get("id") == g.get("id"):
                continue
            ox = float(other["x"]) - cam_x
            oy = float(other["y"]) - cam_y
            oz = float(other["z"]) - cam_z
            o_dist = math.sqrt(ox*ox + oy*oy + oz*oz)
            if o_dist < dist - 0.25:
                proj = ox * ray_x + oy * ray_y + oz * ray_z
                if 0 < proj < dist:
                    perp_sq = (ox - proj * ray_x)**2 + (oy - proj * ray_y)**2 + (oz - proj * ray_z)**2
                    if perp_sq < (0.28 ** 2):
                        occluded = True
                        break
        if occluded:
            continue

        u = int(160.0 + fx * (lat / fwd))
        v = int(120.0 - fy * (vert / fwd))
        box_size = int(max(18, min(140, fx * 0.40 / fwd)))

        noise_pos = 0.012 * (dist / 1.5)
        meas_x = gx + random.gauss(0, noise_pos)
        meas_y = gy + random.gauss(0, noise_pos)
        meas_z = gz + random.gauss(0, noise_pos * 0.5)
        meas_yaw = gyaw + random.gauss(0, 0.015)
        conf = max(0.45, min(0.99, 1.0 - (dist / 6.00) * 0.45))

        detections.append({
            "id": g.get("id", shape_type),
            "shape": shape_type,
            "dist": round(dist, 2),
            "fwd": fwd,
            "lat": lat,
            "vert": vert,
            "x": meas_x,
            "y": meas_y,
            "z": meas_z,
            "yaw": meas_yaw,
            "conf": round(conf, 2),
            "u": u,
            "v": v,
            "box_size": box_size
        })

    return detections

def evaluate_camera_tags(true_x, true_y, true_z, true_yaw, tags):
    cam_x = true_x + 0.18 * math.cos(true_yaw)
    cam_y = true_y + 0.18 * math.sin(true_yaw)
    cam_z = true_z + 0.04

    cos_yaw = math.cos(true_yaw)
    sin_yaw = math.sin(true_yaw)

    fx = 160.0 / math.tan(math.radians(35.0))
    fy = 120.0 / math.tan(math.radians(25.0))

    tag_detections = []

    for t in tags:
        tx, ty, tz = float(t["x"]), float(t["y"]), float(t["z"])
        tyaw = float(t.get("yaw", 0.0))
        tag_id = int(t.get("id", 1))

        dx = tx - cam_x
        dy = ty - cam_y
        dz = tz - cam_z
        dist = math.sqrt(dx*dx + dy*dy + dz*dz)

        # Range filter [0.15m, 6.00m]
        if dist < 0.15 or dist > 6.00:
            continue

        fwd = dx * cos_yaw + dy * sin_yaw
        lat = dx * sin_yaw - dy * cos_yaw
        vert = dz

        # Must be in front of camera
        if fwd <= 0.15:
            continue

        h_angle = math.atan2(lat, fwd)
        v_angle = math.atan2(vert, fwd)
        if abs(h_angle) > math.radians(35.0) or abs(v_angle) > math.radians(25.0):
            continue

        u = int(160.0 + fx * (lat / fwd))
        v = int(120.0 - fy * (vert / fwd))
        tag_size = float(t.get("size", 0.16))
        box_size = int(max(14, min(120, fx * tag_size / fwd)))

        noise_pos = 0.008 * (dist / 1.5)
        meas_x = tx + random.gauss(0, noise_pos)
        meas_y = ty + random.gauss(0, noise_pos)
        meas_z = tz + random.gauss(0, noise_pos * 0.5)
        meas_yaw = tyaw + random.gauss(0, 0.015)
        meas_cam_x = cam_x + random.gauss(0, noise_pos)
        meas_cam_y = cam_y + random.gauss(0, noise_pos)
        meas_cam_z = cam_z + random.gauss(0, noise_pos * 0.5)
        meas_cam_yaw = true_yaw + random.gauss(0, 0.015)
        conf = max(0.50, min(0.99, 1.0 - (dist / 6.00) * 0.35))

        tag_detections.append({
            "id": tag_id,
            "label": t.get("label", f"Tag {tag_id}"),
            "dist": round(dist, 2),
            "x": meas_x,
            "y": meas_y,
            "z": meas_z,
            "yaw": meas_yaw,
            "cam_x": meas_cam_x,
            "cam_y": meas_cam_y,
            "cam_z": meas_cam_z,
            "cam_yaw": meas_cam_yaw,
            "conf": round(conf, 2),
            "u": u,
            "v": v,
            "box_size": box_size
        })

    return tag_detections

def dist_to_segment_2d(pu, pw, ax, ay, bx, by):
    vx = bx - ax
    vy = by - ay
    l2 = vx*vx + vy*vy
    if l2 < 1e-8:
        return math.hypot(pu - ax, pw - ay)
    t = max(0.0, min(1.0, ((pu - ax)*vx + (pw - ay)*vy) / l2))
    return math.hypot(pu - (ax + t * vx), pw - (ay + t * vy))

def check_collision(px, py, pz, pyaw, goals):
    drone_radius = 0.06
    tube_radius = 0.03
    collision_dist = tube_radius + drone_radius # 0.09m

    for g in goals:
        gx, gy, gz = float(g["x"]), float(g["y"]), float(g["z"])
        gyaw = float(g.get("yaw", 0.0))
        shape_type = int(g.get("shape", g.get("id", 1)))

        rx = px - gx
        ry = py - gy
        rz = pz - gz

        gnx = math.cos(gyaw)
        gny = math.sin(gyaw)
        h = rx * gnx + ry * gny # axial distance along normal
        u = -rx * gny + ry * gnx # horizontal distance in hoop plane
        w = rz # vertical distance in hoop plane

        # 1. Circle (shape 1): Torus R = 0.22m, tube = 0.03m
        if shape_type == 1:
            rho = math.hypot(u, w)
            d_2d = abs(rho - 0.22)
            d_tube = math.hypot(d_2d, h)
            if d_tube < collision_dist:
                return True, f"Clipped Gate 1 hoop rim! (d={d_tube:.2f}m, radial: {rho:.2f}m)"
            # Pylon extends below bottom of hoop: w < -0.25m
            if w < -0.25 and pz >= 0.0:
                dist_pylon = math.hypot(rx, ry)
                if dist_pylon < (0.015 + drone_radius):
                    return True, "Collided with Gate 1 pylon!"

        # 2. Triangle (shape 2): Equilateral triangle, R = 0.35m, apex UP
        elif shape_type == 2:
            # Vertices: Apex at (0, 0.35), Bottom-Left at (-0.3031, -0.175), Bottom-Right at (+0.3031, -0.175)
            d_base = dist_to_segment_2d(u, w, -0.30311, -0.175, 0.30311, -0.175)
            d_left = dist_to_segment_2d(u, w, -0.30311, -0.175, 0.0, 0.35)
            d_right = dist_to_segment_2d(u, w, 0.30311, -0.175, 0.0, 0.35)
            d_2d = min(d_base, d_left, d_right)
            d_tube = math.hypot(d_2d, h)
            if d_tube < collision_dist:
                return True, f"Clipped Gate 2 hoop rim! (d={d_tube:.2f}m, in-plane: {d_2d:.2f}m)"
            # Pylon extends below bottom of base: w < -0.205m
            if w < -0.205 and pz >= 0.0:
                dist_pylon = math.hypot(rx, ry)
                if dist_pylon < (0.015 + drone_radius):
                    return True, "Collided with Gate 2 pylon!"

        # 3. Square (shape 3): Upright square, half-width L = 0.1768m
        elif shape_type == 3:
            L = 0.17678
            d_top = dist_to_segment_2d(u, w, -L, L, L, L)
            d_bot = dist_to_segment_2d(u, w, -L, -L, L, -L)
            d_left = dist_to_segment_2d(u, w, -L, -L, -L, L)
            d_right = dist_to_segment_2d(u, w, L, -L, L, L)
            d_2d = min(d_top, d_bot, d_left, d_right)
            d_tube = math.hypot(d_2d, h)
            if d_tube < collision_dist:
                return True, f"Clipped Gate 3 hoop rim! (d={d_tube:.2f}m, in-plane: {d_2d:.2f}m)"
            # Pylon extends below bottom edge: w < -0.207m
            if w < -0.207 and pz >= 0.0:
                dist_pylon = math.hypot(rx, ry)
                if dist_pylon < (0.015 + drone_radius):
                    return True, "Collided with Gate 3 pylon!"

    return False, ""

def fcu_thread():
    try:
        _fcu_thread()
    except Exception as e:
        import traceback; traceback.print_exc()

last_step_cmd = None
last_fcu_cmd = None

def step_fcu(dt=0.05, last_cmd=None, now=None):
    global last_cmd_time, mission_status_text, collision_state, last_step_cmd

    if now is None:
        now = time.time()

    active_cmd = None

    if last_cmd is not None and (now - last_cmd_time) < 1.0 and last_cmd.en:
        active_cmd = last_cmd
        mission_status_text = "AUTOPILOT RUNNING (Port 14551)"
    else:
        # Hover / standby when Autopilot has not engaged motors or is disarmed
        active_cmd = FlightCommand(follower.true_x, follower.true_y, follower.true_z, follower.true_yaw, False, False)
        if not collision_state["collision"]:
            if (now - last_cmd_time) < 3.0:
                mission_status_text = "AUTOPILOT STANDBY"
            else:
                mission_status_text = "STANDBY (Autopilot Ready on UDP 14551/14552)"

    # Check for physical collisions against hoop structures
    with state_lock:
        is_hit, hit_msg = check_collision(follower.true_x, follower.true_y, follower.true_z,
                                          follower.true_yaw, vision_goals)
        if is_hit and not collision_state["collision"]:
            collision_state["collision"] = True
            collision_state["object"] = hit_msg
            collision_state["time"] = now
            mission_status_text = f"CRASH: {hit_msg}"
            print(f"[SIM] CRASH: {hit_msg} at drone pos=({follower.true_x:.3f}, {follower.true_y:.3f}, {follower.true_z:.3f})")
            active_cmd = FlightCommand(follower.true_x, follower.true_y, max(0.0, follower.true_z - 0.2), follower.true_yaw, False, True)

    follower.step(active_cmd, dt)
    last_step_cmd = active_cmd
    return active_cmd

def _fcu_thread():
    global last_cmd_time, last_fcu_cmd
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind(("0.0.0.0", 14551))
    sock.setblocking(False)

    last_fcu_cmd = None
    dt = 0.05

    while True:
        while True:
            try:
                data, addr = sock.recvfrom(2048)
                if len(data) == ctypes.sizeof(FlightCommand):
                    last_fcu_cmd = FlightCommand.from_buffer_copy(data)
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
                break
            except Exception as e:
                print(f"FCU error: {e}")
                break

        active_cmd = step_fcu(dt, last_fcu_cmd)

        # Transmit feedback pose in Telemetry datagram to Autopilot
        with follower.lock:
            c_yaw = math.cos(follower.true_yaw)
            s_yaw = math.sin(follower.true_yaw)
            v_fwd = follower.vx * c_yaw + follower.vy * s_yaw
            v_right = follower.vx * s_yaw - follower.vy * c_yaw
            f_pitch = max(-0.35, min(0.35, v_fwd / 9.81))
            f_roll = max(-0.35, min(0.35, v_right / 9.81))
            p = Pose(follower.feedback_x, follower.feedback_y, follower.feedback_z,
                     f_roll, f_pitch, follower.feedback_yaw, time.time(), 1.0)
        telem = Telemetry(p, 92, False)
        sock.sendto(bytes(telem), ("127.0.0.1", 14550))
        time.sleep(dt)

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
INDEX_PATH = os.path.join(SCRIPT_DIR, "index.html")

pip_forward_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
last_vision_node_time = 0.0
vision_node_detections = []
last_cpp_autopilot_time = 0.0
cpp_autopilot_estimates = []

def autopilot_telemetry_thread():
    global last_cpp_autopilot_time, cpp_autopilot_estimates
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        sock.bind(("0.0.0.0", 14554))
    except Exception as e:
        print(f"Autopilot telemetry bind warning: {e}")
        return
    sock.settimeout(0.5)
    while True:
        try:
            data, _ = sock.recvfrom(2048)
            if len(data) == ctypes.sizeof(TelemetryPacket):
                pkt = TelemetryPacket.from_buffer_copy(data)
                last_cpp_autopilot_time = time.time()
                ests = []
                for g in pkt.goals:
                    ests.append({
                        "shape": g.shape_type,
                        "state": g.state,
                        "state_name": ["SEARCHING", "CONVERGING", "LOCKED", "OCCLUDED (MEMORY)"][min(3, g.state)],
                        "x": round(g.world_position.x, 3),
                        "y": round(g.world_position.y, 3),
                        "z": round(g.world_position.z, 3),
                        "variance": round(g.spatial_variance, 4),
                        "confidence": 0.95 if g.state == 2 else (0.80 if g.state == 3 else (0.50 if g.state == 1 else 0.0)),
                        "age": 0.0,
                        "is_confident": (g.state == 2 or g.state == 3)
                    })
                with state_lock:
                    cpp_autopilot_estimates = ests
        except socket.timeout:
            continue
        except Exception:
            time.sleep(0.1)

def vision_feedback_thread():
    global last_vision_node_time, vision_node_detections
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        sock.bind(("0.0.0.0", 14556))
    except Exception as e:
        print(f"Vision feedback bind warning: {e}")
        return
    sock.settimeout(0.5)
    while True:
        try:
            data, _ = sock.recvfrom(2048)
            if len(data) == ctypes.sizeof(VisionPacket):
                pkt = VisionPacket.from_buffer_copy(data)
                last_vision_node_time = time.time()
                dets = []
                if pkt.num_goals_reported > 0:
                    for i in range(pkt.num_goals_reported):
                        gm = pkt.goals[i]
                        if gm.is_detected:
                            dets.append({
                                "shape": gm.shape_type,
                                "u": gm.centroid.u,
                                "v": gm.centroid.v,
                                "x": gm.hypotheses[0].position_cam.x,
                                "y": gm.hypotheses[0].position_cam.y,
                                "z": gm.hypotheses[0].position_cam.z,
                                "conf": gm.hypotheses[0].confidence,
                                "box_size": int(gm.contour_area) if gm.contour_area > 0 else 50
                            })
                with state_lock:
                    vision_node_detections = dets
        except socket.timeout:
            continue
        except Exception:
            time.sleep(0.1)

threading.Thread(target=autopilot_telemetry_thread, daemon=True).start()
threading.Thread(target=vision_feedback_thread, daemon=True).start()

backend_procs = []

def start_backend_nodes():
    script_dir = os.path.dirname(os.path.abspath(__file__))
    autopilot_dir = os.path.dirname(script_dir)
    repo_dir = os.path.dirname(autopilot_dir)

    vision_bin = os.path.join(repo_dir, "vision-processing", "vision_node")
    autopilot_bin = os.path.join(autopilot_dir, "autopilot_node_bin")

    # Start vision_node if executable exists
    if os.path.isfile(vision_bin) and os.access(vision_bin, os.X_OK):
        try:
            p_v = subprocess.Popen([vision_bin, "--sim", "--headless"])
            backend_procs.append(p_v)
            print(f"[SIM] Started C++ Vision Node (PID {p_v.pid}): Listening for piped frames on UDP 14555.")
        except Exception as e:
            print(f"[SIM] Note: Could not auto-launch vision_node: {e}")

    # Start autopilot_node_bin if executable exists
    if os.path.isfile(autopilot_bin) and os.access(autopilot_bin, os.X_OK):
        try:
            p_a = subprocess.Popen([autopilot_bin])
            backend_procs.append(p_a)
            print(f"[SIM] Started C++ Autopilot Node (PID {p_a.pid}): Real GoalEstimator & PathPlanner active.")
        except Exception as e:
            print(f"[SIM] Note: Could not auto-launch autopilot_node_bin: {e}")

def ensure_backend_nodes():
    script_dir = os.path.dirname(os.path.abspath(__file__))
    autopilot_dir = os.path.dirname(script_dir)
    repo_dir = os.path.dirname(autopilot_dir)

    vision_bin = os.path.join(repo_dir, "vision-processing", "vision_node")
    autopilot_bin = os.path.join(autopilot_dir, "autopilot_node_bin")

    v_running = any(p.poll() is None and "vision_node" in str(p.args) for p in backend_procs)
    if not v_running and os.path.isfile(vision_bin) and os.access(vision_bin, os.X_OK):
        try:
            p_v = subprocess.Popen([vision_bin, "--sim", "--headless"])
            backend_procs.append(p_v)
            print(f"[SIM] Restarted C++ Vision Node (PID {p_v.pid}).")
        except Exception as e:
            print(f"[SIM] Note: Could not restart vision_node: {e}")

    a_running = any(p.poll() is None and "autopilot_node_bin" in str(p.args) for p in backend_procs)
    if not a_running and os.path.isfile(autopilot_bin) and os.access(autopilot_bin, os.X_OK):
        try:
            p_a = subprocess.Popen([autopilot_bin])
            backend_procs.append(p_a)
            print(f"[SIM] Restarted C++ Autopilot Node (PID {p_a.pid}).")
        except Exception as e:
            print(f"[SIM] Note: Could not restart autopilot_node_bin: {e}")

def stop_backend_nodes():
    for p in backend_procs:
        try:
            p.terminate()
            p.wait(timeout=1.0)
        except Exception:
            try:
                p.kill()
            except Exception:
                pass

atexit.register(stop_backend_nodes)

def vision_thread():
    global current_detections, current_tag_detections
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    seq_id = 0
    while True:
        now = time.time()
        # If the real C++ vision node is active, do not inject synthetic vision packets
        if (now - last_vision_node_time) < 1.5:
            time.sleep(0.05)
            continue

        with state_lock:
            enabled = vision_enabled
            goals = list(vision_goals)
            tags = list(vision_tags)

        dets = []
        tag_dets = []
        if enabled:
            with follower.lock:
                tx, ty, tz, tyaw = follower.true_x, follower.true_y, follower.true_z, follower.true_yaw

            dets = evaluate_camera_vision(tx, ty, tz, tyaw, goals)
            tag_dets = evaluate_camera_tags(tx, ty, tz, tyaw, tags)

            with state_lock:
                current_detections = list(dets)
                current_tag_detections = list(tag_dets)

            # Synthetic fallback packet only sent if real vision_node is offline
            pkt = VisionPacket()
            pkt.msg_type = 1 # MSG_TYPE_VISION
            pkt.sequence_id = seq_id
            seq_id += 1
            pkt.timestamp_ns = int(now * 1e9)

            # Goals reported in VisionPacket
            pkt.num_goals_reported = min(len(goals), 3)
            for i, g in enumerate(goals[:3]):
                gm = pkt.goals[i]
                shape_type = int(g.get("shape", g.get("id", i + 1)))
                gm.shape_type = shape_type

                matching_det = next((d for d in dets if d["shape"] == shape_type), None)
                if matching_det is not None:
                    gm.is_detected = 1
                    gm.centroid.u = float(matching_det["u"])
                    gm.centroid.v = float(matching_det["v"])
                    gm.centroid.x = float(matching_det["u"])
                    gm.centroid.y = float(matching_det["v"])
                    gm.num_hypotheses = 1
                    cam_fwd = float(matching_det.get("fwd", matching_det["dist"]))
                    cam_lat = float(matching_det.get("lat", 0.0))
                    cam_vert = float(matching_det.get("vert", 0.0))
                    gm.hypotheses[0].position_cam = Vec3f(cam_lat, -cam_vert, cam_fwd)
                    gm.hypotheses[0].confidence = float(matching_det["conf"])
                    yaw = float(matching_det["yaw"])
                    gm.hypotheses[0].orientation_cam = Quatf(math.cos(yaw / 2.0), 0.0, 0.0, math.sin(yaw / 2.0))
                else:
                    gm.is_detected = 0
                    gm.num_hypotheses = 0

            # Global Tag Targets: populate detected tags
            pkt.num_tags_detected = min(len(tag_dets), 4)
            for i, td in enumerate(tag_dets[:4]):
                tm = pkt.tags[i]
                tm.tag_id = int(td["id"])
                tm.is_valid = 1
                tm.camera_pos_world = Vec3f(float(td.get("cam_x", td["x"])), float(td.get("cam_y", td["y"])), float(td.get("cam_z", td["z"])))
                yaw = float(td.get("cam_yaw", td.get("yaw", 0.0)))
                tm.camera_att_world = Quatf(math.cos(yaw / 2.0), 0.0, 0.0, math.sin(yaw / 2.0))
                tm.confidence = float(td.get("conf", 0.95))
            for i in range(len(tag_dets), 4):
                pkt.tags[i].is_valid = 0

            sock.sendto(bytes(pkt), ("127.0.0.1", 14553))
        else:
            with state_lock:
                current_detections = []
                current_tag_detections = []

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
    global vision_enabled, vision_goals, vision_tags, global_detections, cmd_counter, last_fcu_cmd
    global mission_status_text, collision_state

    async def send_state():
        while True:
            try:
                sim_state = follower.get_state()
                now = time.time()
                with state_lock:
                    sim_state["goals"] = vision_goals
                    sim_state["tags"] = vision_tags
                    sim_state["vision_enabled"] = vision_enabled
                    sim_state["status_text"] = mission_status_text
                    sim_state["collision"] = dict(collision_state)

                    # Authoritative detections from real C++ vision node
                    if (now - last_vision_node_time) < 2.0:
                        sim_state["detections"] = list(vision_node_detections)
                    else:
                        sim_state["detections"] = list(current_detections)

                    sim_state["tag_detections"] = list(current_tag_detections)

                    # Authoritative goal estimates from real C++ GoalEstimator in AutopilotNode
                    if (now - last_cpp_autopilot_time) < 2.0:
                        sim_state["estimator"] = list(cpp_autopilot_estimates)
                    else:
                        sim_state["estimator"] = [
                            {"shape": 1, "state": 0, "state_name": "SEARCHING", "x": 0.0, "y": 0.0, "z": 0.0, "yaw": 0.0, "confidence": 0.0, "is_confident": False},
                            {"shape": 2, "state": 0, "state_name": "SEARCHING", "x": 0.0, "y": 0.0, "z": 0.0, "yaw": 0.0, "confidence": 0.0, "is_confident": False},
                            {"shape": 3, "state": 0, "state_name": "SEARCHING", "x": 0.0, "y": 0.0, "z": 0.0, "yaw": 0.0, "confidence": 0.0, "is_confident": False}
                        ]

                    sim_state["autopilot_connected"] = (now - last_cmd_time < 1.0)
                await websocket.send(json.dumps(sim_state))
            except Exception:
                break
            await asyncio.sleep(0.05) # 20Hz telemetry

    task = asyncio.create_task(send_state())

    try:
        async for message in websocket:
            data = json.loads(message)
            msg_type = data.get("type")

            if msg_type == "pip_frame":
                raw_b64 = data.get("frame", "")
                if "," in raw_b64:
                    raw_b64 = raw_b64.split(",")[-1]
                if raw_b64:
                    try:
                        jpeg_bytes = base64.b64decode(raw_b64)
                        pip_forward_sock.sendto(jpeg_bytes, ("127.0.0.1", 14555))
                    except Exception:
                        pass

            elif msg_type == "ui":
                cmd_val = data.get("command", 3)
                goals_to_send = data.get("goals", vision_goals)
                if cmd_val in (2, 7): # DISARM or ESTOP
                    with state_lock:
                        mission_status_text = "EMERGENCY STOPPED" if cmd_val == 7 else "DISARMED"
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
                last_fcu_cmd = None
                with state_lock:
                    mission_status_text = "STANDBY (Ready)"
                    collision_state["collision"] = False
                    collision_state["object"] = ""
                    collision_state["time"] = 0.0
                    current_detections.clear()
                    current_tag_detections.clear()
                    vision_node_detections.clear()
                with follower.lock:
                    follower.true_x = 0.0
                    follower.true_y = 0.0
                    follower.true_z = 1.0
                    follower.true_yaw = 0.0
                    follower.feedback_x = 0.0
                    follower.feedback_y = 0.0
                    follower.feedback_z = 1.0
                    follower.feedback_yaw = 0.0
                    follower.vx = 0.0; follower.vy = 0.0; follower.vz = 0.0
                    follower.total_samples = 0
                    follower.sum_sq_err = 0.0
                    follower.max_error = 0.0
                    follower.current_error = 0.0
                    follower.total_dist = 0.0
                    follower.last_target = {"x": 0.0, "y": 0.0, "z": 1.0, "yaw": 0.0}

                # Reset AutopilotNode internal state, GoalEstimator, and PathPlanner via UI_CMD_DISARM (2)
                try:
                    reset_cmd = UICommandPacket()
                    reset_cmd.msg_type = 2 # MSG_TYPE_UI_CMD
                    cmd_counter += 1
                    reset_cmd.command_id = cmd_counter
                    reset_cmd.timestamp_ns = int(time.time() * 1e9)
                    reset_cmd.command = 2 # UI_CMD_DISARM
                    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
                    sock.sendto(bytes(reset_cmd), ("127.0.0.1", 14552))
                    sock.close()
                except Exception:
                    pass

                ensure_backend_nodes()

            elif msg_type == "toggle":
                with state_lock:
                    vision_enabled = data["enabled"]

            elif msg_type == "update_goals":
                with state_lock:
                    vision_goals = data["goals"]
                try:
                    cmd = UICommandPacket()
                    cmd.msg_type = 2
                    cmd_counter += 1
                    cmd.command_id = cmd_counter
                    cmd.timestamp_ns = int(time.time() * 1e9)
                    cmd.command = 8 # UI_CMD_SET_SEQUENCE (8)
                    cmd.sequence_length = min(len(vision_goals), 16)
                    for i, g in enumerate(vision_goals[:16]):
                        cmd.goal_sequence[i] = g.get("shape", g.get("id", i + 1))
                    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
                    sock.sendto(bytes(cmd), ("127.0.0.1", 14552))
                    sock.close()
                except Exception:
                    pass

            elif msg_type == "update_tags":
                with state_lock:
                    vision_tags = data["tags"]

    except Exception:
        pass
    finally:
        task.cancel()

async def main(port=5000):
    print(f"Autopilot Simulation Server running at http://localhost:{port}")
    async with serve(handler, "", port, process_request=process_request):
        await asyncio.Future()

if __name__ == '__main__':
    port = 5000
    for i, arg in enumerate(sys.argv):
        if arg.startswith("--port="):
            port = int(arg.split("=")[1])
        elif arg == "--port" and i + 1 < len(sys.argv):
            port = int(sys.argv[i + 1])

    if "--no-backend" not in sys.argv:
        start_backend_nodes()
    t1 = threading.Thread(target=fcu_thread, daemon=True)
    t2 = threading.Thread(target=vision_thread, daemon=True)
    t1.start()
    t2.start()
    try:
        asyncio.run(main(port))
    except Exception as e:
        print(f"MAIN FATAL ERROR: {repr(e)}")
    finally:
        stop_backend_nodes()
