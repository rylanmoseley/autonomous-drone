"""Python mirrors of shared/include/ui_protocol.h (main branch).

Keep these in sync with the C++ headers: the autopilot drops any UDP packet
whose size doesn't exactly match sizeof() of the struct it expects.
"""
import ctypes
import json
import socket

AUTOPILOT_UI_PORT = 14552
AUTOPILOT_LOG_PORT = 14554  # UdpTelemetryLogger in autopilot/telemetry_logger_impl.h
MAX_GOALS = 16

# Order of MissionState in autopilot/autopilot_node.h
MISSION_STATES = ["IDLE", "TAKEOFF", "NAVIGATING", "RETURN_TO_HOME", "LANDING", "ERROR", "ESTOP"]


class UIGoalPose(ctypes.Structure):
    _pack_ = 1
    _fields_ = [
        ("id", ctypes.c_uint32),
        ("x", ctypes.c_double), ("y", ctypes.c_double), ("z", ctypes.c_double),
        ("roll", ctypes.c_double), ("pitch", ctypes.c_double), ("yaw", ctypes.c_double),
        ("timestamp", ctypes.c_double),
        ("confidence", ctypes.c_double),
    ]


class UIGoalSequence(ctypes.Structure):
    _pack_ = 1
    _fields_ = [
        ("goals", UIGoalPose * MAX_GOALS),
        # size_t in the header: 8 bytes on a 64-bit autopilot build, 4 on the 32-bit Pi
        ("num_goals", ctypes.c_uint64),
        ("start", ctypes.c_bool),
        ("stop", ctypes.c_bool),
        ("estop", ctypes.c_bool),
    ]


class UIAckPacket(ctypes.Structure):
    _fields_ = [("received", ctypes.c_bool)]


class UIConfigPacket(ctypes.Structure):
    _pack_ = 1
    _fields_ = [
        ("max_velocity", ctypes.c_double),
        ("max_vertical_velocity", ctypes.c_double),
        ("max_yaw_rate", ctypes.c_double),
    ]


def build_sequence(goals, start=False, stop=False, estop=False):
    """goals: iterable of dicts with id, x, y, z, yaw."""
    goals = list(goals)
    if len(goals) > MAX_GOALS:
        raise ValueError(f"at most {MAX_GOALS} goals")
    seq = UIGoalSequence()
    for i, g in enumerate(goals):
        seq.goals[i] = UIGoalPose(g["id"], g["x"], g["y"], g["z"], 0.0, 0.0, g["yaw"], 0.0, 1.0)
    seq.num_goals = len(goals)
    seq.start, seq.stop, seq.estop = start, stop, estop
    return seq


def build_shape_sequence(shape_ids, **flags):
    """Goal sequence from shape ids only. The autopilot flies to where vision sees
    each shape, so positions are left at zero."""
    return build_sequence(
        ({"id": s, "x": 0.0, "y": 0.0, "z": 0.0, "yaw": 0.0} for s in shape_ids), **flags
    )


def parse_log(data):
    """Decode one logger datagram (a flat JSON object) and add a readable state name."""
    log = json.loads(data)
    state = log.get("state")
    if isinstance(state, int) and 0 <= state < len(MISSION_STATES):
        log["state_name"] = MISSION_STATES[state]
    return log


class AutopilotLink:
    """UDP link to the autopilot's UI port."""

    def __init__(self, host="127.0.0.1", port=AUTOPILOT_UI_PORT, ack_timeout=0.3):
        self.addr = (host, port)
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.settimeout(ack_timeout)

    def send_sequence(self, seq):
        """Send a goal sequence; returns True if the autopilot acknowledged it."""
        self.sock.sendto(bytes(seq), self.addr)
        try:
            data, _ = self.sock.recvfrom(64)
        except (socket.timeout, ConnectionResetError):
            return False
        return len(data) == ctypes.sizeof(UIAckPacket) and UIAckPacket.from_buffer_copy(data).received

    def send_config(self, max_velocity, max_vertical_velocity, max_yaw_rate):
        # No ack for config packets
        self.sock.sendto(bytes(UIConfigPacket(max_velocity, max_vertical_velocity, max_yaw_rate)), self.addr)

    def close(self):
        self.sock.close()
