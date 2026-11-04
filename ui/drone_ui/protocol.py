"""Python mirrors of the shared message formats in shared/include/.

Keep these in sync with the C headers: the autopilot drops any UDP packet whose
size doesn't exactly match sizeof() of the struct it expects.

  common_types.h     enums, Vec3f, Quatf
  ui_protocol.h      UICommandPacket (GUI -> autopilot), UIStatusPacket (reply)
  telemetry_packet.h TelemetryPacket (autopilot -> GUI, ~20 Hz)
"""
import ctypes
import math
import time
from enum import IntEnum

AUTOPILOT_UI_PORT = 14552  # UICommandPacket in, UIStatusPacket back to the sender
AUTOPILOT_TELEMETRY_PORT = 14554  # TelemetryPacket (and the JSON debug log) out
MAX_GOAL_SEQUENCE_LEN = 16


class MsgType(IntEnum):
    NONE = 0
    VISION = 1
    UI_CMD = 2
    UI_STATUS = 3
    TELEMETRY = 4
    LOGGER_STATUS = 5


class UICommand(IntEnum):
    NONE = 0  # does nothing, but the autopilot still replies with a status packet
    ARM = 1
    DISARM = 2
    START_MISSION = 3
    PAUSE_MISSION = 4
    RESUME_MISSION = 5
    LAND_NOW = 6
    ESTOP = 7
    SET_SEQUENCE = 8
    SET_CONSTRAINTS = 9


class MissionEndAction(IntEnum):
    HOVER = 0
    LAND = 1


class FlightState(IntEnum):
    BOOT = 0
    STANDBY_DISARM = 1
    ARMED = 2
    TAKEOFF = 3
    GOAL_SEEK = 4
    PATH_FOLLOWING = 5
    HOVER_WAIT = 6
    LANDING = 7
    ROTOR_STOPPED = 8
    ESTOP_TRIPPED = 9


class TrackState(IntEnum):
    SEARCHING = 0
    CONVERGING = 1
    LOCKED = 2
    OCCLUDED = 3


class Vec3f(ctypes.Structure):
    _pack_ = 1
    _fields_ = [("x", ctypes.c_float), ("y", ctypes.c_float), ("z", ctypes.c_float)]


class Quatf(ctypes.Structure):
    _pack_ = 1
    _fields_ = [("w", ctypes.c_float), ("x", ctypes.c_float), ("y", ctypes.c_float), ("z", ctypes.c_float)]


class UICommandPacket(ctypes.Structure):
    _pack_ = 1
    _fields_ = [
        ("msg_type", ctypes.c_uint8),
        ("command_id", ctypes.c_uint32),
        ("timestamp_ns", ctypes.c_uint64),
        ("command", ctypes.c_uint8),
        ("sequence_length", ctypes.c_uint8),
        ("goal_sequence", ctypes.c_uint8 * MAX_GOAL_SEQUENCE_LEN),
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
        ("cmd_ack_result", ctypes.c_uint8),  # 0 = success, 1 = rejected
    ]


class GoalTrackTelemetry(ctypes.Structure):
    _pack_ = 1
    _fields_ = [
        ("shape_type", ctypes.c_uint8),
        ("state", ctypes.c_uint8),  # TrackState
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


def build_command(command_id, command, shapes=(), end_action=MissionEndAction.LAND):
    """A UICommandPacket; shapes is the goal order as ShapeType values (1-3)."""
    shapes = [int(s) for s in shapes]
    if len(shapes) > MAX_GOAL_SEQUENCE_LEN:
        raise ValueError(f"at most {MAX_GOAL_SEQUENCE_LEN} goals")
    pkt = UICommandPacket()
    pkt.msg_type = MsgType.UI_CMD
    pkt.command_id = command_id
    pkt.timestamp_ns = time.monotonic_ns()
    pkt.command = command
    pkt.sequence_length = len(shapes)
    for i, s in enumerate(shapes):
        pkt.goal_sequence[i] = s
    pkt.end_action = end_action
    return pkt


def parse_packet(data):
    """Decode a datagram from the autopilot into a UIStatusPacket or TelemetryPacket.
    Returns None for anything else (e.g. the JSON debug log that shares the
    telemetry port)."""
    if not data:
        return None
    for cls, msg_type in ((UIStatusPacket, MsgType.UI_STATUS), (TelemetryPacket, MsgType.TELEMETRY)):
        if data[0] == msg_type and len(data) == ctypes.sizeof(cls):
            return cls.from_buffer_copy(data)
    return None


def quat_to_euler_deg(q):
    """(roll, pitch, yaw) in degrees, or None if the quaternion isn't filled in."""
    w, x, y, z = q.w, q.x, q.y, q.z
    norm = math.sqrt(w * w + x * x + y * y + z * z)
    if norm < 0.5:
        return None
    w, x, y, z = w / norm, x / norm, y / norm, z / norm
    roll = math.atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y))
    pitch = math.asin(max(-1.0, min(1.0, 2 * (w * y - z * x))))
    yaw = math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))
    return tuple(math.degrees(a) for a in (roll, pitch, yaw))
