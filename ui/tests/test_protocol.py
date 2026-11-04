import ctypes
import re
from pathlib import Path

import pytest

from drone_ui.protocol import (
    GoalTrackTelemetry, MsgType, Quatf, TelemetryPacket, UICommand, UICommandPacket,
    UIStatusPacket, Vec3f, build_command, parse_packet, quat_to_euler_deg,
)

SHARED = Path(__file__).resolve().parents[2] / "shared" / "include"
C_SIZES = {"uint8_t": 1, "uint16_t": 2, "uint32_t": 4, "uint64_t": 8, "float": 4}


def c_struct_fields(name):
    """(field, size) pairs of a packed typedef struct in shared/include, read from the
    header itself so these tests catch any change to the C side."""
    text = "\n".join(p.read_text() for p in SHARED.glob("*.h"))
    text = re.sub(r"//[^\n]*", "", text)
    m = re.search(r"typedef struct \{([^{}]*)\}\s*" + name + r"\s*;", text)
    assert m, f"{name} not found in {SHARED}"
    fields = []
    for ctype, field, count in re.findall(r"(\w+)\s+(\w+)\s*(?:\[(\w+)\])?\s*;", m.group(1)):
        size = C_SIZES.get(ctype) or sum(sz for _, sz in c_struct_fields(ctype))
        if count:
            n = int(count) if count.isdigit() else int(re.search(r"#define\s+" + count + r"\s+(\d+)", text).group(1))
            size *= n
        fields.append((field, size))
    return fields


@pytest.mark.parametrize("cls", [UICommandPacket, UIStatusPacket, TelemetryPacket,
                                 GoalTrackTelemetry, Vec3f, Quatf])
def test_structs_match_c_headers(cls):
    c_fields = c_struct_fields(cls.__name__)
    py_fields = [(name, getattr(cls, name).size) for name, _ in cls._fields_]
    assert py_fields == c_fields
    assert ctypes.sizeof(cls) == sum(sz for _, sz in c_fields)


def test_build_command_sequence():
    pkt = build_command(7, UICommand.START_MISSION, [3, 1, 2])
    assert pkt.msg_type == MsgType.UI_CMD and pkt.command_id == 7
    assert pkt.sequence_length == 3 and list(pkt.goal_sequence[:3]) == [3, 1, 2]
    assert pkt.end_action == 1  # land at the end


def test_parse_packet_ignores_json_log():
    status = UIStatusPacket(msg_type=MsgType.UI_STATUS, flight_state=4)
    assert parse_packet(bytes(status)).flight_state == 4
    assert isinstance(parse_packet(bytes(TelemetryPacket(msg_type=MsgType.TELEMETRY))), TelemetryPacket)
    assert parse_packet(b'{"state":1,"dt":0.05}') is None


def test_quaternion_to_euler():
    assert quat_to_euler_deg(Quatf(0, 0, 0, 0)) is None  # not filled in
    roll, pitch, yaw = quat_to_euler_deg(Quatf(0.7071068, 0, 0, 0.7071068))
    assert abs(roll) < 1e-3 and abs(pitch) < 1e-3 and abs(yaw - 90) < 1e-3
