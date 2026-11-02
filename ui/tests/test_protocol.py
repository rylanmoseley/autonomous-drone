import ctypes
import socket

from drone_ui.protocol import (
    AutopilotLink, UIAckPacket, UIConfigPacket, UIGoalPose, UIGoalSequence, build_sequence,
    build_shape_sequence, parse_log,
)


def test_build_shape_sequence():
    seq = build_shape_sequence([3, 1, 2], start=True)
    assert seq.num_goals == 3
    assert [seq.goals[i].id for i in range(3)] == [3, 1, 2]
    assert seq.start


def test_parse_log_names_state():
    # Format produced by UdpTelemetryLogger::flush
    log = parse_log(b'{"cmd_tx":1.000000,"dt":0.050000,"state":2,"telem_x":0.500000}')
    assert log["state_name"] == "NAVIGATING"
    assert log["cmd_tx"] == 1.0


def test_struct_sizes_match_64bit_autopilot():
    # Sizes from shared/include/ui_protocol.h on a 64-bit build
    assert ctypes.sizeof(UIGoalPose) == 4 + 8 * 8
    assert ctypes.sizeof(UIGoalSequence) == 16 * 68 + 8 + 3
    assert ctypes.sizeof(UIConfigPacket) == 24
    assert ctypes.sizeof(UIAckPacket) == 1


def test_build_sequence():
    seq = build_sequence([{"id": 3, "x": 1.5, "y": -2.0, "z": 1.0, "yaw": 0.5}], start=True)
    assert seq.num_goals == 1
    assert seq.start and not seq.stop and not seq.estop
    assert (seq.goals[0].id, seq.goals[0].x, seq.goals[0].yaw) == (3, 1.5, 0.5)


def test_send_sequence_gets_ack():
    # Fake autopilot that acks like AutopilotNode::ui_loop
    server = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    server.bind(("127.0.0.1", 0))
    server.settimeout(1.0)
    link = AutopilotLink(*server.getsockname())
    try:
        import threading

        def serve():
            data, addr = server.recvfrom(4096)
            assert len(data) == ctypes.sizeof(UIGoalSequence)
            server.sendto(bytes(UIAckPacket(True)), addr)

        t = threading.Thread(target=serve)
        t.start()
        assert link.send_sequence(build_sequence([], estop=True))
        t.join()
    finally:
        link.close()
        server.close()
