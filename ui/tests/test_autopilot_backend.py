"""AutopilotBackend against a fake autopilot that answers like AutopilotNode::handleUICommand."""
import ctypes
import os
import socket
import threading
import time

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

import pytest  # noqa: E402
from PySide6.QtWidgets import QApplication  # noqa: E402

from drone_ui.backends import AutopilotBackend  # noqa: E402
from drone_ui.model import Phase, Shape  # noqa: E402
from drone_ui.protocol import (  # noqa: E402
    FlightState, MsgType, Quatf, TelemetryPacket, TrackState, UICommand, UICommandPacket,
    UIStatusPacket, Vec3f,
)

app = QApplication.instance() or QApplication([])


class FakeAutopilot:
    def __init__(self):
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind(("127.0.0.1", 0))
        self.sock.settimeout(0.05)
        self.port = self.sock.getsockname()[1]
        self.state = FlightState.STANDBY_DISARM
        self.seq_index = 0
        self.sequence = []
        self.commands = []
        self.reject = False
        self.running = True
        self.thread = threading.Thread(target=self._serve, daemon=True)
        self.thread.start()

    def _serve(self):
        while self.running:
            try:
                data, addr = self.sock.recvfrom(1024)
            except (socket.timeout, OSError):
                continue
            if len(data) != ctypes.sizeof(UICommandPacket):  # the autopilot drops these too
                continue
            pkt = UICommandPacket.from_buffer_copy(data)
            cmd = UICommand(pkt.command)
            self.commands.append(cmd)
            if cmd in (UICommand.SET_SEQUENCE, UICommand.START_MISSION):
                self.sequence = list(pkt.goal_sequence[:pkt.sequence_length])
            if cmd == UICommand.START_MISSION:
                self.state, self.seq_index = FlightState.TAKEOFF, 0
            elif cmd == UICommand.ESTOP:
                self.state = FlightState.ESTOP_TRIPPED
            status = UIStatusPacket(
                msg_type=MsgType.UI_STATUS, flight_state=self.state, active_sequence_len=len(self.sequence),
                current_seq_index=self.seq_index, battery_remaining=90.0,
                last_acked_cmd_id=pkt.command_id, cmd_ack_result=1 if self.reject else 0,
                estop_active=int(self.state == FlightState.ESTOP_TRIPPED))
            self.sock.sendto(bytes(status), addr)

    def close(self):
        self.running = False
        self.thread.join()
        self.sock.close()


def free_udp_port():
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    s.bind(("127.0.0.1", 0))
    port = s.getsockname()[1]
    s.close()
    return port


def pump(until, timeout=2.0):
    end = time.monotonic() + timeout
    while time.monotonic() < end:
        app.processEvents()
        if until():
            return True
        time.sleep(0.01)
    return False


@pytest.fixture
def rig():
    fake = FakeAutopilot()
    telem_port = free_udp_port()
    backend = AutopilotBackend("127.0.0.1", port=fake.port, telemetry_port=telem_port)
    logs, states, confirms = [], [], []
    backend.log.connect(lambda _lvl, msg: logs.append(msg))
    backend.state_changed.connect(states.append)
    backend.sequence_confirmed.connect(confirms.append)
    yield fake, backend, telem_port, logs, states, confirms
    backend.shutdown()
    fake.close()


def test_connects_and_runs_a_mission(rig):
    fake, backend, _, logs, states, confirms = rig
    assert pump(lambda: backend.state.connected)  # status polling with UI_CMD_NONE

    order = [Shape.SQUARE, Shape.CIRCLE, Shape.TRIANGLE]
    backend.confirm_sequence(order)
    assert pump(lambda: confirms == [True])
    assert fake.sequence == [3, 1, 2]

    backend.start_mission()
    assert pump(lambda: states and states[-1].phase == Phase.TAKEOFF)
    assert UICommand.START_MISSION in fake.commands and fake.sequence == [3, 1, 2]

    fake.state = FlightState.GOAL_SEEK
    assert pump(lambda: "Looking for the Square…" in logs)
    fake.state = FlightState.PATH_FOLLOWING
    assert pump(lambda: states[-1].target_index == 0 and states[-1].phase == Phase.FLYING)
    fake.seq_index = 1
    assert pump(lambda: "Flew through the Square (1/3)" in logs)
    fake.seq_index, fake.state = 3, FlightState.LANDING
    assert pump(lambda: states[-1].phase == Phase.LANDING)
    fake.state = FlightState.ROTOR_STOPPED
    assert pump(lambda: states[-1].phase == Phase.COMPLETE)
    assert states[-1].battery == 90.0


def test_rejected_confirm(rig):
    fake, backend, _, _, _, confirms = rig
    fake.reject = True
    backend.confirm_sequence([Shape.CIRCLE, Shape.TRIANGLE, Shape.SQUARE])
    assert pump(lambda: confirms == [False])


def test_no_autopilot_times_out():
    backend = AutopilotBackend("127.0.0.1", port=free_udp_port(), telemetry_port=free_udp_port())
    confirms = []
    backend.sequence_confirmed.connect(confirms.append)
    try:
        backend.confirm_sequence([Shape.CIRCLE])
        assert pump(lambda: confirms == [False])
        assert not backend.state.connected
    finally:
        backend.shutdown()


def test_estop(rig):
    fake, backend, _, logs, states, _ = rig
    backend.emergency_stop()
    assert pump(lambda: states and states[-1].phase == Phase.ESTOP)
    assert fake.commands.count(UICommand.ESTOP) == 3


def test_telemetry(rig):
    _, backend, telem_port, logs, states, _ = rig
    pkt = TelemetryPacket(msg_type=MsgType.TELEMETRY, timestamp_ns=1_000_000_000,
                          drone_pos_ned=Vec3f(1.0, 2.0, 1.5), drone_vel_ned=Vec3f(0.6, 0.8, 0.0),
                          drone_att_ned=Quatf(0.7071068, 0, 0, 0.7071068))
    pkt.goals[0].shape_type = Shape.CIRCLE
    pkt.goals[0].state = TrackState.LOCKED
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    s.sendto(b'{"state":1}', ("127.0.0.1", telem_port))  # JSON debug log shares the port
    s.sendto(bytes(pkt), ("127.0.0.1", telem_port))
    s.close()
    assert pump(lambda: states and states[-1].x == 1.0)
    st = states[-1]
    assert (st.y, st.z) == (2.0, 1.5) and abs(st.speed - 1.0) < 1e-6 and round(st.yaw) == 90
    assert "Camera spotted the Circle" in logs
