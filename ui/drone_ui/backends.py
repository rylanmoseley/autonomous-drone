"""Where the GUI's data comes from.

SimBackend flies a pretend mission so the GUI can be demoed without hardware.
AutopilotBackend talks to the real autopilot over UDP using the shared packet
formats; anything the autopilot doesn't report stays None and shows as a dash.
"""
import math
import random
import time
from dataclasses import replace

from PySide6.QtCore import QObject, QTimer, Signal
from PySide6.QtNetwork import QHostAddress, QUdpSocket

from drone_ui.model import ACTIVE_PHASES, DroneState, LogLevel, Phase, Shape
from drone_ui.protocol import (
    AUTOPILOT_TELEMETRY_PORT, AUTOPILOT_UI_PORT, FlightState, TelemetryPacket, TrackState,
    UICommand, UIStatusPacket, build_command, parse_packet, quat_to_euler_deg,
)


class DroneBackend(QObject):
    state_changed = Signal(object)  # DroneState
    log = Signal(object, str)  # LogLevel, message
    sequence_confirmed = Signal(bool)

    def confirm_sequence(self, shapes):
        raise NotImplementedError

    def start_mission(self):
        raise NotImplementedError

    def emergency_stop(self):
        raise NotImplementedError

    def shutdown(self):
        pass


# --------------------------------------------------------------------------------------
# Simulator

TICK_S = 0.1
CRUISE_Z = 1.5
CRUISE_SPEED = 0.8  # slow enough for an audience to follow
CLIMB_SPEED = 0.6
YAW_RATE = 120.0  # deg/s
HOOP_X = 4.0
HOOP_CLEARANCE = 1.0  # approach / exit distance either side of a hoop
HOOP_Y = {1: -1.6, 2: 0.0, 3: 1.6}  # Shape value -> lateral position, as in the arena photo
LANDING_PAD = (0.0, 0.0)  # the drone takes off from and lands on the same pad every time


def hoop_position(shape):
    return (HOOP_X, HOOP_Y[int(shape)], CRUISE_Z)


class SimBackend(DroneBackend):
    def __init__(self, parent=None, autostart=True):
        super().__init__(parent)
        self.state = DroneState(connected=True, link_label="Simulator", x=0.0, y=0.0, z=0.0,
                                roll=0.0, pitch=0.0, yaw=0.0, speed=0.0, battery=100.0, rssi=-48)
        self._confirmed = ()
        self._leg = 0  # 0 = to approach point, 1 = through the hoop
        self._approach = self._exit = None
        self.timer = QTimer(self, interval=int(TICK_S * 1000), timeout=self.tick)
        if autostart:
            self.timer.start()
            QTimer.singleShot(0, lambda: self.log.emit(LogLevel.OK, "Connected to simulator"))

    # Commands ---------------------------------------------------------------------------

    def confirm_sequence(self, shapes):
        s = self.state
        if s.phase in ACTIVE_PHASES:
            self.sequence_confirmed.emit(False)
            return
        if s.battery < 25:
            s.battery = 100.0
            self.log.emit(LogLevel.INFO, "Fresh battery installed")
        if math.hypot(s.x - LANDING_PAD[0], s.y - LANDING_PAD[1]) > 0.05 or s.z > 0.05:
            # After an emergency stop someone carries the drone back to the pad
            s.x, s.y, s.z, s.yaw, s.speed = *LANDING_PAD, 0.0, 0.0, 0.0
            self.log.emit(LogLevel.INFO, "Drone placed back on the landing pad")
        self._confirmed = tuple(shapes)
        s.phase, s.sequence, s.target_index, s.mission_time = Phase.READY, self._confirmed, None, None
        names = " → ".join(sh.label for sh in shapes)
        self.log.emit(LogLevel.OK, f"Sequence received: {names}")
        self.sequence_confirmed.emit(True)
        self._publish()

    def start_mission(self):
        s = self.state
        if not self._confirmed or s.phase != Phase.READY:
            return
        s.phase, s.mission_time, s.target_index = Phase.TAKEOFF, 0.0, None
        self.log.emit(LogLevel.OK, "Mission started")
        self.log.emit(LogLevel.INFO, "Taking off")
        self._publish()

    def emergency_stop(self):
        s = self.state
        if s.phase == Phase.ESTOP:
            return
        s.phase = Phase.ESTOP
        self._confirmed = ()
        self.log.emit(LogLevel.ERROR, "EMERGENCY STOP - motors off")
        self._publish()

    # Simulation -------------------------------------------------------------------------

    def tick(self):
        s = self.state
        dt = TICK_S
        s.rssi = max(-70, min(-38, s.rssi + random.choice((-1, 0, 0, 1))))
        if s.phase in ACTIVE_PHASES:
            s.mission_time += dt
            s.battery = max(0.0, s.battery - 0.25 * dt)

        if s.phase == Phase.TAKEOFF:
            if self._move_to((s.x, s.y, CRUISE_Z), dt, face_travel=False):
                s.phase, s.target_index = Phase.FLYING, 0
                self._begin_hoop()
        elif s.phase == Phase.FLYING:
            self._fly_hoop(dt)
        elif s.phase == Phase.RETURNING:
            if self._move_to((*LANDING_PAD, CRUISE_Z), dt):
                s.phase = Phase.LANDING
                self.log.emit(LogLevel.INFO, "Landing on the landing pad")
        elif s.phase == Phase.LANDING:
            if self._move_to((*LANDING_PAD, 0.0), dt, face_travel=False):
                s.phase, s.speed, s.target_index = Phase.COMPLETE, 0.0, None
                self._confirmed = ()
                self.log.emit(LogLevel.OK, "Landed safely. Mission complete! Great job!")
        elif s.phase == Phase.ESTOP:
            s.speed = 0.0
            s.z = max(0.0, s.z - 2.0 * dt)  # motors cut: it drops
            s.roll = s.pitch = 0.0
        self._publish()

    def _begin_hoop(self):
        s = self.state
        shape = s.sequence[s.target_index]
        hx, hy, hz = hoop_position(shape)
        side = 1.0 if s.x > hx else -1.0  # approach from whichever side we're on
        self._approach = (hx + side * HOOP_CLEARANCE, hy, hz)
        self._exit = (hx - side * HOOP_CLEARANCE, hy, hz)
        self._leg = 0
        n = len(s.sequence)
        self.log.emit(LogLevel.INFO, f"Navigating to {shape.label} ({s.target_index + 1}/{n})")

    def _fly_hoop(self, dt):
        s = self.state
        if self._leg == 0:
            if self._move_to(self._approach, dt):
                self._leg = 1
        elif self._move_to(self._exit, dt):
            shape = s.sequence[s.target_index]
            self.log.emit(LogLevel.OK, f"Flew through the {shape.label} ({s.target_index + 1}/{len(s.sequence)})")
            s.target_index += 1
            if s.target_index < len(s.sequence):
                self._begin_hoop()
            else:
                s.phase, s.target_index = Phase.RETURNING, None
                self.log.emit(LogLevel.INFO, "All shapes done - heading back to the landing pad")

    def _move_to(self, target, dt, face_travel=True):
        """Step toward target; returns True on arrival."""
        s = self.state
        dx, dy, dz = target[0] - s.x, target[1] - s.y, target[2] - s.z
        horiz = math.hypot(dx, dy)

        if face_travel and horiz > 0.05:
            want = math.degrees(math.atan2(dy, dx))
            err = (want - s.yaw + 180.0) % 360.0 - 180.0
            s.yaw += max(-YAW_RATE * dt, min(YAW_RATE * dt, err))
            s.yaw = (s.yaw + 180.0) % 360.0 - 180.0

        step_h = min(horiz, CRUISE_SPEED * dt)
        step_v = max(-CLIMB_SPEED * dt, min(CLIMB_SPEED * dt, dz))
        if horiz > 0:
            s.x += dx / horiz * step_h
            s.y += dy / horiz * step_h
        s.z += step_v
        s.speed = math.hypot(step_h, step_v) / dt
        s.pitch = -4.0 * step_h / (CRUISE_SPEED * dt) + random.uniform(-0.3, 0.3)
        s.roll = random.uniform(-0.8, 0.8)
        return horiz < 0.02 and abs(dz) < 0.02

    def _publish(self):
        self.state_changed.emit(replace(self.state))

    def shutdown(self):
        self.timer.stop()


# --------------------------------------------------------------------------------------
# Real autopilot

# FlightState -> what the GUI shows. HOVER_WAIT only happens on a pause or a HOVER end
# action; the GUI always asks to LAND at the end, so it's shown as still flying.
FLIGHT_PHASES = {
    FlightState.BOOT: Phase.READY,
    FlightState.STANDBY_DISARM: Phase.READY,
    FlightState.ARMED: Phase.READY,
    FlightState.TAKEOFF: Phase.TAKEOFF,
    FlightState.GOAL_SEEK: Phase.FLYING,
    FlightState.PATH_FOLLOWING: Phase.FLYING,
    FlightState.HOVER_WAIT: Phase.FLYING,
    FlightState.LANDING: Phase.LANDING,
    FlightState.ROTOR_STOPPED: Phase.READY,
    FlightState.ESTOP_TRIPPED: Phase.ESTOP,
}
POLL_MS = 250  # the autopilot only sends status as a reply, so keep asking for it
ACK_TIMEOUT_S = 0.6
STALE_AFTER_S = 1.0


class AutopilotBackend(DroneBackend):
    """UICommandPackets go to the autopilot's UI port and each one is answered with a
    UIStatusPacket. TelemetryPackets arrive on the telemetry port."""

    def __init__(self, host="127.0.0.1", port=AUTOPILOT_UI_PORT,
                 telemetry_port=AUTOPILOT_TELEMETRY_PORT, parent=None):
        super().__init__(parent)
        self.addr = (QHostAddress(host), port)
        self.state = DroneState(link_label=f"Autopilot {host}")
        self._next_id = 1
        self._pending = {}  # command_id -> (on_result, sent_at)
        self._last_status = 0.0
        self._last_pos = None  # (timestamp_ns, (x, y, z)) for speed when velocity is unset
        self._mission_start = None
        self._was_active = False
        self._flight_state = None
        self._seq_index = None
        self._tracks = {}

        self.cmd_sock = QUdpSocket(self)
        self.cmd_sock.bind(QHostAddress.AnyIPv4, 0)
        self.cmd_sock.readyRead.connect(self._read_status)

        self.telem_sock = QUdpSocket(self)
        if not self.telem_sock.bind(QHostAddress.AnyIPv4, telemetry_port):
            QTimer.singleShot(0, lambda: self.log.emit(
                LogLevel.ERROR, f"Can't listen for telemetry on port {telemetry_port}"))
        self.telem_sock.readyRead.connect(self._read_telemetry)

        self.timer = QTimer(self, interval=POLL_MS, timeout=self._poll)
        self.timer.start()

    # Commands ---------------------------------------------------------------------------

    def _send(self, command, shapes=(), on_result=None):
        cid = self._next_id
        self._next_id += 1
        self.cmd_sock.writeDatagram(bytes(build_command(cid, command, shapes)), *self.addr)
        if on_result:
            self._pending[cid] = (on_result, time.monotonic())
        return cid

    def confirm_sequence(self, shapes):
        shapes = tuple(shapes)

        def done(ok):
            if ok:
                self.state.sequence = shapes
                self.log.emit(LogLevel.OK, "Sequence received: " + " → ".join(s.label for s in shapes))
            else:
                self.log.emit(LogLevel.WARN, "Autopilot didn't accept the order - is it running?")
            self.sequence_confirmed.emit(ok)
        self._send(UICommand.SET_SEQUENCE, shapes, done)

    def start_mission(self):
        def done(ok):
            self.log.emit(LogLevel.OK if ok else LogLevel.WARN,
                          "Mission started" if ok else "Autopilot didn't accept Start")
        # START_MISSION carries the sequence too, so the autopilot flies exactly what's shown
        self._send(UICommand.START_MISSION, self.state.sequence, done)

    def emergency_stop(self):
        self.log.emit(LogLevel.ERROR, "EMERGENCY STOP sent")

        def done(ok):
            if not ok:
                self.log.emit(LogLevel.ERROR, "Autopilot didn't confirm the emergency stop!")
        for _ in range(3):  # UDP can drop packets; repeats are harmless
            cid = self._send(UICommand.ESTOP)
        self._pending[cid] = (done, time.monotonic())

    # Incoming ---------------------------------------------------------------------------

    def _poll(self):
        self._send(UICommand.NONE)
        now = time.monotonic()
        for cid, (on_result, sent) in list(self._pending.items()):
            if now - sent > ACK_TIMEOUT_S:
                del self._pending[cid]
                on_result(False)
        connected = now - self._last_status < STALE_AFTER_S
        if connected != self.state.connected:
            self.state.connected = connected
            self.log.emit(LogLevel.OK if connected else LogLevel.WARN,
                          "Connected to autopilot" if connected else "Lost contact with autopilot")
            self.state_changed.emit(replace(self.state))

    def _read_status(self):
        while self.cmd_sock.hasPendingDatagrams():
            pkt = parse_packet(self.cmd_sock.receiveDatagram().data().data())
            if isinstance(pkt, UIStatusPacket):
                self.handle_status(pkt)

    def _read_telemetry(self):
        while self.telem_sock.hasPendingDatagrams():
            pkt = parse_packet(self.telem_sock.receiveDatagram().data().data())
            if isinstance(pkt, TelemetryPacket):
                self.handle_telemetry(pkt)

    def handle_status(self, pkt):
        s = self.state
        self._last_status = time.monotonic()
        if not s.connected:
            s.connected = True
            self.log.emit(LogLevel.OK, "Connected to autopilot")

        pending = self._pending.pop(pkt.last_acked_cmd_id, None)
        if pending:
            pending[0](pkt.cmd_ack_result == 0)

        try:
            flight = FlightState(pkt.flight_state)
        except ValueError:
            flight = None
        phase = FLIGHT_PHASES.get(flight, Phase.ERROR)
        active = phase in ACTIVE_PHASES
        if flight == FlightState.ROTOR_STOPPED and self._was_active:
            phase = Phase.COMPLETE
            self.log.emit(LogLevel.OK, "Landed safely. Mission complete! Great job!")
        elif phase == Phase.READY and s.phase == Phase.COMPLETE:
            phase = Phase.COMPLETE  # keep showing it until the next mission
        if flight != self._flight_state:
            self._log_transition(flight, pkt)
        self._flight_state = flight

        if active and self._mission_start is None:
            self._mission_start = time.monotonic()
        elif not active:
            self._mission_start = None
        self._was_active = active
        s.phase = phase
        s.mission_time = None if self._mission_start is None else time.monotonic() - self._mission_start

        index = pkt.current_seq_index
        if phase == Phase.FLYING and s.sequence and index != self._seq_index:
            if self._seq_index is not None and index > self._seq_index and self._seq_index < len(s.sequence):
                done = s.sequence[self._seq_index]
                self.log.emit(LogLevel.OK, f"Flew through the {done.label} ({self._seq_index + 1}/{len(s.sequence)})")
        self._seq_index = index if active else None
        s.target_index = index if phase == Phase.FLYING and index < len(s.sequence) else None
        s.battery = pkt.battery_remaining
        self.state_changed.emit(replace(s))

    def _log_transition(self, flight, pkt):
        s = self.state
        target = s.sequence[pkt.current_seq_index] if pkt.current_seq_index < len(s.sequence) else None
        if flight == FlightState.TAKEOFF:
            self.log.emit(LogLevel.INFO, "Taking off")
        elif flight == FlightState.GOAL_SEEK and target:
            self.log.emit(LogLevel.INFO, f"Looking for the {target.label}…")
        elif flight == FlightState.PATH_FOLLOWING and target:
            self.log.emit(LogLevel.INFO, f"Found the {target.label}! Flying to it "
                                         f"({pkt.current_seq_index + 1}/{len(s.sequence)})")
        elif flight == FlightState.LANDING:
            self.log.emit(LogLevel.INFO, "Landing")
        elif flight == FlightState.ESTOP_TRIPPED:
            self.log.emit(LogLevel.ERROR, "Emergency stop is active - motors off")

    def handle_telemetry(self, pkt):
        s = self.state
        p, v = pkt.drone_pos_ned, pkt.drone_vel_ned
        s.x, s.y = p.x, p.y
        s.z = abs(p.z)  # shown as altitude whichever way up the autopilot reports z
        speed = math.sqrt(v.x * v.x + v.y * v.y + v.z * v.z)
        if speed == 0.0 and self._last_pos is not None:
            # Velocity isn't filled in yet: estimate it from successive positions
            t0, (x0, y0, z0) = self._last_pos
            dt = (pkt.timestamp_ns - t0) / 1e9
            if dt > 0:
                speed = math.sqrt((p.x - x0) ** 2 + (p.y - y0) ** 2 + (p.z - z0) ** 2) / dt
        self._last_pos = (pkt.timestamp_ns, (p.x, p.y, p.z))
        s.speed = speed
        euler = quat_to_euler_deg(pkt.drone_att_ned)
        s.roll, s.pitch, s.yaw = euler if euler else (None, None, None)

        for goal in pkt.goals:
            try:
                shape = Shape(goal.shape_type)
            except ValueError:
                continue
            seen = goal.state in (TrackState.CONVERGING, TrackState.LOCKED)
            if seen and not self._tracks.get(shape, False):
                self.log.emit(LogLevel.INFO, f"Camera spotted the {shape.label}")
            self._tracks[shape] = seen
        self.state_changed.emit(replace(s))

    def shutdown(self):
        self.timer.stop()
        self.cmd_sock.close()
        self.telem_sock.close()
