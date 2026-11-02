"""Where the GUI's data comes from.

SimBackend flies a pretend mission so the GUI can be demoed without hardware.
AutopilotBackend talks to the real autopilot over UDP; anything the autopilot
doesn't report yet stays None and the GUI shows a dash.
"""
import math
import random
import time
from dataclasses import replace

from PySide6.QtCore import QObject, QTimer, Signal
from PySide6.QtNetwork import QHostAddress, QUdpSocket

from drone_ui.model import ACTIVE_PHASES, DroneState, LogLevel, Phase
from drone_ui.protocol import (
    AUTOPILOT_LOG_PORT, AUTOPILOT_UI_PORT, AutopilotLink, build_shape_sequence, parse_log,
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

# Order of MissionState in autopilot/autopilot_node.h
AUTOPILOT_PHASES = [Phase.READY, Phase.TAKEOFF, Phase.FLYING, Phase.RETURNING, Phase.LANDING,
                    Phase.ERROR, Phase.ESTOP]
STALE_AFTER_S = 1.0

# Logger key -> (DroneState field, scale). Today the autopilot only logs state and
# telem_x (see AutopilotNode::run); add logger.log(...) calls there with these names
# and the values show up in the GUI with no other changes.
LOG_FIELDS = {
    "telem_x": ("x", 1.0),
    "telem_y": ("y", 1.0),
    "telem_z": ("z", 1.0),
    "telem_roll": ("roll", 180.0 / math.pi),  # autopilot uses radians
    "telem_pitch": ("pitch", 180.0 / math.pi),
    "telem_yaw": ("yaw", 180.0 / math.pi),
    "speed": ("speed", 1.0),
    "battery": ("battery", 1.0),
    "rssi": ("rssi", 1),
}


class AutopilotBackend(DroneBackend):
    def __init__(self, host="127.0.0.1", port=AUTOPILOT_UI_PORT, parent=None):
        super().__init__(parent)
        self.link = AutopilotLink(host, port)
        self.state = DroneState(link_label=f"Autopilot {host}")
        self._last_rx = 0.0
        self._mission_start = None
        self._was_active = False

        self.sock = QUdpSocket(self)
        if not self.sock.bind(QHostAddress.AnyIPv4, AUTOPILOT_LOG_PORT):
            QTimer.singleShot(0, lambda: self.log.emit(
                LogLevel.ERROR, f"Can't listen for autopilot status on port {AUTOPILOT_LOG_PORT}"))
        self.sock.readyRead.connect(self._read)
        self.timer = QTimer(self, interval=250, timeout=self._check_link)
        self.timer.start()

    def confirm_sequence(self, shapes):
        acked = self.link.send_sequence(build_shape_sequence(shapes))
        if acked:
            self.state.sequence = tuple(shapes)
            self.log.emit(LogLevel.OK, "Sequence received: " + " → ".join(s.label for s in shapes))
        else:
            self.log.emit(LogLevel.WARN, "Autopilot didn't answer - is it running?")
        self.sequence_confirmed.emit(acked)

    def start_mission(self):
        if self.link.send_sequence(build_shape_sequence(self.state.sequence, start=True)):
            self.log.emit(LogLevel.OK, "Mission started")
        else:
            self.log.emit(LogLevel.WARN, "Start not acknowledged")

    def emergency_stop(self):
        acked = self.link.send_sequence(build_shape_sequence(self.state.sequence, estop=True))
        self.log.emit(LogLevel.ERROR, "EMERGENCY STOP sent" + ("" if acked else " (no ack!)"))

    def _read(self):
        while self.sock.hasPendingDatagrams():
            try:
                log = parse_log(self.sock.receiveDatagram().data().data())
            except ValueError:
                continue
            self._last_rx = time.monotonic()
            self.update_from_log(log)

    def update_from_log(self, log):
        s = self.state
        if not s.connected:
            self.log.emit(LogLevel.OK, "Connected to autopilot")
        s.connected = True
        state = log.get("state")
        if isinstance(state, int) and 0 <= state < len(AUTOPILOT_PHASES):
            phase = AUTOPILOT_PHASES[state]
            if phase == Phase.READY and self._was_active:
                phase = Phase.COMPLETE
                self.log.emit(LogLevel.OK, "Mission complete!")
            elif phase == Phase.READY and s.phase == Phase.COMPLETE:
                phase = Phase.COMPLETE  # keep showing it until the next mission
            if phase != s.phase and phase in (Phase.ERROR, Phase.ESTOP):
                self.log.emit(LogLevel.ERROR, f"Autopilot reports {phase.value}")
            active = phase in ACTIVE_PHASES
            if active and self._mission_start is None:
                self._mission_start = time.monotonic()
            elif not active:
                self._mission_start = None
            self._was_active = active
            s.phase = phase
        if self._mission_start is not None:
            s.mission_time = time.monotonic() - self._mission_start

        for key, (field, scale) in LOG_FIELDS.items():
            if key in log:
                setattr(s, field, log[key] * scale)
        if "goal_index" in log:
            index = int(log["goal_index"])
            if s.phase == Phase.FLYING and index != s.target_index and index < len(s.sequence):
                shape = s.sequence[index]
                self.log.emit(LogLevel.INFO, f"Navigating to {shape.label} ({index + 1}/{len(s.sequence)})")
            s.target_index = index if s.phase == Phase.FLYING else None
        self.state_changed.emit(replace(s))

    def _check_link(self):
        connected = time.monotonic() - self._last_rx < STALE_AFTER_S
        if connected != self.state.connected:
            self.state.connected = connected
            if not connected:
                self.log.emit(LogLevel.WARN, "Lost contact with autopilot")
            self.state_changed.emit(replace(self.state))

    def shutdown(self):
        self.timer.stop()
        self.sock.close()
        self.link.close()
