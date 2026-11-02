"""Operator GUI for the shape-navigation demo.

Run with the built-in simulator (default):
    python -m drone_ui.app
Against the real autopilot, with a webcam or a stream URL for the camera panel:
    python -m drone_ui.app --autopilot 127.0.0.1 --camera webcam
"""
import argparse
import sys

from PySide6.QtCore import QTimer
from PySide6.QtWidgets import (
    QApplication, QFrame, QHBoxLayout, QLabel, QMainWindow, QPushButton, QVBoxLayout, QWidget,
)

from drone_ui.backends import AutopilotBackend, SimBackend, hoop_position
from drone_ui.model import ACTIVE_PHASES, PHASE_COLORS, SHAPE_COLORS, Phase, Shape
from drone_ui.widgets import (
    CameraFeed, Header, InfoCard, MissionLog, SequenceSlots, ShapeButton, card, fmt,
)

STYLE = """
* { font-family: "Segoe UI", sans-serif; color: #e6edf7; }
QMainWindow, #root { background: #070d1a; }
#header { background: #0d1628; border-bottom: 1px solid #1e2a44; }
#title { font-size: 24px; font-weight: 600; }
#clock { font-size: 20px; font-weight: 600; }
#strong { font-size: 14px; font-weight: 600; }
#muted { color: #8a9bb8; font-size: 13px; }
#card { background: #0f1a2e; border: 1px solid #1e2a44; border-radius: 12px; }
#card QLabel { background: transparent; }
#cardTitle { font-size: 17px; font-weight: 600; }
#value { font-size: 14px; font-weight: 600; }
#hint { color: #9fc3ff; font-size: 14px; }
#pill { color: white; font-weight: 600; font-size: 14px; padding: 4px 14px; border-radius: 6px; }
#shapeBtn {
    background: #16233d; border: 2px solid #2a3a5c; border-radius: 14px;
    font-size: 15px; font-weight: 600; padding: 8px 4px;
}
#shapeBtn:hover { border-color: #2f80ed; background: #1a2b4b; }
#shapeBtn:pressed { background: #2f80ed; }
#shapeBtn:disabled { background: #0c1424; border: 2px dashed #1e2a44; color: #3e4c68; }
QPushButton#clear {
    background: transparent; color: #9fc3ff; font-size: 14px; padding: 4px 8px;
    border: 1px solid #2a3a5c; border-radius: 8px;
}
QPushButton#clear:hover { background: #16233d; }
QPushButton#clear:disabled { color: #3e4c68; border-color: #1b2740; background: transparent; }
QPushButton {
    border: none; border-radius: 10px; font-size: 18px; font-weight: 600;
    padding: 14px; color: white;
}
#confirm { background: #2f80ed; }
#confirm:hover { background: #4a92f0; }
#confirm[done="true"] { background: #1d4f8f; color: #cfe1ff; }
#start { background: #219653; }
#start:hover { background: #27ae60; }
#estop { background: #e53935; font-size: 20px; padding: 18px; }
#estop:hover { background: #f44336; }
#estop:pressed { background: #b71c1c; }
#confirm:disabled, #start:disabled { background: #1b2740; color: #5f6f8c; }
#confirm[done="true"]:disabled { background: #1d4f8f; color: #cfe1ff; }
#log { background: transparent; border: none; font-size: 13px; }
#log::item { padding: 6px 2px; }
QScrollBar:vertical { background: transparent; width: 8px; }
QScrollBar::handle:vertical { background: #2a3a5c; border-radius: 4px; min-height: 24px; }
QScrollBar::add-line, QScrollBar::sub-line { height: 0; }
QScrollBar::add-page, QScrollBar::sub-page { background: none; }
"""

FINISHED_PHASES = {Phase.COMPLETE, Phase.ESTOP, Phase.ERROR}

HINTS = {
    "empty": "Tap the shape the drone should fly through first.",
    "next": "Nice! Now tap the shape it should fly through {nth}.",
    "full": "Looks good! Press Confirm Order.",
    "confirmed": "Great! Now press Start Mission.",
    "flying": "The drone is flying! Watch it find each shape.",
    "complete": "Mission complete! Press Start over to fly again.",
    "estop": "Emergency stop! Press Start over to reset.",
}


class MainWindow(QMainWindow):
    def __init__(self, backend, camera="sim"):
        super().__init__()
        self.setWindowTitle("Autonomous Drone - Shape Navigation")
        self.backend = backend
        self.sequence = []
        self.confirmed = False
        self.finished = False  # mission ended or stopped; picking waits for "Start over"
        self.phase = Phase.READY
        self.target_index = None

        self.header = Header()
        self.clock_timer = QTimer(self, interval=1000, timeout=self.header.tick_clock)
        self.clock_timer.start()

        body = QHBoxLayout()
        body.setContentsMargins(14, 14, 14, 14)
        body.setSpacing(14)
        body.addWidget(self._build_controls(), 0)
        self.camera = CameraFeed(camera)
        body.addWidget(self.camera, 1)
        body.addLayout(self._build_status(), 0)

        root = QWidget()
        root.setObjectName("root")
        layout = QVBoxLayout(root)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.setSpacing(0)
        layout.addWidget(self.header)
        layout.addLayout(body, 1)
        self.setCentralWidget(root)

        self.hoops = {sh: hoop_position(sh) for sh in Shape}
        backend.state_changed.connect(self.on_state)
        backend.log.connect(self.mission_log.add)
        backend.sequence_confirmed.connect(self.on_confirmed)
        self._refresh_controls()

    # Layout ---------------------------------------------------------------------------

    def _build_controls(self):
        panel = card()
        panel.setFixedWidth(340)
        v = QVBoxLayout(panel)
        v.setContentsMargins(18, 16, 18, 18)
        v.setSpacing(12)

        title = QLabel("1. Tap the shapes in order")
        title.setObjectName("cardTitle")
        self.hint = QLabel()
        self.hint.setObjectName("hint")
        self.hint.setWordWrap(True)
        self.hint.setMinimumHeight(40)
        v.addWidget(title)
        v.addWidget(self.hint)

        buttons = QHBoxLayout()
        buttons.setSpacing(8)
        self.shape_btns = []
        for shape in Shape:
            btn = ShapeButton(shape)
            btn.clicked.connect(lambda _=False, s=shape: self.on_shape_tapped(s))
            self.shape_btns.append(btn)
            buttons.addWidget(btn)
        v.addLayout(buttons)

        order_head = QHBoxLayout()
        order_title = QLabel("Flight order")
        order_title.setObjectName("cardTitle")
        self.clear_btn = QPushButton("↺  Start over", objectName="clear", clicked=self.on_clear)
        order_head.addWidget(order_title)
        order_head.addStretch(1)
        order_head.addWidget(self.clear_btn)
        v.addSpacing(4)
        v.addLayout(order_head)
        self.slots = SequenceSlots(len(Shape))
        v.addWidget(self.slots)

        self.confirm_btn = QPushButton(objectName="confirm", clicked=self.on_confirm)
        v.addWidget(self.confirm_btn)

        line = QFrame()
        line.setFrameShape(QFrame.HLine)
        line.setStyleSheet("color:#1e2a44;")
        v.addWidget(line)
        controls_title = QLabel("2. Fly!")
        controls_title.setObjectName("cardTitle")
        v.addWidget(controls_title)
        self.start_btn = QPushButton("▶  Start Mission", objectName="start", clicked=self.on_start)
        v.addWidget(self.start_btn)
        v.addStretch(1)
        self.estop_btn = QPushButton("⚠  Emergency Stop", objectName="estop", clicked=self.backend_estop)
        v.addWidget(self.estop_btn)
        return panel

    def _build_status(self):
        col = QVBoxLayout()
        col.setSpacing(14)

        self.status_card = InfoCard("Drone Status", with_pill=True)
        self.v_target = self.status_card.add_row("Current Target")
        self.v_alt = self.status_card.add_row("Altitude")
        self.v_speed = self.status_card.add_row("Speed")
        self.v_battery = self.status_card.add_row("Battery")
        self.v_time = self.status_card.add_row("Mission Time")

        self.telem_card = InfoCard("Telemetry")
        self.v_pos = self.telem_card.add_row("Position (x, y, z)")
        self.v_orient = self.telem_card.add_row("Orientation (roll, pitch, yaw)")
        self.v_vel = self.telem_card.add_row("Velocity")
        self.v_rssi = self.telem_card.add_row("Signal Strength")

        self.mission_log = MissionLog()
        for w in (self.status_card, self.telem_card, self.mission_log):
            w.setFixedWidth(340)
        col.addWidget(self.status_card)
        col.addWidget(self.telem_card)
        col.addWidget(self.mission_log, 1)
        return col

    # Sequence selection -----------------------------------------------------------------

    def on_shape_tapped(self, shape):
        # Each shape is flown once, in the order tapped
        if shape in self.sequence or len(self.sequence) >= len(Shape):
            return
        self._set_sequence(self.sequence + [shape])

    def on_clear(self):
        self.finished = False
        self._set_sequence([])

    def _set_sequence(self, shapes):
        self.sequence = shapes
        self.slots.set_sequence(shapes)
        self.confirmed = False
        self._refresh_controls()

    def on_confirm(self):
        self.backend.confirm_sequence(list(self.sequence))

    def on_confirmed(self, ok):
        self.confirmed = ok
        self._refresh_controls()

    def on_start(self):
        self.backend.start_mission()
        self.confirmed = False  # a new mission needs a fresh confirm
        self._refresh_controls()

    def backend_estop(self):
        self.backend.emergency_stop()
        self.confirmed = False
        self._refresh_controls()

    def _refresh_controls(self):
        active = self.phase in ACTIVE_PHASES
        full = len(self.sequence) == len(Shape)
        # After a finished or stopped mission the old order stays on screen until "Start over"
        picking = not active and not self.finished
        for btn in self.shape_btns:
            btn.setEnabled(picking and btn.shape not in self.sequence)
        self.clear_btn.setEnabled(not active and bool(self.sequence))
        self.slots.set_progress(self.phase, self.target_index, picking)

        self.confirm_btn.setEnabled(picking and full and not self.confirmed)
        self.confirm_btn.setText("✓  Order Confirmed" if self.confirmed else "✓  Confirm Order")
        self.confirm_btn.setProperty("done", self.confirmed)
        self.confirm_btn.style().polish(self.confirm_btn)
        self.start_btn.setEnabled(self.confirmed and not active)

        if active:
            hint = HINTS["flying"]
        elif self.finished:
            hint = HINTS["complete" if self.phase == Phase.COMPLETE else "estop"]
        elif self.confirmed:
            hint = HINTS["confirmed"]
        elif full:
            hint = HINTS["full"]
        elif self.sequence:
            hint = HINTS["next"].format(nth=["first", "second", "third"][len(self.sequence)])
        else:
            hint = HINTS["empty"]
        self.hint.setText(hint)

    # State updates --------------------------------------------------------------------

    def on_state(self, s):
        if s.phase != self.phase and s.phase in FINISHED_PHASES:
            self.finished = True
        self.phase, self.target_index = s.phase, s.target_index
        self.header.update_state(s)
        self.status_card.set_pill(s.phase.value, PHASE_COLORS[s.phase])

        if s.phase == Phase.FLYING and s.target_index is not None and s.target_index < len(s.sequence):
            shape = s.sequence[s.target_index]
            self.v_target.setText(f"{shape.label} ({s.target_index + 1}/{len(s.sequence)})")
            self.v_target.setStyleSheet(f"color:{SHAPE_COLORS[shape]};")
        else:
            self.v_target.setText("—")
            self.v_target.setStyleSheet("")
        self.v_alt.setText(fmt(s.z, ".1f", " m"))
        self.v_speed.setText(fmt(s.speed, ".1f", " m/s"))
        self.v_battery.setText(fmt(s.battery, ".0f", "%"))
        if s.battery is not None:
            color = "#27ae60" if s.battery > 50 else "#f2c94c" if s.battery > 20 else "#e53935"
            self.v_battery.setStyleSheet(f"color:{color};")
        if s.mission_time is None:
            self.v_time.setText("—")
        else:
            t = int(s.mission_time)
            self.v_time.setText(f"{t // 3600:02d}:{t // 60 % 60:02d}:{t % 60:02d}")

        def triple(a, b, c, spec, unit):
            return f"({fmt(a, spec)}, {fmt(b, spec)}, {fmt(c, spec)}){unit}"
        self.v_pos.setText(triple(s.x, s.y, s.z, ".1f", " m"))
        self.v_orient.setText(triple(s.roll, s.pitch, s.yaw, ".1f", "°"))
        self.v_vel.setText(fmt(s.speed, ".1f", " m/s"))
        self.v_rssi.setText(fmt(s.rssi, ".0f", " dBm"))

        self.camera.view.update_state(s, self.hoops)
        self._refresh_controls()

    def closeEvent(self, event):
        self.backend.shutdown()
        self.camera.view.stop()
        super().closeEvent(event)


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--autopilot", metavar="HOST", help="connect to the real autopilot instead of the simulator")
    parser.add_argument("--camera", default=None,
                        help='"sim" (simulated view), "webcam", "none", or a stream URL. '
                             "Defaults to sim with the simulator and none with the autopilot.")
    args = parser.parse_args()

    app = QApplication(sys.argv)
    app.setStyleSheet(STYLE)
    if args.autopilot:
        backend = AutopilotBackend(args.autopilot)
        camera = args.camera or "none"
    else:
        backend = SimBackend()
        camera = args.camera or "sim"
    if camera == "sim" and args.autopilot:
        camera = "none"  # the simulated view needs simulated positions
    win = MainWindow(backend, camera)
    win.resize(1360, 820)
    win.show()
    sys.exit(app.exec())


if __name__ == "__main__":
    main()
