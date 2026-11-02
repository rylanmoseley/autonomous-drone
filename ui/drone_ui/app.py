"""Starter operator GUI: edit a goal sequence and send start/stop/E-stop to the autopilot."""
import sys

from PySide6.QtWidgets import (
    QApplication, QHBoxLayout, QLabel, QLineEdit, QMainWindow, QPushButton,
    QSpinBox, QTableWidget, QTableWidgetItem, QVBoxLayout, QWidget,
)

from drone_ui.protocol import AUTOPILOT_UI_PORT, MAX_GOALS, AutopilotLink, build_sequence

COLUMNS = ["id", "x", "y", "z", "yaw"]
DEFAULT_GOALS = [
    {"id": 1, "x": 1.0, "y": 0.0, "z": 1.0, "yaw": 0.0},
    {"id": 2, "x": 1.0, "y": 1.0, "z": 1.0, "yaw": 0.0},
]


class MainWindow(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Drone Operator")
        self.link = None

        self.host = QLineEdit("127.0.0.1")
        self.port = QSpinBox(maximum=65535, value=AUTOPILOT_UI_PORT)
        conn = QHBoxLayout()
        conn.addWidget(QLabel("Autopilot"))
        conn.addWidget(self.host)
        conn.addWidget(self.port)

        self.table = QTableWidget(0, len(COLUMNS))
        self.table.setHorizontalHeaderLabels(COLUMNS)
        for g in DEFAULT_GOALS:
            self._add_row(g)

        add = QPushButton("Add goal", clicked=lambda: self._add_row({c: 0 for c in COLUMNS}))
        remove = QPushButton("Remove goal", clicked=lambda: self.table.removeRow(self.table.currentRow()))
        edit = QHBoxLayout()
        edit.addWidget(add)
        edit.addWidget(remove)

        start = QPushButton("Start", clicked=lambda: self._send(start=True))
        stop = QPushButton("Land", clicked=lambda: self._send(stop=True))
        estop = QPushButton("E-STOP", clicked=lambda: self._send(estop=True))
        estop.setStyleSheet("background:#c62828;color:white;font-weight:bold")
        cmds = QHBoxLayout()
        cmds.addWidget(start)
        cmds.addWidget(stop)
        cmds.addWidget(estop)

        self.status = QLabel("Not sent")

        layout = QVBoxLayout()
        layout.addLayout(conn)
        layout.addWidget(self.table)
        layout.addLayout(edit)
        layout.addLayout(cmds)
        layout.addWidget(self.status)
        root = QWidget()
        root.setLayout(layout)
        self.setCentralWidget(root)

    def _add_row(self, goal):
        if self.table.rowCount() >= MAX_GOALS:
            return
        row = self.table.rowCount()
        self.table.insertRow(row)
        for col, key in enumerate(COLUMNS):
            self.table.setItem(row, col, QTableWidgetItem(str(goal[key])))

    def _goals(self):
        goals = []
        for row in range(self.table.rowCount()):
            vals = {key: self.table.item(row, col).text() for col, key in enumerate(COLUMNS)}
            goals.append({"id": int(vals["id"]), **{k: float(vals[k]) for k in COLUMNS[1:]}})
        return goals

    def _send(self, **flags):
        try:
            seq = build_sequence(self._goals(), **flags)
        except ValueError as e:
            self.status.setText(f"Bad goal table: {e}")
            return
        if self.link is None or self.link.addr != (self.host.text(), self.port.value()):
            if self.link:
                self.link.close()
            self.link = AutopilotLink(self.host.text(), self.port.value())
        acked = self.link.send_sequence(seq)
        self.status.setText("Autopilot acknowledged" if acked else "No ack from autopilot")


def main():
    app = QApplication(sys.argv)
    win = MainWindow()
    win.resize(520, 420)
    win.show()
    sys.exit(app.exec())


if __name__ == "__main__":
    main()
