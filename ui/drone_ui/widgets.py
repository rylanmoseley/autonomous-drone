"""Building blocks for the operator GUI."""
import math
import time
from pathlib import Path

from PySide6.QtCore import QPointF, QRectF, QSize, Qt, QUrl
from PySide6.QtGui import (
    QColor, QFont, QIcon, QImage, QLinearGradient, QPainter, QPainterPath, QPen, QPixmap,
    QPolygonF,
)
from PySide6.QtWidgets import (
    QFrame, QGridLayout, QHBoxLayout, QLabel, QListWidget, QListWidgetItem, QSizePolicy,
    QToolButton, QVBoxLayout, QWidget,
)

from drone_ui.backends import LANDING_PAD
from drone_ui.model import SHAPE_COLORS, LogLevel, Phase, Shape

# --------------------------------------------------------------------------------------
# Shape drawing


def shape_path(shape, rect):
    """Filled outline of a shape inside rect."""
    path = QPainterPath()
    if shape == Shape.CIRCLE:
        path.addEllipse(rect)
    elif shape == Shape.TRIANGLE:
        path.addPolygon(QPolygonF([
            QPointF(rect.center().x(), rect.top()), rect.bottomRight(), rect.bottomLeft(),
        ]))
        path.closeSubpath()
    else:
        inset = rect.width() * 0.06
        path.addRect(rect.adjusted(inset, inset, -inset, -inset))
    return path


def paint_shape(painter, shape, rect, ring=False):
    color = QColor(SHAPE_COLORS[shape])
    painter.save()
    painter.setRenderHint(QPainter.Antialiasing)
    if ring:
        pen = QPen(color, max(2.0, rect.width() * 0.16))
        pen.setJoinStyle(Qt.MiterJoin)
        painter.setPen(pen)
        painter.setBrush(Qt.NoBrush)
        w = pen.widthF() / 2
        painter.drawPath(shape_path(shape, rect.adjusted(w, w, -w, -w)))
    else:
        painter.setPen(Qt.NoPen)
        painter.setBrush(color)
        painter.drawPath(shape_path(shape, rect))
    painter.restore()


def shape_icon(shape, size=40, ring=True):
    pix = QPixmap(size, size)
    pix.fill(Qt.transparent)
    p = QPainter(pix)
    paint_shape(p, shape, QRectF(2, 2, size - 4, size - 4), ring=ring)
    p.end()
    return QIcon(pix)


def badge_pixmap(color, text, size=22):
    pix = QPixmap(size, size)
    pix.fill(Qt.transparent)
    p = QPainter(pix)
    p.setRenderHint(QPainter.Antialiasing)
    p.setPen(Qt.NoPen)
    p.setBrush(QColor(color))
    p.drawEllipse(QRectF(0, 0, size, size))
    p.setPen(QColor("white"))
    f = QFont()
    f.setBold(True)
    f.setPixelSize(int(size * 0.6))
    p.setFont(f)
    p.drawText(QRectF(0, 0, size, size), Qt.AlignCenter, text)
    p.end()
    return pix


def card():
    frame = QFrame()
    frame.setObjectName("card")
    return frame


def fmt(value, spec, unit=""):
    return "—" if value is None else f"{value:{spec}}{unit}"


# --------------------------------------------------------------------------------------
# Header


LOGO_PATH = Path(__file__).parent / "assets" / "hexagon_flight_logo.png"


def logo_pixmap(height, color="#ffffff"):
    """Team logo recolored for the dark theme: dark ink becomes `color`, the white
    background becomes transparent, and empty margins are trimmed."""
    src = QImage(str(LOGO_PATH)).convertToFormat(QImage.Format_ARGB32)
    ink = QColor(color)
    out = QImage(src.size(), QImage.Format_ARGB32)
    out.fill(Qt.transparent)
    left, top, right, bottom = src.width(), src.height(), -1, -1
    for y in range(src.height()):
        for x in range(src.width()):
            c = src.pixelColor(x, y)
            lum = 0.299 * c.red() + 0.587 * c.green() + 0.114 * c.blue()
            # Light grey (the image's dotted border) and white drop out entirely
            alpha = int(max(0.0, min(1.0, (170.0 - lum) / 150.0)) * c.alpha())
            if alpha > 8:
                out.setPixelColor(x, y, QColor(ink.red(), ink.green(), ink.blue(), alpha))
                left, top, right, bottom = min(left, x), min(top, y), max(right, x), max(bottom, y)
    if right >= 0:
        out = out.copy(left, top, right - left + 1, bottom - top + 1)
    return QPixmap.fromImage(out).scaledToHeight(height, Qt.SmoothTransformation)


class TeamLogo(QLabel):
    def __init__(self, height=64):
        super().__init__()
        self.setPixmap(logo_pixmap(height))


class Header(QFrame):
    def __init__(self):
        super().__init__()
        self.setObjectName("header")
        title = QLabel("Autonomous Drone")
        title.setObjectName("title")
        subtitle = QLabel("Shape Navigation System")
        subtitle.setObjectName("muted")
        titles = QVBoxLayout()
        titles.setSpacing(0)
        titles.addWidget(title)
        titles.addWidget(subtitle)

        self.conn_dot = QLabel()
        self.conn_text = QLabel()
        self.conn_text.setObjectName("strong")
        self.conn_sub = QLabel()
        self.conn_sub.setObjectName("muted")
        self.signal = QLabel("Signal")
        self.signal.setObjectName("strong")
        self.signal_sub = QLabel("—")
        self.signal_sub.setObjectName("muted")
        self.date = QLabel()
        self.date.setObjectName("muted")
        self.date.setAlignment(Qt.AlignRight)
        self.clock = QLabel()
        self.clock.setObjectName("clock")
        self.clock.setAlignment(Qt.AlignRight)

        layout = QHBoxLayout(self)
        layout.setContentsMargins(16, 10, 20, 10)
        layout.addWidget(TeamLogo())
        layout.addSpacing(18)
        divider = QFrame()
        divider.setFrameShape(QFrame.VLine)
        divider.setStyleSheet("color:#2a3a5c;")
        layout.addWidget(divider)
        layout.addSpacing(18)
        layout.addLayout(titles)
        layout.addStretch(1)
        layout.addWidget(self.conn_dot)
        layout.addLayout(self._stack(self.conn_text, self.conn_sub))
        layout.addSpacing(36)
        layout.addLayout(self._stack(self.signal, self.signal_sub))
        layout.addSpacing(36)
        layout.addLayout(self._stack(self.date, self.clock))
        self.tick_clock()

    @staticmethod
    def _stack(a, b):
        v = QVBoxLayout()
        v.setSpacing(0)
        v.addWidget(a)
        v.addWidget(b)
        return v

    def tick_clock(self):
        self.date.setText(time.strftime("%b %d, %Y"))
        self.clock.setText(time.strftime("%I:%M %p").lstrip("0"))

    def update_state(self, s):
        self.conn_dot.setPixmap(badge_pixmap("#27ae60" if s.connected else "#e53935", "", 14))
        self.conn_text.setText("Connected" if s.connected else "Not connected")
        self.conn_sub.setText(s.link_label)
        self.signal_sub.setText(fmt(s.rssi, ".0f", " dBm"))


# --------------------------------------------------------------------------------------
# Shape sequence


class ShapeButton(QToolButton):
    """Big tappable shape; greys out once it's been placed in the order."""

    def __init__(self, shape):
        super().__init__()
        self.shape = shape
        self.setObjectName("shapeBtn")
        self.setText(shape.label)
        self.setIcon(shape_icon(shape, 64, ring=False))
        self.setIconSize(QSize(56, 56))
        self.setToolButtonStyle(Qt.ToolButtonTextUnderIcon)
        self.setCursor(Qt.PointingHandCursor)
        self.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Fixed)
        self.setMinimumHeight(104)


class SequenceSlots(QWidget):
    """The flight order as numbered boxes: empty boxes show what's still to pick,
    the next box to fill is highlighted, and during a mission it shows progress."""

    def __init__(self, length=3):
        super().__init__()
        self.length = length
        self.shapes = []
        self.target_index = None
        self.phase = Phase.READY
        self.picking = True
        self.setMinimumHeight(118)
        self.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Fixed)

    def set_sequence(self, shapes):
        self.shapes = list(shapes)
        self.update()

    def set_progress(self, phase, target_index, picking):
        self.phase, self.target_index, self.picking = phase, target_index, picking
        self.update()

    def _done(self, i):
        if self.phase in (Phase.RETURNING, Phase.LANDING, Phase.COMPLETE):
            return True
        return self.phase == Phase.FLYING and self.target_index is not None and i < self.target_index

    def paintEvent(self, _):
        p = QPainter(self)
        p.setRenderHint(QPainter.Antialiasing)
        n = self.length
        box = min(80.0, self.width() / (n + (n - 1) * 0.45))
        gap = (self.width() - n * box) / (n - 1) if n > 1 else 0
        top = 4.0
        num_font = QFont()
        num_font.setPixelSize(15)
        num_font.setBold(True)
        next_slot = len(self.shapes) if self.picking and len(self.shapes) < n else None

        for i in range(n):
            x = i * (box + gap)
            rect = QRectF(x, top, box, box)
            shape = self.shapes[i] if i < len(self.shapes) else None
            current = shape is not None and self.phase == Phase.FLYING and i == self.target_index

            if shape is None:
                pen = QPen(QColor("#2f80ed" if i == next_slot else "#33456b"), 2.5,
                           Qt.SolidLine if i == next_slot else Qt.DashLine)
                p.setPen(pen)
                p.setBrush(QColor(47, 128, 237, 30) if i == next_slot else Qt.NoBrush)
                p.drawRoundedRect(rect, 12, 12)
                p.setPen(QColor("#5f6f8c"))
                f = QFont()
                f.setPixelSize(30)
                f.setBold(True)
                p.setFont(f)
                p.drawText(rect, Qt.AlignCenter, "?")
            else:
                p.setPen(QPen(QColor(SHAPE_COLORS[shape]) if current else QColor("#2a3a5c"), 3 if current else 1.5))
                p.setBrush(QColor("#16233d"))
                p.drawRoundedRect(rect, 12, 12)
                paint_shape(p, shape, rect.adjusted(box * 0.18, box * 0.18, -box * 0.18, -box * 0.18))
                if self._done(i):
                    p.drawPixmap(int(rect.right() - 28), int(rect.top() + 4), badge_pixmap("#27ae60", "✓", 24))

            p.setPen(QColor("#e6edf7" if current or i == next_slot else "#8a9bb8"))
            p.setFont(num_font)
            label = ["1st", "2nd", "3rd"][i] if i < 3 else f"{i + 1}th"
            p.drawText(QRectF(x - 10, rect.bottom() + 6, box + 20, 22), Qt.AlignCenter, label)

            if i < n - 1:
                ax = rect.right() + gap * 0.25
                bx = rect.right() + gap * 0.75
                y = rect.center().y()
                p.setPen(QPen(QColor("#5f6f8c"), 3, Qt.SolidLine, Qt.RoundCap))
                p.drawLine(QPointF(ax, y), QPointF(bx, y))
                p.drawLine(QPointF(bx, y), QPointF(bx - 7, y - 6))
                p.drawLine(QPointF(bx, y), QPointF(bx - 7, y + 6))


# --------------------------------------------------------------------------------------
# Status cards and log


class InfoCard(QFrame):
    def __init__(self, title, with_pill=False):
        super().__init__()
        self.setObjectName("card")
        head = QHBoxLayout()
        label = QLabel(title)
        label.setObjectName("cardTitle")
        head.addWidget(label)
        head.addStretch(1)
        self.pill = None
        if with_pill:
            self.pill = QLabel()
            self.pill.setObjectName("pill")
            head.addWidget(self.pill)
        self.grid = QGridLayout()
        self.grid.setVerticalSpacing(10)
        layout = QVBoxLayout(self)
        layout.addLayout(head)
        layout.addSpacing(4)
        layout.addLayout(self.grid)

    def add_row(self, name):
        row = self.grid.rowCount()
        key = QLabel(name)
        key.setObjectName("muted")
        value = QLabel("—")
        value.setObjectName("value")
        value.setAlignment(Qt.AlignRight | Qt.AlignVCenter)
        self.grid.addWidget(key, row, 0)
        self.grid.addWidget(value, row, 1)
        return value

    def set_pill(self, text, color):
        self.pill.setText(text)
        self.pill.setStyleSheet(f"background:{color};")


class MissionLog(QFrame):
    ICONS = {
        LogLevel.OK: ("#27ae60", "✓"),
        LogLevel.INFO: ("#2f80ed", "i"),
        LogLevel.WARN: ("#f2994a", "!"),
        LogLevel.ERROR: ("#e53935", "✕"),
    }

    def __init__(self):
        super().__init__()
        self.setObjectName("card")
        title = QLabel("Mission Log")
        title.setObjectName("cardTitle")
        self.list = QListWidget()
        self.list.setObjectName("log")
        self.list.setFocusPolicy(Qt.NoFocus)
        self.list.setSelectionMode(QListWidget.NoSelection)
        self.list.setWordWrap(True)
        self.list.setHorizontalScrollBarPolicy(Qt.ScrollBarAlwaysOff)
        self._icons = {lvl: QIcon(badge_pixmap(c, t, 20)) for lvl, (c, t) in self.ICONS.items()}
        layout = QVBoxLayout(self)
        layout.addWidget(title)
        layout.addWidget(self.list, 1)

    def add(self, level, message):
        item = QListWidgetItem(self._icons[level], f"{time.strftime('%H:%M:%S')}   {message}")
        self.list.addItem(item)
        if self.list.count() > 500:
            self.list.takeItem(0)
        self.list.scrollToBottom()


# --------------------------------------------------------------------------------------
# Camera feed


class CameraView(QWidget):
    """Shows real video frames, or a rendered view of the simulated arena."""

    NEAR = 0.15
    ARENA = (-1.0, 7.0, -3.5, 3.5, 0.0, 4.0)  # x0, x1, y0, y1, z0, z1
    HOOP_RADIUS = 0.5

    def __init__(self, mode):
        super().__init__()
        self.mode = mode  # "sim", "webcam", a URL, or "none"
        self.image = None
        self.message = ""
        self.state = None
        self.hoops = {}
        self.setMinimumSize(420, 320)
        self.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
        if mode not in ("sim", "none"):
            self._start_video(mode)
        elif mode == "none":
            self.message = "No camera connected"

    # Video ----------------------------------------------------------------------------

    def _start_video(self, source):
        from PySide6.QtMultimedia import (
            QCamera, QMediaCaptureSession, QMediaDevices, QMediaPlayer, QVideoSink,
        )
        self.sink = QVideoSink(self)
        self.sink.videoFrameChanged.connect(self._on_frame)
        self.message = "Connecting to camera…"
        if source == "webcam":
            device = QMediaDevices.defaultVideoInput()
            if device.isNull():
                self.message = "No webcam found"
                return
            self.camera = QCamera(device, self)
            self.session = QMediaCaptureSession(self)
            self.session.setCamera(self.camera)
            self.session.setVideoSink(self.sink)
            self.camera.errorOccurred.connect(lambda _e, s: self._fail(s))
            self.camera.start()
        else:
            self.player = QMediaPlayer(self)
            self.player.setVideoSink(self.sink)
            self.player.errorOccurred.connect(lambda _e, s: self._fail(s))
            self.player.setSource(QUrl(source))
            self.player.play()

    def _on_frame(self, frame):
        if frame.isValid():
            self.image = frame.toImage()
            self.update()

    def _fail(self, text):
        self.image = None
        self.message = f"Camera error: {text}"
        self.update()

    def stop(self):
        for name in ("camera", "player"):
            if hasattr(self, name):
                getattr(self, name).stop()

    # Simulated view -------------------------------------------------------------------

    def update_state(self, state, hoops):
        self.state, self.hoops = state, hoops
        if self.mode == "sim":
            self.update()

    def _cam(self, p):
        """World point -> (forward, right, up) relative to the drone's camera."""
        s = self.state
        yaw = math.radians(s.yaw or 0.0)
        dx, dy, dz = p[0] - s.x, p[1] - s.y, p[2] - max(s.z, 0.3)
        return (math.cos(yaw) * dx + math.sin(yaw) * dy,
                -math.sin(yaw) * dx + math.cos(yaw) * dy, dz)

    def _screen(self, c):
        f = self.height() * 0.85
        return QPointF(self.width() / 2 + f * c[1] / c[0], self.height() / 2 - f * c[2] / c[0])

    def _segment(self, p, a, b):
        ca, cb = self._cam(a), self._cam(b)
        if ca[0] < self.NEAR and cb[0] < self.NEAR:
            return None
        if ca[0] < self.NEAR or cb[0] < self.NEAR:
            t = (self.NEAR - ca[0]) / (cb[0] - ca[0])
            clipped = tuple(ca[i] + t * (cb[i] - ca[i]) for i in range(3))
            if ca[0] < self.NEAR:
                ca = clipped
            else:
                cb = clipped
        sa, sb = self._screen(ca), self._screen(cb)
        p.drawLine(sa, sb)
        return sa, sb

    def _hoop_points(self, shape, center):
        hx, hy, hz = center
        r = self.HOOP_RADIUS
        if shape == Shape.CIRCLE:
            pts = [(hy + r * math.cos(t), hz + r * math.sin(t))
                   for t in (2 * math.pi * i / 40 for i in range(40))]
        elif shape == Shape.TRIANGLE:
            pts = [(hy, hz + r), (hy + r * 0.95, hz - r * 0.65), (hy - r * 0.95, hz - r * 0.65)]
        else:
            k = r * 0.85
            pts = [(hy - k, hz - k), (hy + k, hz - k), (hy + k, hz + k), (hy - k, hz + k)]
        return [(hx, y, z) for y, z in pts]

    def _paint_sim(self, p):
        w, h = self.width(), self.height()
        wall = QLinearGradient(0, 0, 0, h / 2)
        wall.setColorAt(0, QColor("#4a5160"))
        wall.setColorAt(1, QColor("#8b929e"))
        p.fillRect(QRectF(0, 0, w, h / 2), wall)
        floor = QLinearGradient(0, h / 2, 0, h)
        floor.setColorAt(0, QColor("#9aa0a8"))
        floor.setColorAt(1, QColor("#c9cdd2"))
        p.fillRect(QRectF(0, h / 2, w, h / 2), floor)
        if self.state is None:
            return

        # Netting cage
        x0, x1, y0, y1, z0, z1 = self.ARENA
        p.setPen(QPen(QColor(30, 30, 30, 70), 1))
        for x in [x0 + i * 0.5 for i in range(int((x1 - x0) / 0.5) + 1)]:
            for y in (y0, y1):
                self._segment(p, (x, y, z0), (x, y, z1))
        for y in [y0 + i * 0.5 for i in range(int((y1 - y0) / 0.5) + 1)]:
            self._segment(p, (x1, y, z0), (x1, y, z1))
        for z in [i * 0.5 for i in range(9)]:
            self._segment(p, (x1, y0, z), (x1, y1, z))
            self._segment(p, (x0, y0, z), (x1, y0, z))
            self._segment(p, (x0, y1, z), (x1, y1, z))
        p.setPen(QPen(QColor("#222"), 3))
        for x, y in ((x1, y0), (x1, y1), (x0, y0), (x0, y1)):
            self._segment(p, (x, y, z0), (x, y, z1))
        # Floor lines for depth
        p.setPen(QPen(QColor(255, 255, 255, 60), 1))
        for x in range(int(x0), int(x1) + 1):
            self._segment(p, (x, y0, 0), (x, y1, 0))

        # Landing pad
        px, py = LANDING_PAD
        p.setPen(QPen(QColor("#f2c94c"), 3))
        ring = [(px + 0.45 * math.cos(t), py + 0.45 * math.sin(t), 0.0)
                for t in (2 * math.pi * i / 32 for i in range(32))]
        for a, b in zip(ring, ring[1:] + ring[:1]):
            self._segment(p, a, b)
        for dy in (-0.15, 0.15):
            self._segment(p, (px - 0.2, py + dy, 0.0), (px + 0.2, py + dy, 0.0))
        self._segment(p, (px, py - 0.15, 0.0), (px, py + 0.15, 0.0))

        # Hoops, far to near
        target = None
        s = self.state
        if s.phase == Phase.FLYING and s.target_index is not None and s.target_index < len(s.sequence):
            target = s.sequence[s.target_index]
        order = sorted(self.hoops.items(),
                       key=lambda kv: -((kv[1][0] - s.x) ** 2 + (kv[1][1] - s.y) ** 2))
        for shape, center in order:
            p.setPen(QPen(QColor("#b03030"), 1.5))
            self._segment(p, (center[0], center[1], center[2] + self.HOOP_RADIUS), (center[0], center[1], z1))
            pts = self._hoop_points(shape, center)
            dist = max(self._cam(center)[0], self.NEAR)
            pen = QPen(QColor(SHAPE_COLORS[shape]), max(2.0, h * 0.06 / dist))
            pen.setCapStyle(Qt.RoundCap)
            p.setPen(pen)
            drawn = []
            for a, b in zip(pts, pts[1:] + pts[:1]):
                seg = self._segment(p, a, b)
                if seg:
                    drawn.extend(seg)
            if shape == target and drawn:
                self._paint_bracket(p, drawn, shape)

        badge = "SIMULATED VIEW"
        f = QFont()
        f.setPixelSize(12)
        f.setBold(True)
        p.setFont(f)
        p.setPen(Qt.NoPen)
        p.setBrush(QColor(0, 0, 0, 140))
        p.drawRoundedRect(QRectF(10, h - 34, 130, 24), 6, 6)
        p.setPen(QColor("white"))
        p.drawText(QRectF(10, h - 34, 130, 24), Qt.AlignCenter, badge)

    def _paint_bracket(self, p, points, shape):
        xs = [pt.x() for pt in points]
        ys = [pt.y() for pt in points]
        r = QRectF(min(xs), min(ys), max(xs) - min(xs), max(ys) - min(ys)).adjusted(-14, -14, 14, 14)
        if r.width() > self.width() * 1.2:
            return
        p.setPen(QPen(QColor("#2ee66b"), 3))
        k = min(r.width(), r.height()) * 0.25
        for cx, cy, sx, sy in ((r.left(), r.top(), 1, 1), (r.right(), r.top(), -1, 1),
                               (r.left(), r.bottom(), 1, -1), (r.right(), r.bottom(), -1, -1)):
            p.drawLine(QPointF(cx, cy), QPointF(cx + sx * k, cy))
            p.drawLine(QPointF(cx, cy), QPointF(cx, cy + sy * k))
        f = QFont()
        f.setPixelSize(14)
        f.setBold(True)
        p.setFont(f)
        label = f"TARGET: {shape.label.upper()}"
        box = QRectF(r.left(), r.top() - 28, max(130.0, len(label) * 9.5), 22)
        p.setPen(Qt.NoPen)
        p.setBrush(QColor("#2ee66b"))
        p.drawRoundedRect(box, 4, 4)
        p.setPen(QColor("#0b1220"))
        p.drawText(box, Qt.AlignCenter, label)

    # Paint ----------------------------------------------------------------------------

    def paintEvent(self, _):
        p = QPainter(self)
        p.setRenderHint(QPainter.Antialiasing)
        clip = QPainterPath()
        clip.addRoundedRect(QRectF(self.rect()), 10, 10)
        p.setClipPath(clip)
        if self.mode == "sim":
            self._paint_sim(p)
        elif self.image is not None:
            p.fillRect(self.rect(), QColor("black"))
            scaled = self.image.scaled(self.size(), Qt.KeepAspectRatio, Qt.SmoothTransformation)
            p.drawImage((self.width() - scaled.width()) // 2, (self.height() - scaled.height()) // 2, scaled)
        else:
            p.fillRect(self.rect(), QColor("#060a14"))
            p.setPen(QColor("#8a9bb8"))
            f = QFont()
            f.setPixelSize(18)
            p.setFont(f)
            p.drawText(self.rect(), Qt.AlignCenter, self.message)

        live = self.mode == "sim" or self.image is not None
        box = QRectF(self.width() - 78, 12, 64, 26)
        p.setPen(Qt.NoPen)
        p.setBrush(QColor(0, 0, 0, 150))
        p.drawRoundedRect(box, 6, 6)
        p.setBrush(QColor("#27ae60" if live else "#8a9bb8"))
        p.drawEllipse(QPointF(box.left() + 14, box.center().y()), 5, 5)
        f = QFont()
        f.setPixelSize(13)
        p.setFont(f)
        p.setPen(QColor("white"))
        p.drawText(box.adjusted(24, 0, 0, 0), Qt.AlignVCenter, "Live" if live else "Off")


class CameraFeed(QFrame):
    def __init__(self, mode):
        super().__init__()
        self.setObjectName("card")
        title = QLabel("Live Camera Feed")
        title.setObjectName("cardTitle")
        dot = QLabel()
        dot.setPixmap(badge_pixmap("#27ae60", "", 12))
        head = QHBoxLayout()
        head.addWidget(dot)
        head.addWidget(title)
        head.addStretch(1)
        self.view = CameraView(mode)
        layout = QVBoxLayout(self)
        layout.addLayout(head)
        layout.addWidget(self.view, 1)
