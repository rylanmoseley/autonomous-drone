"""Data shown by the GUI, independent of where it comes from (simulator or autopilot)."""
from dataclasses import dataclass
from enum import Enum, IntEnum


class Shape(IntEnum):
    # Values are the goal ids sent to the autopilot (matched against vision's goal_id)
    CIRCLE = 1
    TRIANGLE = 2
    SQUARE = 3

    @property
    def label(self):
        return self.name.title()


SHAPE_COLORS = {Shape.CIRCLE: "#e53935", Shape.TRIANGLE: "#fbc02d", Shape.SQUARE: "#1e88e5"}


class Phase(Enum):
    READY = "Ready"
    TAKEOFF = "Taking Off"
    FLYING = "In Flight"
    RETURNING = "Returning to Pad"
    LANDING = "Landing"
    COMPLETE = "Mission Complete"
    ESTOP = "Emergency Stop"
    ERROR = "Error"


ACTIVE_PHASES = {Phase.TAKEOFF, Phase.FLYING, Phase.RETURNING, Phase.LANDING}

PHASE_COLORS = {
    Phase.READY: "#2f80ed",
    Phase.TAKEOFF: "#27ae60",
    Phase.FLYING: "#27ae60",
    Phase.RETURNING: "#27ae60",
    Phase.LANDING: "#27ae60",
    Phase.COMPLETE: "#8e44ad",
    Phase.ESTOP: "#e53935",
    Phase.ERROR: "#e53935",
}


@dataclass
class DroneState:
    """None means "not reported by this source" and is shown as a dash."""
    connected: bool = False
    link_label: str = ""
    phase: Phase = Phase.READY
    sequence: tuple = ()
    target_index: int | None = None  # index into sequence
    x: float | None = None
    y: float | None = None
    z: float | None = None
    roll: float | None = None  # degrees
    pitch: float | None = None
    yaw: float | None = None
    speed: float | None = None  # m/s
    battery: float | None = None  # percent
    rssi: int | None = None  # dBm
    mission_time: float | None = None  # seconds


class LogLevel(Enum):
    OK = "ok"
    INFO = "info"
    WARN = "warn"
    ERROR = "error"
