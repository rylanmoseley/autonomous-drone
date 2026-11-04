import os

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

import pytest  # noqa: E402
from PySide6.QtWidgets import QApplication  # noqa: E402

from drone_ui.app import MainWindow  # noqa: E402
from drone_ui.backends import AutopilotBackend, SimBackend  # noqa: E402
from drone_ui.model import Phase, Shape  # noqa: E402

app = QApplication.instance() or QApplication([])


@pytest.fixture
def sim():
    backend = SimBackend(autostart=False)
    yield backend
    backend.shutdown()


def run_until(backend, phase, max_ticks=5000):
    for _ in range(max_ticks):
        backend.tick()
        if backend.state.phase == phase:
            return
    raise AssertionError(f"never reached {phase}")


def tap(win, *shapes):
    for shape in shapes:
        next(b for b in win.shape_btns if b.shape == shape).click()


def test_tapping_builds_order_once_each(sim):
    win = MainWindow(sim, camera="none")
    tap(win, Shape.SQUARE, Shape.SQUARE, Shape.CIRCLE)
    assert win.sequence == [Shape.SQUARE, Shape.CIRCLE]
    assert not next(b for b in win.shape_btns if b.shape == Shape.SQUARE).isEnabled()
    assert not win.confirm_btn.isEnabled()  # all three needed
    tap(win, Shape.TRIANGLE)
    assert win.confirm_btn.isEnabled()
    win.clear_btn.click()
    assert win.sequence == [] and all(b.isEnabled() for b in win.shape_btns)


def test_start_needs_confirm(sim):
    win = MainWindow(sim, camera="none")
    tap(win, Shape.CIRCLE, Shape.TRIANGLE, Shape.SQUARE)
    assert not win.start_btn.isEnabled()
    win.confirm_btn.click()
    assert win.start_btn.isEnabled()
    win.clear_btn.click()
    assert not win.start_btn.isEnabled()  # changing the order un-confirms


def test_sim_flies_full_mission_in_order(sim):
    reached = []
    sim.log.connect(lambda _lvl, msg: msg.startswith("Flew through") and reached.append(msg.split()[3]))
    order = [Shape.SQUARE, Shape.CIRCLE, Shape.TRIANGLE]
    sim.confirm_sequence(order)
    sim.start_mission()
    run_until(sim, Phase.COMPLETE)
    assert reached == ["Square", "Circle", "Triangle"]
    # Lands back on the pad
    assert abs(sim.state.x) < 0.05 and abs(sim.state.y) < 0.05 and abs(sim.state.z) < 0.05
    assert sim.state.mission_time > 0


def test_window_tracks_mission(sim):
    win = MainWindow(sim, camera="sim")
    tap(win, Shape.CIRCLE, Shape.TRIANGLE, Shape.SQUARE)
    win.confirm_btn.click()
    win.start_btn.click()
    run_until(sim, Phase.FLYING)
    assert not any(b.isEnabled() for b in win.shape_btns)
    assert not win.clear_btn.isEnabled()
    assert win.v_target.text() == "Circle (1/3)"
    win.camera.view.grab()  # renders the simulated camera without errors
    win.estop_btn.click()
    assert sim.state.phase == Phase.ESTOP
    assert not win.confirm_btn.isEnabled() and not win.start_btn.isEnabled()
    # Start over -> pick again -> confirm puts the drone back on the pad
    win.clear_btn.click()
    tap(win, Shape.SQUARE, Shape.TRIANGLE, Shape.CIRCLE)
    win.confirm_btn.click()
    assert sim.state.phase == Phase.READY and sim.state.z == 0.0
    assert win.start_btn.isEnabled()


