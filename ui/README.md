# Operator UI

Python (PySide6) GUI for sending goal sequences and start / land / E-stop commands to the autopilot.

## Setup (Windows, PowerShell)

```powershell
cd ui
python -m venv .venv
.\.venv\Scripts\Activate.ps1
pip install -r requirements.txt
```

## Run

```powershell
python -m drone_ui.app                                   # GUI with the built-in simulator
python -m drone_ui.app --autopilot 127.0.0.1             # GUI driving the real autopilot
python -m drone_ui.app --autopilot 127.0.0.1 --camera webcam
python -m pytest                                         # tests
```

`--camera` takes `sim`, `webcam`, `none`, or a stream URL (e.g. `rtsp://...` or an HTTP MJPEG stream from the Pi).

## Layout

- `model.py`: what the GUI displays (`DroneState`, shapes, mission phases)
- `backends.py`: `SimBackend` (pretend mission) and `AutopilotBackend` (UDP to the real autopilot). Fields the autopilot doesn't report yet stay `None` and show as a dash.
- `widgets.py`: header, shape pickers, sequence preview, camera view, status cards, mission log
- `app.py`: main window, theme, command-line options

## How it talks to the autopilot

UDP to port 14552 (`AUTOPILOT_UI_PORT`), using the packed structs in
`shared/include/ui_protocol.h`. `drone_ui/protocol.py` mirrors those structs; if the
header changes, update the Python side too, because the autopilot ignores packets of the wrong size.

The autopilot only builds on Linux; run it in WSL and point the GUI at it.
