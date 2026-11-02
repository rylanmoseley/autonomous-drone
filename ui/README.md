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
python -m drone_ui.app      # the GUI
python -m pytest            # tests
```

## How it talks to the autopilot

UDP to port 14552 (`AUTOPILOT_UI_PORT`), using the packed structs in
`shared/include/ui_protocol.h`. `drone_ui/protocol.py` mirrors those structs; if the
header changes, update the Python side too, because the autopilot ignores packets of the wrong size.

The autopilot only builds on Linux; run it in WSL and point the GUI at it.
