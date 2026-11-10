# Arduino Uno R3 MAVLink FCU Emulator

This utility turns an **Arduino Uno R3 (ATmega328P)** into a simulated ArduPilot (ArduCopter) Flight Control Unit (FCU). It outputs standard **MAVLink v2** telemetry over hardware serial (`Serial` / Pins 0 & 1), allowing you to bench-test the companion computer MAVLink proxy and Ground Control Station (GCS) software (such as **Mission Planner** or **QGroundControl**) without risking real flight hardware or propellers.

---

## Capabilities & Telemetry Streams

The sketch acts as a self-contained, zero-dependency ArduPilot quadcopter:

| MAVLink Message | ID | Rate | Purpose |
| :--- | :---: | :---: | :--- |
| **`HEARTBEAT`** | `#0` | 1 Hz | Identifies vehicle as ArduPilot Quadcopter (`MAV_TYPE_QUADROTOR`, `MAV_AUTOPILOT_ARDUPILOTMEGA`). Broadcasts armed/disarmed state. |
| **`SYS_STATUS`** | `#1` | 1 Hz | Reports 12.6V battery voltage (3S LiPo), 1.45A current draw, 91% remaining capacity, and healthy sensor flags. |
| **`ATTITUDE`** | `#30` | 4 Hz | Streams roll, pitch, and yaw with subtle synthetic oscillation to prove live HUD responsiveness. |
| **`LOCAL_POSITION_NED`** | `#32` | 2 Hz | Position setpoint ($X=1.5\text{ m}, Y=0.0\text{ m}, Z=-1.25\text{ m}$) compatible with the onboard Autopilot daemon. |
| **`GLOBAL_POSITION_INT`** | `#33` | 1 Hz | GPS coordinate position and altitude so Mission Planner displays the drone icon on the map. |
| **`VFR_HUD`** | `#74` | 2 Hz | Drives the Mission Planner HUD instrument tape (airspeed, altitude, heading, throttle). |
| **`PARAM_VALUE`** | `#22` | On Request | Responds to `PARAM_REQUEST_LIST` with standard ArduPilot parameters (`SYSID_THISMAV`, `PILOT_SPEED_UP`, `WP_RADIUS`, `FRAME_CLASS`, etc.) stored in Flash (`PROGMEM`) so Mission Planner completes connection instantly. |
| **`COMMAND_ACK`** | `#77` | On Request | Acknowledges arm/disarm (`MAV_CMD_COMPONENT_ARM_DISARM`) and capability requests. |
| **`STATUSTEXT`** | `#253` | Event | Emits status messages on boot and when motors are armed or disarmed. |

---

## Hardware Connections & Pinout

```
+------------------------+                     +-------------------------------+
|     Arduino Uno R3     |                     |    Raspberry Pi 40-Pin Header |
|                        |                     |                               |
|                    GND |-------------------->| Pin 6  (GND)                  |
|             Pin 0 (RX) |<--------------------| Pin 8  (GPIO 14 / UART0 TX)   |
|             Pin 1 (TX) |--[Divider / Shifter]| Pin 10 (GPIO 15 / UART0 RX)   |
+------------------------+                     +-------------------------------+
```

### ⚠️ Critical Voltage Level Notice (5.0V vs 3.3V)
* **Raspberry Pi GPIOs are strictly 3.3V and ARE NOT 5V tolerant.**
* **Pi TX (3.3V) $\rightarrow$ Arduino RX**: Safe to connect directly (3.3V meets the ATmega328P input HIGH threshold $V_{IH} \ge 3.0\text{ V}$).
* **Arduino TX (5.0V) $\rightarrow$ Pi RX**: **DO NOT connect directly.** Use either:
  1. A bidirectional logic level shifter (e.g. BSS138), or
  2. A simple 2-resistor voltage divider:
     ```
     Arduino Pin 1 (TX) ---> [ 1kΩ Resistor ] ---+---> Pi Pin 10 (RX)
                                                  |
                                           [ 2kΩ Resistor ]
                                                  |
                                                 GND
     ```
     This steps $5.0\text{ V}$ down safely to $\approx 3.33\text{ V}$.

---

## Quickstart Guide

### 1. Flash the Arduino
1. Open [`arduino_fcu_emulator.ino`](arduino_fcu_emulator.ino) in the **Arduino IDE**.
2. Select **Board**: *Arduino Uno*.
3. Select your Uno's **Port** and click **Upload**.
4. Once flashed, the onboard LED on **Pin 13** will begin blinking at 1 Hz (indicating active heartbeat generation).

### 2. Configure & Start the MAVLink Proxy on the Pi
The Arduino emulator uses **115,200 baud** (at 16 MHz, the ATmega328P has an 8.5% clock divisor error at 921,600 baud, but only 2.1% error at 115,200 baud).

SSH into the Raspberry Pi and run the proxy:
```bash
# Ensure serial UART is enabled in raspi-config and /dev/ttyAMA0 exists
python3 /opt/drone/scripts/mavlink_proxy.py \
    --serial /dev/ttyAMA0 \
    --baud 115200 \
    --gcs-broadcast-ip 192.168.4.255 \
    --gcs-port 14550
```

*Note: If testing locally or via USB cable plugged directly into the Pi, use `--serial /dev/ttyACM0`.*

### 3. Connect Mission Planner
1. Connect your ground station laptop to the drone's Wi-Fi Access Point: **`HEXAGON-flight-drone`**.
2. Open **Mission Planner**.
3. In the top right corner:
   - Select connection type: **UDP**
   - Click **Connect**
   - Enter Port: **`14550`**
4. Mission Planner will detect the `HEARTBEAT`, request parameters, receive the parameter table, and transition to the live Flight Data screen:
   - **HUD**: Active horizon oscillating gently.
   - **Battery**: Displays 12.6V / 91%.
   - **Map**: Vehicle located at 37.7749° N, 122.4194° W.
   - **Action Tab**: Clicking **Arm/Disarm** will toggle the emulator's state and Pin 13 LED (solid ON when armed).

---

## Manual Serial Controls (Optional)
If you connect to the Uno using the Arduino IDE Serial Monitor (set to 115,200 baud, No line ending):
* Press **`p`** or **`P`**: Forces re-broadcasting the full parameter table.
* Press **`a`** or **`A`**: Toggles motor arm/disarm state manually.
