# Arduino Uno R3 MAVLink FCU Emulator

This utility turns an **Arduino Uno R3 (ATmega328P)** into a simulated ArduPilot (ArduCopter) Flight Control Unit (FCU). It outputs standard **MAVLink v2** telemetry over hardware serial, allowing you to bench-test the companion computer MAVLink proxy and Ground Control Station (GCS) software (such as **Mission Planner** or **QGroundControl**) without risking real flight hardware or propellers.

---

## ❓ Why Does the Serial Monitor Show "Slop" / Gibberish?

If you open the Arduino IDE Serial Monitor and see a continuous stream of strange symbols like `ý`, question marks, and unprintable glyphs:

1. **MAVLink is a raw binary wire protocol**, NOT human-readable ASCII text.
   - Every packet starts with the magic byte `0xFD` (which renders in ASCII as `ý`).
   - The remaining bytes are raw binary IEEE-754 floats (angles, velocities), bitmasks, and CRCs (bytes like `0x00`, `0x1E`, `0x9F`).
   - Text terminals like the Arduino Serial Monitor cannot decode binary packets and will render them as gibberish ("slop").
   - **Seeing `ý` and binary characters actually confirms that the MAVLink packet generator is alive and transmitting!**
2. **Serial Monitor Baud Rate**: If the Serial Monitor is set to the default `9600` baud instead of `115200` baud, framing errors will make the binary look even more erratic.
3. **Hardware Pin Sharing on Arduino Uno**:
   - On the Uno, **Pin 0 (RX) and Pin 1 (TX) are physically wired to the onboard ATmega16U2 USB chip**.
   - If the Uno is connected to your PC via USB to view Serial Monitor *and* simultaneously wired to the Raspberry Pi GPIO pins, the USB chip and the Pi will fight over the RX pin.

---

## Operating Modes

[`arduino_fcu_emulator.ino`](arduino_fcu_emulator.ino) supports two modes via `#define USE_SOFTWARE_SERIAL`:

### Mode 0: Hardware Serial (Default - `#define USE_SOFTWARE_SERIAL 0`)
* **MAVLink Stream**: Transmits on **Pins 0 & 1** (and USB) at **115,200 baud**.
* **Usage**:
  1. Upload the sketch.
  2. **Close the Arduino IDE Serial Monitor**.
  3. Start the proxy on the Raspberry Pi:
     ```bash
     python3 /opt/drone/scripts/mavlink_proxy.py --serial /dev/ttyAMA0 --baud 115200
     ```
  4. Connect Mission Planner via UDP (`14550`). Mission Planner's parser will ingest the binary stream and display your HUD, attitude, and battery indicators!

### Mode 1: Dual Mode with Human-Readable Debug Console (`#define USE_SOFTWARE_SERIAL 1`)
* **MAVLink Stream**: Transmits on **Pin 2 (RX) and Pin 3 (TX)** at **57,600 baud** to the Raspberry Pi.
* **Serial Monitor (USB)**: Prints clean, formatted English debug logs at **115,200 baud** with **zero slop**!
  ```
  [1Hz] Heartbeat | STABILIZE (DISARMED) | Batt: 12.58V (91%) | Alt: 1.25m
  [GCS] PARAM_REQUEST_LIST received! Sending 8 parameters to Mission Planner...
  [CMD] Motor Arm State Changed -> ARMED
  ```
* **Usage**:
  1. Wire Pi TX $\rightarrow$ Arduino Pin 2 (RX).
  2. Wire Arduino Pin 3 (TX) through voltage divider $\rightarrow$ Pi RX.
  3. Change line 63 of [`arduino_fcu_emulator.ino`](arduino_fcu_emulator.ino) to:
     ```cpp
     #define USE_SOFTWARE_SERIAL 1
     ```
  4. Keep your USB cable plugged into your PC and open Serial Monitor at **115,200 baud**.
  5. On the Pi, run:
     ```bash
     python3 /opt/drone/scripts/mavlink_proxy.py --serial /dev/ttyAMA0 --baud 57600
     ```

---

## Hardware Connections & Pinout

### Mode 0 (Default Hardware Serial):
```
+------------------------+                     +-------------------------------+
|     Arduino Uno R3     |                     |    Raspberry Pi 40-Pin Header |
|                        |                     |                               |
|                    GND |-------------------->| Pin 6  (GND)                  |
|             Pin 0 (RX) |<--------------------| Pin 8  (GPIO 14 / UART0 TX)   |
|             Pin 1 (TX) |--[Divider / Shifter]| Pin 10 (GPIO 15 / UART0 RX)   |
+------------------------+                     +-------------------------------+
```

### Mode 1 (SoftwareSerial with USB Debug Console):
```
+------------------------+                     +-------------------------------+
|     Arduino Uno R3     |                     |    Raspberry Pi 40-Pin Header |
|                        |                     |                               |
|                    GND |-------------------->| Pin 6  (GND)                  |
|             Pin 2 (RX) |<--------------------| Pin 8  (GPIO 14 / UART0 TX)   |
|             Pin 3 (TX) |--[Divider / Shifter]| Pin 10 (GPIO 15 / UART0 RX)   |
|              USB Cable |-------------------->| PC (Serial Monitor @ 115200)  |
+------------------------+                     +-------------------------------+
```

### ⚠️ Critical Voltage Level Notice (5.0V vs 3.3V)
* **Raspberry Pi GPIOs are strictly 3.3V and ARE NOT 5V tolerant.**
* **Pi TX (3.3V) $\rightarrow$ Arduino RX**: Safe to connect directly (3.3V meets the ATmega328P input HIGH threshold $V_{IH} \ge 3.0\text{ V}$).
* **Arduino TX (5.0V) $\rightarrow$ Pi RX**: **DO NOT connect directly.** Use either:
  1. A bidirectional logic level shifter (e.g. BSS138), or
  2. A simple 2-resistor voltage divider:
     ```
     Arduino TX Pin ---> [ 1kΩ Resistor ] ---+---> Pi Pin 10 (RX)
                                             |
                                      [ 2kΩ Resistor ]
                                             |
                                            GND
     ```
     This steps $5.0\text{ V}$ down safely to $\approx 3.33\text{ V}$.
* **Alternative (No Resistors Needed!)**: Plug the Arduino directly into one of the Raspberry Pi's USB ports with a USB cable and run the proxy with `--serial /dev/ttyACM0`.

---

## Connecting Mission Planner
1. Connect your PC to the drone Wi-Fi: **`HEXAGON-flight-drone`**.
2. Open **Mission Planner**.
3. In the top right corner, select **UDP** and click **Connect**.
4. Set Port to **`14550`**.
5. Mission Planner will connect, download parameters, and display live HUD telemetry and battery indicators.
