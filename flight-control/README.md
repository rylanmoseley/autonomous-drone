# Flight Control & Stabilization Subsystem

[![Firmware](https://img.shields.io/badge/Firmware-ArduPilot%20(ArduCopter)-green.svg)](https://ardupilot.org/)
[![Hardware](https://img.shields.io/badge/FCU-Flywoo%20GOKU%20GN745%20AIO-blue.svg)](https://flywoo.net/)
[![Sensors](https://img.shields.io/badge/Sensors-MTF--02P%20Flow%20%2B%20VL53L0X%20ToF-orange.svg)](#sensor-suite--indoor-stabilization)

The **Flight Control Subsystem** provides low-level attitude stabilization, motor speed regulation, sensor fusion, and actuator execution for the HEXAGON Flight quadcopter. Built around the **Flywoo GOKU GN745 AIO** flight controller running **ArduPilot (ArduCopter)**, the system fuses downward optical flow and laser Time-of-Flight (ToF) telemetry through ArduPilot's Extended Kalman Filter (EKF3) to achieve steady indoor hovering and position hold in GPS-denied environments.

---

## Table of Contents

- [Hardware Architecture & Avionics](#hardware-architecture--avionics)
- [Sensor Suite & Indoor Stabilization](#sensor-suite--indoor-stabilization)
- [ArduPilot Parameter Configuration (EKF3 & ExternalNav)](#ardupilot-parameter-configuration-ekf3--externalnav)
- [Wiring & Electrical Interfacing](#wiring--electrical-interfacing)
- [Companion Computer Serial Bridge](#companion-computer-serial-bridge)
- [Calibration & Pre-Flight Checkout](#calibration--pre-flight-checkout)
- [Safety Failsafes & Interlocks](#safety-failsafes--interlocks)

---

## Hardware Architecture & Avionics

```
+-----------------------------------------------------------------------------------+
|                            FLYWOO GOKU GN745 AIO FCU                              |
|   - MCU: STM32F745 (32-bit ARM Cortex-M7 @ 216 MHz)                               |
|   - Onboard IMU: InvenSense MPU6000 / BMI270 (Gyro & Accel)                       |
|   - Barometer: DPS310 / SPL06                                                     |
|   - ESC: 40A 4-in-1 BLHeli_S (DShot600 Digital Protocol)                          |
|   - Form Factor: 20x20 mm mounting, 9.4 g total weight                            |
+-------------------+-------------------------------+-------------------------------+
                    |                               |
       UART3 (921,600 baud)           UART1 / I2C   |   DShot600 Motor Lines
                    v                               v   v
+-------------------+-------+       +---------------+-------+     +-----------------+
| Radxa Zero 3W Companion   |       | MicoAir MTF-02P Sensor|     | 4× GR2004 2550KV|
| Computer (Autopilot Node) |       | - Optical Flow (Yaw/XY|     | Brushless Motors|
| /dev/flightcontroller     |       | - VL53L0X Laser ToF   |     | with 3.5" Props |
+---------------------------+       +-----------------------+     +-----------------+
```

---

## Sensor Suite & Indoor Stabilization

Because the quadcopter operates exclusively indoors without GPS:

1. **MicoAir MTF-02P Optical Flow Sensor**:
   - Ingests high-speed downward image gradients to measure relative ground surface displacement.
   - Computes horizontal velocity ($v_x, v_y$) in body frame coordinates.
   - Crucial for satisfying **ER 1** (hover drift $\le \pm 15.0\text{ cm}$) and **ER 22** (drift velocity $\le 5.0\text{ cm/s}$).

2. **VL53L0X Laser Time-of-Flight (ToF) Rangefinder**:
   - Emits $940\text{ nm}$ VCSEL laser pulses to calculate true ground distance with millimeter accuracy up to $2.0\text{ m}$.
   - Supplies precise ground-truth altitude to compensate optical flow scale factors and govern automated landing (descent $v_z \le 0.25\text{ m/s}$, **ER 6**).

3. **Onboard Inertial Measurement Unit (IMU)**:
   - High-rate gyro and accelerometer streams ($1\text{ kHz}$) providing angular rate and tilt estimates for inner attitude loops.

---

## ArduPilot Parameter Configuration (EKF3 & ExternalNav)

To enable stable GPS-denied navigation and external visual position corrections from the companion computer:

```ini
# Enable EKF3 state estimator
EK3_ENABLE = 1
AHRS_EKF_TYPE = 3

# Optical Flow Configuration (MTF-02P)
FLOW_TYPE = 5             # MicoAir / MSP optical flow protocol
FLOW_FXSCALER = 0         # Calibrated focal scaling
FLOW_FYSCALER = 0
RNGFND1_TYPE = 24         # VL53L0X / MSP Rangefinder
RNGFND1_MIN_CM = 5
RNGFND1_MAX_CM = 200

# Primary Navigation Source Selection
EK3_SRC1_POSXY = 0        # None (Optical flow velocity primary)
EK3_SRC1_VELXY = 5        # Optical Flow
EK3_SRC1_POSZ = 1         # Rangefinder / ToF
EK3_SRC1_YAW = 1          # Compass (or disabled indoors via EK3_SRC1_YAW = 0)

# External Vision Fusion (Global AprilTag Drift Reset)
VISO_TYPE = 1             # MAVLink external vision enabled
VISO_POS_M_NSE = 0.10     # External vision position measurement noise (0.1 m)
VISO_YAW_M_NSE = 0.05     # External vision yaw measurement noise (0.05 rad)
EK3_SRC2_POSXY = 6        # ExternalNav (active on AprilTag sighting)
EK3_SRC2_YAW = 6          # ExternalNav for absolute heading alignment

# High-Speed MAVLink Serial Port Configuration (TELEM1 / UART3)
SERIAL2_PROTOCOL = 2      # MAVLink 2
SERIAL2_BAUD = 921        # 921,600 baud
BRD_SAFETY_DEFLT = 0      # Disable hardware safety switch for autonomous arming
```

---

## Wiring & Electrical Interfacing

```
Flywoo GOKU GN745 Pinout:
---------------------------------------------------------------------------------
Pad Name    Connected To              Function
---------------------------------------------------------------------------------
VBAT        LiPo Battery (+)          14.8V (4S) raw battery power
GND         LiPo Battery (-)          Common ground reference
TX2 / RX2   Radxa Zero 3W Header      MAVLink v2 telemetry (921,600 baud)
TX1 / RX1   MTF-02P Optical Flow      Sensor UART datastream
SCL / SDA   VL53L0X ToF (if I2C)      Laser rangefinder bus
5V          Flywoo 5V/2A BEC          Regulated SBC & sensor rail
M1 - M4     GR2004 Brushless Motors   DShot600 motor signals
---------------------------------------------------------------------------------
```

---

## Companion Computer Serial Bridge

The Autopilot connects to the FCU through [`MavlinkDaemon`](file:///home/sandbox/autonomous-drone/autopilot/mavlink-daemon/mavlink_daemon.h) running on the Radxa Zero 3W:

- **UART Device**: `/dev/flightcontroller` (created dynamically via udev rule).
- **Baud Rate**: `921600` baud.
- **Flight Commands**: In `GUIDED` mode, `AutopilotNode` transmits velocity commands via `SET_POSITION_TARGET_LOCAL_NED` (Msg #84) at 20 Hz.
- **Visual Position Resets**: Incoming AprilTag observations from `vision-processing` are formatted into `VISION_POSITION_ESTIMATE` (Msg #102) and pushed to ArduPilot, immediately zeroing accumulated dead-reckoning drift.

---

## Calibration & Pre-Flight Checkout

Prior to flight tests, execute the following procedures in **Mission Planner** or **QGroundControl**:

1. **Accelerometer Calibration**: Level the airframe across 6 orthogonal orientations.
2. **Optical Flow Calibration**:
   - Suspend the drone at $1.0\text{ m}$ height over textured ground.
   - Tilt the vehicle gently by hand in roll/pitch; verify that flow sensor body rates track IMU gyro rates (`OF.flowX` vs `IMU.gyroX`).
3. **Laser Rangefinder Check**: Verify distance readout against a physical tape measure from $0.1\text{ m}$ to $2.0\text{ m}$.
4. **Motor Spin Direction**: Verify clockwise rotation on motors 1 & 4, counterclockwise on motors 2 & 3 using the motor test tab.

---

## Safety Failsafes & Interlocks

- **Loss of Companion Telemetry**: If MAVLink setpoints cease for $> 200\text{ ms}$, ArduPilot automatically enters `LAND` or `ALTHOLD` mode (**ER 9**).
- **Emergency Stop Override**: The operator UI E-Stop dispatches an immediate disarm packet (`MAV_CMD_COMPONENT_ARM_DISARM`, param1=0, force=21196), cutting motor power within $\le 150\text{ ms}$ (**ER 10**).
- **Low Battery Failsafe**: Monitors individual cell voltages; triggers controlled descent at $3.5\text{ V/cell}$.
