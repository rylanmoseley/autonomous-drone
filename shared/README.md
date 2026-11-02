# Shared Contracts & Wire Protocol Specifications

[![Language](https://img.shields.io/badge/C%2B%2B-17%20%2F%20C99-blue.svg)](https://en.cppreference.com/w/cpp/17)
[![Memory Layout](https://img.shields.io/badge/Layout-Packed%20(Zero--Copy)-orange.svg)](#memory-packing--cross-platform-rules)
[![MAVLink](https://img.shields.io/badge/Wire%20Protocol-MAVLink%20v2-green.svg)](file:///home/sandbox/autonomous-drone/shared/include/mavlink_protocol.h)

The **Shared Subproject** defines the single source of truth for all data contracts, Inter-Process Communication (IPC) schemas, coordinate systems, and serialization formats across the autonomous drone system.

Designed for high-throughput, low-latency execution on embedded Linux, all data structures are defined as compact, binary-packed C/C++ structs with zero heap allocation and zero runtime serialization overhead.

---

## Table of Contents

- [Directory Structure](#directory-structure)
- [Subsystem IPC Topology & Network Endpoints](#subsystem-ipc-topology--network-endpoints)
- [Header Files & Data Contracts](#header-files--data-contracts)
  - [1. `common_types.h` (Vectors, Orientations, Shapes)](#1-commontypesh-vectors-orientations-shapes)
  - [2. `vision_packet.h` (Gates & AprilTags)](#2-visionpacketh-gates--apriltags)
  - [3. `ui_protocol.h` (Operator Command & Status)](#3-uiprotocolh-operator-command--status)
  - [4. `mavlink_protocol.h` (MAVLink v2 Wire Engine)](#4-mavlinkprotocolh-mavlink-v2-wire-engine)
  - [5. `telemetry_packet.h` (State & Telemetry Broadcast)](#5-telemetrypacketh-state--telemetry-broadcast)
  - [6. `logger_status.h` (Diagnostic Bitmasks)](#6-loggerstatush-diagnostic-bitmasks)
- [Memory Packing & Cross-Platform Rules](#memory-packing--cross-platform-rules)
- [CMake Integration](#cmake-integration)

---

## Directory Structure

```
shared/
├── CMakeLists.txt              # Defines the header-only shared_contracts INTERFACE target
└── include/
    ├── common_types.h          # Mathematical primitives (Vec3f, Quatf) and ShapeType enum
    ├── vision_packet.h         # VisionGoalEstimate, TagMeasurement, VisionPacket schemas
    ├── ui_protocol.h           # UICommandPacket, UIStatusPacket, UICommandType schemas
    ├── mavlink_protocol.h      # Zero-dependency MAVLink v2 encoder/decoder with CRC-16
    ├── telemetry_packet.h      # FlightTelemetryPacket diagnostic logging schema
    └── logger_status.h         # Flight logger status bitmasks
```

---

## Subsystem IPC Topology & Network Endpoints

To maintain strict modularity, independently executing processes exchange structured datagrams over loopback UDP sockets and Unix Domain Sockets:

```
+-------------------------------------------------------------------------------------------------+
|                                     RADXA ZERO 3W (COMPANION SBC)                               |
|                                                                                                 |
|   +-----------------------+                    +--------------------------------------------+   |
|   |   vision-processing   |                    |               autopilot_node               |   |
|   |      (vision_node)    |                    |                                            |   |
|   +-----------+-----------+                    +----+-------------------+-------------------+   |
|               |                                     ^                   |                       |
|               | VisionPacket (UDP :14553)           |                   | TelemetryPacket       |
|               +-------------------------------------+                   | (UDP :14554)          |
|                                                     |                   v                       |
|                                     UIProtocol      |      +----------------------------+       |
|                                     (UDP :14552)    |      |    UdpTelemetryLogger      |       |
|                                                     v      +----------------------------+       |
|                                       +-------------+--------------+                            |
|                                       |       MavlinkDaemon        |                            |
|                                       +-------------+--------------+                            |
+-----------------------------------------------------|-------------------------------------------+
                        Wi-Fi (802.11ac)              | UART (/dev/flightcontroller @ 921,600 baud)
                     UDP :14552 / WebSockets          | MAVLink v2 Packets
                                                      v
                                        +----------------------------+
                                        |  ArduPilot FCU (GN745 AIO) |
                                        +----------------------------+
```

| Channel | Endpoint / Device | Protocol | Message Type | Frequency |
| :--- | :--- | :--- | :--- | :--- |
| **Vision $\to$ Autopilot** | `127.0.0.1:14553` (or `/tmp/vision_to_autopilot.sock`) | Raw UDP / Unix Datagram | [`VisionPacket`](include/vision_packet.h) | 30–60 Hz |
| **UI $\leftrightarrow$ Autopilot** | `192.168.4.1:14552` (or `/tmp/ui_to_autopilot.sock`) | Raw UDP / Unix Datagram | [`UICommandPacket`](include/ui_protocol.h) $\leftrightarrow$ [`UIStatusPacket`](include/ui_protocol.h) | 5–10 Hz |
| **Autopilot $\leftrightarrow$ FCU** | `/dev/flightcontroller` (UART @ 921.6k) | MAVLink v2 Serial Stream | Msg #84, #102, #32, #30, #0 | 20–50 Hz |
| **Autopilot $\to$ Logger** | `127.0.0.1:14554` | Raw UDP Datagram | [`FlightTelemetryPacket`](include/telemetry_packet.h) | 20 Hz |

---

## Header Files & Data Contracts

### 1. `common_types.h` (Vectors, Orientations, Shapes)
Defines basic mathematical structures in **SI units**:
- **[`Vec3f`](file:///home/sandbox/autonomous-drone/shared/include/common_types.h#L11-L15)**: 3D vector `(x, y, z)` in meters.
- **[`Quatf`](file:///home/sandbox/autonomous-drone/shared/include/common_types.h#L17-L22)**: Unit orientation quaternion `(w, x, y, z)`.
- **[`ShapeType`](file:///home/sandbox/autonomous-drone/shared/include/common_types.h#L24-L31)**: Geometric target classification:
  - `SHAPE_UNKNOWN = 0`
  - `SHAPE_CIRCLE = 1`
  - `SHAPE_TRIANGLE = 2`
  - `SHAPE_SQUARE = 3`
- Vector operations: `vec3_add`, `vec3_sub`, `vec3_scale`, `vec3_norm`, `vec3_dist`.

---

### 2. `vision_packet.h` (Gates & AprilTags)
Carries visual detection hypotheses from `vision-processing` to the autopilot:
- **[`VisionGoalEstimate`](file:///home/sandbox/autonomous-drone/shared/include/vision_packet.h#L10-L16)**:
  - `shape_type`: Geometric polygon shape.
  - `position`: Relative $(x, y, z)$ position from camera in meters.
  - `attitude`: Relative $(w, x, y, z)$ orientation quaternion.
  - `confidence`: $[0.0, 1.0]$ detection quality metric.
- **[`TagMeasurement`](file:///home/sandbox/autonomous-drone/shared/include/vision_packet.h#L18-L26)**:
  - `tag_id`: AprilTag / ChArUco identifier ($1 \dots 65535$).
  - `camera_pos_world`: Inverted absolute World NED camera position derived via known tag coordinates.
  - `camera_att_world`: Absolute World NED camera attitude quaternion.
  - `confidence`: Metric based on reprojection error and corner sharpness.
- **[`VisionPacket`](file:///home/sandbox/autonomous-drone/shared/include/vision_packet.h#L28-L34)**:
  - `timestamp_us`: Microsecond frame acquisition timestamp.
  - `gate_count`: Number of active gate detections ($\le 4$).
  - `tag_count`: Number of active tag detections ($\le 4$).
  - Packed arrays of `VisionGoalEstimate` and `TagMeasurement`.

---

### 3. `ui_protocol.h` (Operator Command & Status)
Defines command and status messages exchanged with the operator interface:
- **[`UICommandPacket`](file:///home/sandbox/autonomous-drone/shared/include/ui_protocol.h#L24-L48)**:
  - `command_id`: Monotonic counter for delivery acknowledgments.
  - `command`: `UICommandType` (`ARM`, `DISARM`, `START_MISSION`, `PAUSE_MISSION`, `RESUME_MISSION`, `LAND_NOW`, `ESTOP`, `SET_SEQUENCE`).
  - `sequence_length`: Number of ordered gates ($\le 16$).
  - `goal_sequence[16]`: Array of `ShapeType` enums defining the mission order.
  - `end_action`: Post-mission behavior (0 = Hover, 1 = Auto-Land).
- **[`UIStatusPacket`](file:///home/sandbox/autonomous-drone/shared/include/ui_protocol.h#L51-L75)**:
  - `flight_state`: Master state enum (`STANDBY`, `SEEK`, `PATH`, `HOVER_WAIT`, `LAND`).
  - `is_armed`, `estop_active`: Safety interlock states.
  - `current_goal_idx`, `total_goals`: Mission progress indicators.
  - `battery_millivolts`: Live pack voltage.

---

### 4. `mavlink_protocol.h` (MAVLink v2 Wire Engine)
A lightweight, zero-dependency C++17 implementation of the MAVLink v2 framing protocol with CRC-16 (MCRF4XX) checksum generation:
- **Framing**:
  `[0xFD | LEN | INCOMP_FLAGS | COMP_FLAGS | SEQ | SYSID | COMPID | MSGID (3B) | PAYLOAD | CRC (2B)]`
- **Supported Messages**:
  - `VISION_POSITION_ESTIMATE` (Msg #102): Global NED position, Euler attitude, and 6x6 upper-triangular covariance matrix forwarded to ArduPilot EKF3.
  - `SET_POSITION_TARGET_LOCAL_NED` (Msg #84): Guided velocity and yaw setpoints sent to the FCU.
  - `LOCAL_POSITION_NED` (Msg #32): Vehicle position and velocity decoded from FCU telemetry.
  - `ATTITUDE` (Msg #30): Roll, pitch, yaw, and angular rates decoded from FCU telemetry.
  - `HEARTBEAT` (Msg #0): System status and arming state verification.
- **Conversion Utility**: `tag_to_mavlink_vision_position()` converts [`TagMeasurement`](file:///home/sandbox/autonomous-drone/shared/include/vision_packet.h#L18) directly into [`MavlinkVisionPositionEstimate`](file:///home/sandbox/autonomous-drone/shared/include/mavlink_protocol.h#L143) with dynamic confidence-weighted covariance.

---

### 5. `telemetry_packet.h` (State & Telemetry Broadcast)
Streams real-time drone state over UDP to the diagnostic logger and simulation viewers:
- **[`FlightTelemetryPacket`](file:///home/sandbox/autonomous-drone/shared/include/telemetry_packet.h#L9-L38)**:
  - `timestamp_us`: Microsecond timestamp.
  - `position_ned`, `velocity_ned`: Estimated drone states.
  - `attitude_rpy`: Roll, pitch, and yaw angles in radians.
  - `battery_voltage`, `battery_percentage`: Power metrics.
  - `active_gate_id`, `active_goal_shape`: Navigation tracking targets.
  - `system_status`: Bitmask of operational and warning flags.

---

## Memory Packing & Cross-Platform Rules

To guarantee binary compatibility between the 64-bit companion computer (Radxa Zero 3W / x86_64), 32-bit platforms (Raspberry Pi Zero 2W `armhf`), and the STM32F745 flight controller:

1. **Explicit Alignment**: All IPC struct definitions are wrapped in:
   ```cpp
   #pragma pack(push, 1)
   // Struct fields
   #pragma pack(pop)
   ```
2. **Fixed-Width Types**: Only `<cstdint>` types (`uint8_t`, `uint16_t`, `uint32_t`, `uint64_t`, `float`) are used. Native `int` or `long` types are forbidden.
3. **Floating Point Standard**: All floats conform strictly to **IEEE 754 single-precision 32-bit**.
4. **Endianness**: Network wire transmissions use standard **Little-Endian** byte ordering matching ARM Cortex-A and STM32 architectures.

---

## CMake Integration

The shared library is packaged as an interface library in CMake:

```cmake
# In any subproject CMakeLists.txt:
target_link_libraries(my_subproject PUBLIC shared_contracts)
```

This automatically injects `${CMAKE_SOURCE_DIR}/shared/include` into the include search path of the consuming executable or static library.
