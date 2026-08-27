# Logging & Telemetry

The Logging and Telemetry sub-project handles the recording of flight data to the onboard SD card (Blackbox) and the live streaming of data to the ground station.

## Core Requirements

### Onboard Blackbox Logging
- **Binary Format**: Logs must be written in a dense, binary format to maximize SD card write speed and minimize CPU overhead.
- **Post-Run Parsing**: A desktop utility will be required to parse the binary logs into human-readable formats (e.g., CSV or JSON) after the flight.
- **Comprehensive Data**: *Everything* must be logged onboard. This includes:
  - All raw and filtered sensor readings
  - All FCU outputs (motor commands)
  - All discrete "events" (e.g., base-station connect/disconnect, received commands)
- **Log Replay**: The native desktop simulation must be capable of ingesting these logs to replay flights, ensuring that all flight-control logic is completely deterministic and debuggable.

### Live Telemetry Streaming
- **Live-Swappable Fidelity**: The bandwidth usage of the live telemetry stream (Drone <--> Laptop base station) must be adjustable on the fly to adapt to RF interference:
  - *Low-Fidelity*: Minimal bandwidth usage. Streams only critical data (e.g., battery voltage, current state).
  - *Balanced-Fidelity*: High-frequency critical data, low-frequency optional data.
  - *High-Fidelity*: Maximum bandwidth usage. Streams all high-frequency data (essentially mirroring the blackbox over the air).

### Vision Processor Separation
- The Vision Processor (CM4) must maintain its own independent logs.
- The FCU will treat the Vision Processor exactly like a standard "sensor," ensuring the flight control logic remains deterministic regardless of the CM4's internal state.

## Log Structure

### Header / Metadata
Every individual log file must begin with a metadata header containing:
- **Date/Time Identifier**: When the log was created.
- **Drone "Mode"**: The startup mode of the drone.
- **Firmware Hash**: The Git commit hash of the deployed firmware to ensure perfect traceability for simulation replays.

### Data Entries
Every individual data entry appended to the log must contain:
- **Timestamp**: Highly precise time since boot.
- **Label / Identifier**: What the data represents (e.g., `IMU_ACCEL_Z`, `MOTOR_1_CMD`).
- **Input / Output Boolean**: Whether this data was read from a sensor (Input) or sent to an actuator/system (Output).
- **Value**: The actual numeric payload.
