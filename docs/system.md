# Autonomous Drone System Overview

## Project Requirements
### 1. Maximum Weight 250g
The drone must be small ( $\le$ 250 grams). It must be sturdy enough to fly reliably.
### 2. Drone Flight
The drone must fly indoors in a stable and controllable way. It must read from sensors (IMU, distance to ground) and output via rotor-attached motors.
### 3. Image Recognition & Autonomy Workflow
The drone must be capable of a hands-off demonstration workflow: Power-on -> show shape pattern to the drone (initiation method at group's discretion) -> drone executes the pattern autonomously. 

### 4. Autonomous Navigation
The drone must be able to move autonomously to trace the shapes, in the order they are shown. This must be possible without any external connection.

## Implicit and Optional Requirements
### 5. Flight Time
The drone's battery must last long enough to complete the flight. A drone of this size, with these capabilities should be able to run for 5-10 minutes?
### 6. Flight Telemetry & Comms
The drone should report (optional) live, useful telemetry data to a computer for diagnostics, as well as record to an SD card. The demonstration room may have heavy RF interference from concurrent demos. The telemetry system must be stable under these conditions.
### 7. Software Simulation
The drone's Flight Control firmware should be able to be run in a desktop simulation environment, for testing and hardware-independant development.
### 8. Localization/SLAM - Optional
The drone should be able to measure its own position with the camera for autonomous tracking. If this is not acheived, dead-reckoning can be used for navigation.

## Environment & Safety Requirements
### 9. Operating Environment
The drone will operate in classroom or ballroom lighting. The floor and wall surfaces are dull but not matte. (Note: A patterned mat may be introduced for the demo if optical flow/tracking issues arise). There is no strict volumetric limit for the drone, only the 250g weight limit.

### 10. Safety & Failsafes
The drone must support two primary failsafe states: "stop immediately" (disarm motors) and "land immediately". These states must be invokable by the base station. Because of potential RF interference, an additional, highly resilient mechanism for emergency stop should be explored.

## Subsystems

This section outlines the 6 core subsystems, their responsibilities, interactions, and current research considerations.

### 1. Flight Controller (FCU)
* **Domain**: Firmware, Core Logic, Integration, Telemetry
* **Overview**: The central brain for flight stability. Reads sensor data, computes PID loops, and sends throttle commands to the ESCs.
* **Hardware Candidate**: ESP32-S3 (Dual-core allows separation of control loops and comms) or other embedded MCU.
* **Interactions**:
    * **Sensors**: Reads IMU and ToF/LiDAR via I2C/SPI.
    * **Motors/ESCs**: Outputs CAN or UART signals, or possibly PWM.
    * **Vision Processor**: Receives navigation setpoints via UART.
    * **Ground Station**: Sends telemetry via Wi-Fi/ESP-NOW/low-bandwidth radio.
* **Research Notes**: Firmware should be developed with a Hardware Abstraction Layer (HAL) to allow for desktop SITL (Software-In-The-Loop) simulation before physical hardware arrives.

### 2. Vision Processor
* **Domain**: Image Recognition, SLAM, Autonomous Navigation Setpoints
* **Overview**: Processes camera feed to identify shapes and determine the drone's position, calculating where the drone should move next.
* **Hardware Candidate**: Raspberry Pi CM4 + Global Shutter Camera, or similar vision processor.
* **Interactions**:
    * **FCU**: Sends 3D displacement vectors and the identified shape sequence over UART.
    * **Power**: Requires stable 5V from the BMS/PDB.
* **Research Notes**: The CM4 is powerful but heavy and power-hungry. To meet the <250g requirement, a custom, ultra-light carrier board is likely mandatory. If SLAM proves too heavy, consider falling back to a Raspberry Pi Zero 2 W for simple shape detection and relying on dead-reckoning. 
* **Interaction Note**: UART comms between CM4 and ESP32 should not use raw strings. Recommend **COBS** framing with **Nanopb** or a packed C-struct.

### 3. Sensors
* **Domain**: IMU, Downward Distance Sensor (ToF/LiDAR), Optical Flow (Optional)
* **Overview**: Provides the raw data required for the FCU to maintain stable indoor flight.
* **Interactions**:
    * **FCU**: Data transmission via I2C or SPI.
* **Research Notes**: Needs extremely low latency. For indoor flight, a downward-facing Time-of-Flight (ToF) or LiDAR sensor is heavily recommended over a barometric altimeter to avoid HVAC pressure issues and ground-effect turbulence.

### 4. Motors & ESCs
* **Domain**: Propulsion
* **Overview**: Converts FCU commands into physical thrust.
* **Interactions**:
    * **FCU**: Receives throttle/velocity signals via **CAN** or **UART**.
    * **Power**: Draws high current directly from the battery/BMS.
* **Research Notes**: Given the 250g weight limit and 5-10 minute flight time requirement, a commercial **CAN/UART FOC (Field-Oriented Control) BLDC 4-in-1 ESC** is highly recommended. A custom CAN / FOC BLDC ESC is pretty complex but not impossible.

### 5. Battery & Power Distribution (BMS/PDB)
* **Domain**: Power Management
* **Overview**: Supplies clean power to all components and monitors battery health.
* **Interactions**:
    * **ESCs**: Provides raw unregulated voltage.
    * **CM4**: Provides regulated 5V (high current capable).
    * **FCU/Sensors**: Provides regulated 3.3V.
* **Research Notes**: Must be extremely light. Look into 2S LiPo batteries common in sub-250g FPV drones.

### 6. Chassis & Mechanics
* **Domain**: Frame, Mounting, Weight Budget
* **Overview**: The physical structure of the drone.
* **Interactions**: Houses all components securely, minimizes vibration to the IMU, and protects the camera.
* **Research Notes**: The absolute hard limit is 250g. A detailed weight budget (Bill of Materials with weights) must be established immediately before components are finalized. 3D printed frames (e.g., carbon fiber infused PETG or standard PLA) or custom cut carbon-fiber plates are viable.
