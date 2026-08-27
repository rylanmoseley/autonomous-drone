# Flight Controller Firmware Architecture

The flight controller firmware is the brain of the autonomous drone. To ensure stability, testability, and clean code, the firmware architecture is highly modularized.

This directory contains the documentation for each of the core sub-projects (components) that make up the flight controller:

* **[[build-system]]**: Toolchains, environments, and CMake structure for ESP32 and native desktop builds.
* **[[simulation]]**: The native desktop Software-In-The-Loop (SITL) environment for testing logic without hardware.
* **[[math-physics]]**: Platform-agnostic kinematics, filtering, and matrix math.
* **[[flight-logic]]**: Platform-agnostic PID controllers, state machines, and motor mixers.
* **[[sensor-reading]]**: ESP32-specific hardware abstraction layer (HAL) for reading the IMU and ToF sensors.
* **[[external-interaction]]**: ESP32-specific handlers for UART (Vision Processor), CAN (ESCs), and ESP-NOW (Telemetry).
* **[[logging-and-telemetry]]**: Requirements and architecture for the onboard blackbox and live telemetry streams.
