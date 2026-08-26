# Autonomous Drone System Overview

## Project Requirements
### 1. Maximum Weight 250g
The drone must be small ($\le$250 grams). It must be sturdy enough to fly reliably.
### 2. Drone Flight
The drone must fly indoors in a stable and controllable way. It must read from sensors (IMU, distance to ground) and output via rotor-attached motors.
### 3. Image Recognition
The drone must identify shapes, and identify the order of the shapes.
### 4. Autonomous Navigation
The drone must be able to move autonomously in the shapes, in the order they are shown.

## Implicit and Optional Requirements
### 5. Flight Time
The drone's battery must last long enough to complete the flight. A drone of this size, with these capabilities should be able to run for 5-10 minutes?
### 6. Flight Telemetry
The drone should report live, useful telemetry data to a computer for diagnostics, as well as record to an SD card.
### 7. Software Simulation
The drone's Flight Control firmware should be able to be run in a desktop simulation environment, for testing and hardware-independant development.
### 8. Localization/SLAM - Optional
The drone should be able to measure its own position with the camera for autonomous tracking. If this is not acheived, dead-reckoning can be used for navigation.
