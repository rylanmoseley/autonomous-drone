# Simulation (SITL)

The Software-In-The-Loop (SITL) simulation allows developers to run and test the flight control logic without flashing the ESP32.

## How it Works (Current)
The simulation runs as a standard desktop executable. It instantiates the core `ControlSystem` but injects **Mock Hardware Drivers** instead of real I2C/SPI drivers.

* **Mock Sensors**: Feed fake gravity or user-input angles into the flight logic.
* **Mock Motors**: Print the expected thrust output to the console instead of spinning real props.

This enables high-speed iteration on PID tuning, state machine logic, and failsafe testing.

## Simulator Specification Draft
*Note: This is a working specification for the next iteration of the desktop simulator.*

### 1. Physics Engine Integration
The simulator must model the rigid body dynamics of the drone.
*   **Kinematics & Dynamics:** Need a basic physics loop (or integration with a library like Bullet/MuJoCo) to translate motor torque and RPM into thrust, and consequently into drone attitude/position changes.
*   **Time Management:** Must support fixed-step simulation execution, feeding a precise `dt` to the flight controller to ensure deterministic behavior.

### 2. Visualization and Telemetry
*   **Output:** The mock sensors should not just print to console, but stream state data (attitude, position, motor outputs) over UDP/TCP.
*   **External Tooling:** We should define a protocol to view this data in a tool like Foxglove Studio or a custom 3D visualizer.

### 3. Input Injection
*   **Manual Control:** Ability to map a connected USB gamepad/joystick to simulated RC inputs.
*   **Automated Testing:** Provide an API or scriptable interface to inject specific setpoints and test edge cases (e.g., motor failure, extreme wind disturbance).

### 4. Component Interfaces
*   **Mock IMU (`MockIMU`)**: Reads the attitude/acceleration from the physics engine and adds simulated noise/bias before feeding it to the control system.
*   **Mock Motors (`MockMotors`)**: Takes the speed/torque commands from the control system and passes them to the physics engine to calculate thrust vectors.
