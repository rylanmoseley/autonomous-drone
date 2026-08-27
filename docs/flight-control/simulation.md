# Simulation (SITL)

The Software-In-The-Loop (SITL) simulation allows developers to run and test the flight control logic without flashing the ESP32.

## How it Works
The simulation runs as a standard desktop executable. It instantiates the core `ControlSystem` but injects **Mock Hardware Drivers** instead of real I2C/SPI drivers.

* **Mock Sensors**: Feed fake gravity or user-input angles into the flight logic.
* **Mock Motors**: Print the expected thrust output to the console instead of spinning real props.

This enables high-speed iteration on PID tuning, state machine logic, and failsafe testing.
