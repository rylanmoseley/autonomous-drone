# Flight Logic

The flight logic sub-project contains the core decision-making algorithms for the drone.

## Components
* **State Machine**: Manages the high-level state of the drone (e.g., Armed, Disarmed, Failsafe, Auto-Hover, Auto-Navigate).
* **PID Controllers**: Independent Proportional-Integral-Derivative loops for Roll, Pitch, Yaw, and Altitude.
* **Motor Mixer**: A matrix that maps the requested Roll/Pitch/Yaw forces into throttle percentages for the 4 individual motors.

## Constraints
Like the Math & Physics module, the Flight Logic must be **platform-agnostic**. It interacts with the outside world purely through abstract interfaces defined by the HAL (Hardware Abstraction Layer).
