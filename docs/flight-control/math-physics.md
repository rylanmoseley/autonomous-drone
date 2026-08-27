# Math & Physics

This sub-project handles all the heavy computational lifting for the drone. 

## Core Responsibilities
* **Kinematics**: Converting raw gyroscope rates and accelerometer vectors into a usable 3D orientation (Euler angles and Quaternions).
* **Filtering**: Implementing sensor fusion algorithms (e.g., Mahony, Madgwick, or Kalman filters) to eliminate noise and vibration from the raw sensor data.
* **Math Primitives**: Custom vector and matrix operations tailored for 3D flight dynamics.

## Constraints
This component must be **platform-agnostic** (pure C++). It cannot include any ESP32-specific headers, ensuring it can be unit-tested seamlessly in the desktop simulation.
