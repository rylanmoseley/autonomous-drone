#include "mock_hardware.h"
#include <iostream>

void MockIMU::init() {
    std::cout << "[MockIMU] Initialized." << std::endl;
}

void MockIMU::read_accel(float& x, float& y, float& z) {
    x = 0.0f; y = 0.0f; z = 9.81f; // Simulated gravity
}

void MockIMU::read_gyro(float& x, float& y, float& z) {
    x = 0.0f; y = 0.0f; z = 0.0f;
}

void MockMotors::init() {
    std::cout << "[MockMotors] Initialized." << std::endl;
}

void MockMotors::set_throttle(int motor_index, float percentage) {
    // Intentionally empty to avoid spamming the console at 100Hz
}
