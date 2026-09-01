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

void MockMotors::set_torque(int motor_id, units::torque::newton_meter_t output) {
    // Intentionally empty to avoid spamming the console at 100Hz
}

void MockMotors::set_speed(int motor_id, units::angular_velocity::revolutions_per_minute_t speed) {
    // Intentionally empty
}

units::angular_velocity::revolutions_per_minute_t MockMotors::get_speed(int motor_id) {
    return units::angular_velocity::revolutions_per_minute_t(0.0);
}

units::current::amperes_t MockMotors::get_current(int motor_id) {
    return units::current::amperes_t(0.0);
}

units::temperature::celsius_t MockMotors::get_temperature(int motor_id) {
    return units::temperature::celsius_t(25.0);
}

units::time::millisecond_t MockMotors::get_time_since_last_message(int motor_id) {
    return units::time::millisecond_t(0.0);
}
