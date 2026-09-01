#include "mock_hardware.h"
#include <iostream>

void MockIMU::init() {
    std::cout << "[MockIMU] Initialized." << std::endl;
}

bool MockIMU::update(HAL::IMUData& data) {
    data.timestamp = units::time::millisecond_t(0.0); // Would be actual sim time
    
    data.accel(0) = units::acceleration::meters_per_second_squared_t(0.0);
    data.accel(1) = units::acceleration::meters_per_second_squared_t(0.0);
    data.accel(2) = units::acceleration::meters_per_second_squared_t(-9.81);
    
    data.gyro(0) = units::angular_velocity::radians_per_second_t(0.0);
    data.gyro(1) = units::angular_velocity::radians_per_second_t(0.0);
    data.gyro(2) = units::angular_velocity::radians_per_second_t(0.0);
    
    data.mag(0) = units::magnetic_field::gauss_t(0.0);
    data.mag(1) = units::magnetic_field::gauss_t(0.0);
    data.mag(2) = units::magnetic_field::gauss_t(0.0);
    
    data.temperature = units::temperature::celsius_t(25.0);
    
    return true;
}

void MockIMU::calibrate_gyro() {
    std::cout << "[MockIMU] Calibrating Gyro..." << std::endl;
}

void MockIMU::calibrate_accel() {
    std::cout << "[MockIMU] Calibrating Accel..." << std::endl;
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
