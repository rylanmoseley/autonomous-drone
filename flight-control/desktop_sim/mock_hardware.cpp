#include "mock_hardware.h"
#include "physics_engine.h"
#include <iostream>

void MockIMU::init() {
    std::cout << "[MockIMU] Initialized." << std::endl;
}

bool MockIMU::update(HAL::IMUData& data) {
    data.timestamp = units::time::millisecond_t(0.0); // Would be actual sim time
    
    data.accel(0) = 0.0f;
    data.accel(1) = 0.0f;
    data.accel(2) = 9.81f; 
    
    data.gyro(0) = 0.0f;
    data.gyro(1) = 0.0f;
    data.gyro(2) = 0.0f;

    data.mag(0) = 0.0f;
    data.mag(1) = 0.0f;
    data.mag(2) = 0.0f;
    
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
    if (engine_) {
        // We pass 0 for torque since the engine now uses target_motor_rpms_
        engine_->set_motor_output(motor_id, units::torque::newton_meter_t(0), speed);
    }
}

units::angular_velocity::revolutions_per_minute_t MockMotors::get_speed(int motor_id) {
    return units::angular_velocity::revolutions_per_minute_t(0.0);
}

units::current::ampere_t MockMotors::get_current(int motor_id) {
    return units::current::ampere_t(0.0);
}

units::temperature::celsius_t MockMotors::get_temperature(int motor_id) {
    return units::temperature::celsius_t(25.0);
}

units::time::millisecond_t MockMotors::get_time_since_last_message(int motor_id) {
    return units::time::millisecond_t(0.0);
}
