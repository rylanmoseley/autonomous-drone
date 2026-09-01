#include "control_system/control_system.h"
#include <iostream>

ControlSystem::ControlSystem(HAL::IMUInterface* imu, HAL::MotorInterface* motors)
    : imu_(imu), motors_(motors) {}

void ControlSystem::init() {
    if (imu_) imu_->init();
    if (motors_) motors_->init();
    // std::cout works natively and often routes to UART in ESP-IDF (though ESP_LOGI is preferred for production)
    std::cout << "[ControlSystem] Initialized." << std::endl;
}

void ControlSystem::tick() {
    HAL::IMUData imu_data;

    if (imu_) {
        imu_->update(imu_data);
    }
    
    // Open-loop Motor Mixer (Direct mapping of setpoint to motor speed percentage)
    if (motors_) {
        float throttle = current_cmd_.lz;
        if (throttle < 0) throttle = 0.0f; // Don't allow negative throttle
        
        float max_rpm = 10000.0f; // Typical max RPM, could use config_.motor_params.max_rpm
        
        for (size_t i = 0; i < config_.rotors.size(); ++i) {
            float percent = throttle;
            const auto& r = config_.rotors[i];
            
            // Mix pitch (ay): Front/Back
            if (r.position(0) > 0.01f) percent -= current_cmd_.ay * 0.2f;
            else if (r.position(0) < -0.01f) percent += current_cmd_.ay * 0.2f;
            
            // Mix roll (ax): Left/Right
            if (r.position(1) > 0.01f) percent -= current_cmd_.ax * 0.2f;
            else if (r.position(1) < -0.01f) percent += current_cmd_.ax * 0.2f;
            
            // Mix yaw (az): CW/CCW
            if (r.spins_clockwise) percent += current_cmd_.az * 0.2f;
            else percent -= current_cmd_.az * 0.2f;
            
            // Clamp
            if (percent < 0.0f) percent = 0.0f;
            if (percent > 1.0f) percent = 1.0f;
            
            motors_->set_speed(i, units::angular_velocity::revolutions_per_minute_t(max_rpm * percent));
        }
    }
}
