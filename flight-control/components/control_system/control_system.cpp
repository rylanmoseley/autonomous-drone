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
    
    // Dummy PID logic
    units::torque::newton_meter_t desired_torque(0.5);

    if (motors_) {
        for(int i = 0; i < 4; ++i) {
            motors_->set_torque(i, desired_torque);
        }
    }
}
