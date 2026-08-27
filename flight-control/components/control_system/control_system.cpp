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
    float ax, ay, az;
    float gx, gy, gz;

    if (imu_) {
        imu_->read_accel(ax, ay, az);
        imu_->read_gyro(gx, gy, gz);
    }
    
    // Dummy PID logic
    float desired_throttle = 0.5f;

    if (motors_) {
        for(int i = 0; i < 4; ++i) {
            motors_->set_throttle(i, desired_throttle);
        }
    }
}
