#pragma once

#include "hal/imu_interface.h"
#include "hal/motor_interface.h"

class ControlSystem {
public:
    ControlSystem(HAL::IMUInterface* imu, HAL::MotorInterface* motors);
    void init();
    void tick();

private:
    HAL::IMUInterface* imu_;
    HAL::MotorInterface* motors_;
};
