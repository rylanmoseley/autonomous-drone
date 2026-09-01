#pragma once
#include "hal/imu_interface.h"
#include "hal/motor_interface.h"
#include "hal/drone_config.h"

struct CommandVector {
    float lx = 0;
    float ly = 0;
    float lz = 0;
    float ax = 0;
    float ay = 0;
    float az = 0;
};

class ControlSystem {
public:
    ControlSystem(HAL::IMUInterface* imu, HAL::MotorInterface* motors);
    void init();
    void tick();
    
    void set_command(const CommandVector& cmd) { current_cmd_ = cmd; }
    void set_config(const Config::DroneHardwareConfig& config) { config_ = config; }

private:
    HAL::IMUInterface* imu_;
    HAL::MotorInterface* motors_;
    CommandVector current_cmd_;
    Config::DroneHardwareConfig config_;
};
