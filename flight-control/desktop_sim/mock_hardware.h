#pragma once
#include "hal/imu_interface.h"
#include "hal/motor_interface.h"

class MockIMU : public HAL::IMUInterface {
public:
    void init() override;
    void read_accel(float& x, float& y, float& z) override;
    void read_gyro(float& x, float& y, float& z) override;
};

class MockMotors : public HAL::MotorInterface {
public:
    void init() override;
    void set_throttle(int motor_index, float percentage) override;
};
