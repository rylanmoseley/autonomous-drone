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
    void set_torque(int motor_index, units::torque::newton_meter_t output) override;
    units::angular_velocity::revolutions_per_minute_t get_rpm(int motor_index) override;
};
