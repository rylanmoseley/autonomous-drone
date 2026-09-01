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
    void set_torque(int motor_id, units::torque::newton_meter_t output) override;
    void set_speed(int motor_id, units::angular_velocity::revolutions_per_minute_t speed) override;

    units::angular_velocity::revolutions_per_minute_t get_speed(int motor_id) override;
    units::current::amperes_t get_current(int motor_id) override;
    units::temperature::celsius_t get_temperature(int motor_id) override;

    units::time::millisecond_t get_time_since_last_message(int motor_id) override;
};
