#pragma once
#include "hal/imu_interface.h"
#include "hal/motor_interface.h"

class PhysicsEngine; // Forward declaration

class MockIMU : public HAL::IMUInterface {
public:
    MockIMU(PhysicsEngine* engine = nullptr) : engine_(engine) {}
    void init() override;
    bool update(HAL::IMUData& data) override;
    void calibrate_gyro() override;
    void calibrate_accel() override;
private:
    PhysicsEngine* engine_;
};

class PhysicsEngine; // Forward declaration

class MockMotors : public HAL::MotorInterface {
public:
    MockMotors(PhysicsEngine* engine = nullptr) : engine_(engine) {}
    void init() override;
    void set_torque(int motor_id, units::torque::newton_meter_t output) override;
    void set_speed(int motor_id, units::angular_velocity::revolutions_per_minute_t speed) override;

    units::angular_velocity::revolutions_per_minute_t get_speed(int motor_id) override;
    units::current::ampere_t get_current(int motor_id) override;
    units::temperature::celsius_t get_temperature(int motor_id) override;

    units::time::millisecond_t get_time_since_last_message(int motor_id) override;

private:
    PhysicsEngine* engine_;
};
