#pragma once
#include "hal/drone_config.h"
#include <mutex>

class PhysicsEngine {
public:
    PhysicsEngine(const Config::DroneHardwareConfig& initial_config);

    // Step the physics simulation forward by dt
    void step(units::time::second_t dt);

    // Runtime configuration updates
    void update_config(const Config::DroneHardwareConfig& new_config);
    Config::DroneHardwareConfig get_config();

    // ---- Inputs from Mock ESCs/Motors ----
    // Set the actual output of a motor (torque or RPM)
    void set_motor_output(int motor_index, units::torque::newton_meter_t torque, units::angular_velocity::revolutions_per_minute_t rpm);

    // ---- Outputs to Mock Sensors ----
    // Get the true simulated state
    struct TrueState {
        Eigen::Matrix<units::length::meter_t, 3, 1> position;
        Eigen::Matrix<units::velocity::meters_per_second_t, 3, 1> velocity;
        Eigen::Matrix<units::acceleration::meters_per_second_squared_t, 3, 1> acceleration;
        
        // Orientation (Roll, Pitch, Yaw)
        Eigen::Matrix<units::angle::radian_t, 3, 1> euler_angles;
        Eigen::Matrix<units::angular_velocity::radians_per_second_t, 3, 1> angular_velocity;
        Eigen::Matrix<units::angular_acceleration::radians_per_second_squared_t, 3, 1> angular_acceleration;
        
        units::current::ampere_t total_current_draw;
        units::voltage::volt_t battery_voltage;
        units::charge::ampere_hour_t consumed_capacity;
    };
    
    TrueState get_true_state();

private:
    std::mutex state_mutex_;
    Config::DroneHardwareConfig config_;
    
    TrueState state_;
    
    // Internal state for motors (simulating spin-up dynamics)
    std::vector<units::angular_velocity::revolutions_per_minute_t> current_motor_rpms_;
    std::vector<units::torque::newton_meter_t> target_motor_torques_;
    
    void calculate_forces_and_torques(
        Eigen::Matrix<units::force::newton_t, 3, 1>& out_force, 
        Eigen::Matrix<units::torque::newton_meter_t, 3, 1>& out_torque);
};
