#pragma once
#include <units.h>
#include <Eigen/Dense>
#include <vector>

namespace Config {

struct RotorConfig {
    // Position of the rotor relative to the Center of Mass
    Eigen::Vector3f position;
    
    // Direction the thrust is applied (usually [0, 0, -1] for NED)
    Eigen::Vector3f thrust_axis;
    
    // Aerodynamic constants
    float thrust_coefficient; // Thrust = k_f * omega^2
    float torque_coefficient; // Torque = k_m * omega^2
    
    bool spins_clockwise;
};

struct MotorESCConfig {
    // If the ESC supports closed-loop torque control, we can abstract away KV/Resistance.
    // However, for physical bounds, we still need limits.
    bool supports_torque_control;
    
    units::angular_velocity::revolutions_per_minute_t max_rpm;
    units::torque::newton_meter_t max_torque;
    
    // Simplified First-Order Motor Dynamics (spin-up time constant)
    units::time::second_t time_constant;
};

struct BatteryConfig {
    units::voltage::volt_t nominal_voltage;
    units::charge::ampere_hour_t capacity;
    units::impedance::ohm_t internal_resistance; // For voltage sag simulation
};

struct ProtocolConfig {
    // Simulating communication bus limits (SPI/I2C/CAN)
    units::frequency::hertz_t imu_update_rate;
    units::frequency::hertz_t esc_update_rate;
    units::frequency::hertz_t command_telemetry_rate;
    
    // Simulated latency
    units::time::millisecond_t imu_latency;
};

struct DroneHardwareConfig {
    units::mass::kilogram_t mass;
    Eigen::Matrix3f moment_of_inertia; // 3x3 inertia tensor (kg * m^2)
    
    // Aerodynamic properties
    float linear_drag_coefficient; // Simplified linear drag (N / (m/s))
    
    std::vector<RotorConfig> rotors;
    MotorESCConfig motor_params;
    BatteryConfig battery_params;
    ProtocolConfig protocol_params;
};

} // namespace Config
