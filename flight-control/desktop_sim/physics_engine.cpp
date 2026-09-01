#include "physics_engine.h"
#include <iostream>

PhysicsEngine::PhysicsEngine(const Config::DroneHardwareConfig& initial_config) 
    : config_(initial_config) 
{
    current_motor_rpms_.resize(config_.rotors.size(), units::angular_velocity::revolutions_per_minute_t(0));
    target_motor_rpms_.resize(config_.rotors.size(), units::angular_velocity::revolutions_per_minute_t(0));
    
    // Initialize true state to zero
    state_.position.setZero();
    state_.velocity.setZero();
    state_.acceleration.setZero();
    state_.euler_angles.setZero();
    state_.angular_velocity.setZero();
    state_.angular_acceleration.setZero();
    state_.total_current_draw = units::current::ampere_t(0);
    state_.battery_voltage = config_.battery_params.nominal_voltage;
    state_.consumed_capacity = units::charge::ampere_hour_t(0);
}

void PhysicsEngine::update_config(const Config::DroneHardwareConfig& new_config) {
    std::lock_guard<std::mutex> lock(state_mutex_);
    config_ = new_config;
    // Resize vectors if rotor count changed
    current_motor_rpms_.resize(config_.rotors.size(), units::angular_velocity::revolutions_per_minute_t(0));
    target_motor_rpms_.resize(config_.rotors.size(), units::angular_velocity::revolutions_per_minute_t(0));
}

Config::DroneHardwareConfig PhysicsEngine::get_config() {
    std::lock_guard<std::mutex> lock(state_mutex_);
    return config_;
}

void PhysicsEngine::set_motor_output(int motor_index, units::torque::newton_meter_t torque, units::angular_velocity::revolutions_per_minute_t rpm) {
    std::lock_guard<std::mutex> lock(state_mutex_);
    if (motor_index >= 0 && motor_index < target_motor_rpms_.size()) {
        target_motor_rpms_[motor_index] = rpm;
    }
}

PhysicsEngine::TrueState PhysicsEngine::get_true_state() {
    std::lock_guard<std::mutex> lock(state_mutex_);
    return state_;
}

void PhysicsEngine::calculate_forces_and_torques(
    Eigen::Vector3f& out_force, 
    Eigen::Vector3f& out_torque) 
{
    out_force.setZero();
    out_torque.setZero();
    
    // 1. Gravity (Assume Z-Up coordinate system: gravity is negative Z)
    out_force(2) = (config_.mass * units::acceleration::meters_per_second_squared_t(-9.81)).to<float>();
    
    // 2. Aerodynamic Drag (Simplified linear and angular drag model)
    out_force(0) -= config_.linear_drag_coefficient * state_.velocity(0);
    out_force(1) -= config_.linear_drag_coefficient * state_.velocity(1);
    out_force(2) -= config_.linear_drag_coefficient * state_.velocity(2);
    
    out_torque(0) -= config_.angular_drag_coefficient * state_.angular_velocity(0);
    out_torque(1) -= config_.angular_drag_coefficient * state_.angular_velocity(1);
    out_torque(2) -= config_.angular_drag_coefficient * state_.angular_velocity(2);
    
    Eigen::Vector3f body_thrust_force(0.0f, 0.0f, 0.0f);
    
    // 3. Rotor forces and torques
    for (size_t i = 0; i < config_.rotors.size(); ++i) {
        auto rpm = current_motor_rpms_[i].to<float>();
        // Convert RPM to rad/s for calculations
        float omega = rpm * (2.0f * M_PI / 60.0f);
        
        // Thrust = k_f * w^2
        float thrust_mag = config_.rotors[i].thrust_coefficient * (omega * omega);
        Eigen::Vector3f thrust_vec = config_.rotors[i].thrust_axis * thrust_mag;
        
        // Accumulate in body frame
        body_thrust_force += thrust_vec;
        
        // Structural Torque = r x F
        Eigen::Vector3f position_vec = config_.rotors[i].position;
        Eigen::Vector3f structural_torque = position_vec.cross(thrust_vec);
        
        // Drag Torque (Yaw authority) = k_m * w^2
        float drag_torque_mag = config_.rotors[i].torque_coefficient * (omega * omega);
        // Reaction torque direction: if rotor spins CW, frame feels CCW torque
        float torque_direction = config_.rotors[i].spins_clockwise ? -1.0f : 1.0f;
        
        // Assuming rotors are mounted perfectly vertically for drag torque (aligned with Z)
        Eigen::Vector3f drag_torque(0.0f, 0.0f, drag_torque_mag * torque_direction);
        
        out_torque(0) += structural_torque(0) + drag_torque(0);
        out_torque(1) += structural_torque(1) + drag_torque(1);
        out_torque(2) += structural_torque(2) + drag_torque(2);
    }
    
    // Convert body frame thrust to global frame
    Eigen::Matrix3f R;
    R = Eigen::AngleAxisf(state_.euler_angles(2), Eigen::Vector3f::UnitZ())
      * Eigen::AngleAxisf(state_.euler_angles(1), Eigen::Vector3f::UnitY())
      * Eigen::AngleAxisf(state_.euler_angles(0), Eigen::Vector3f::UnitX());
      
    Eigen::Vector3f global_thrust = R * body_thrust_force;
    out_force += global_thrust;
}

void PhysicsEngine::step(units::time::second_t dt) {
    std::lock_guard<std::mutex> lock(state_mutex_);
    
    float dt_sec = dt.to<float>();
    float electrical_power = 0.0f;
    float mechanical_power = 0.0f;
    
    for (size_t i = 0; i < config_.rotors.size(); ++i) {
        float target_rpm_val = target_motor_rpms_[i].to<float>();
        auto current_rpm = current_motor_rpms_[i];
        
        // Simple first order response for motor RPM
        float alpha = dt_sec / (config_.motor_params.time_constant.to<float>() + dt_sec);
        float next_rpm = current_rpm.to<float>() + alpha * (target_rpm_val - current_rpm.to<float>());
        
        current_motor_rpms_[i] = units::angular_velocity::revolutions_per_minute_t(next_rpm);
        
        // Power = Torque * Omega (assuming torque_coefficient * w^2 is the load torque)
        float omega = next_rpm * (2.0f * M_PI / 60.0f);
        float load_torque = config_.rotors[i].torque_coefficient * (omega * omega);
        mechanical_power += (load_torque * omega);
    }
    
    // 2. Calculate sum of forces and torques
    Eigen::Vector3f total_force;
    Eigen::Vector3f total_torque;
    calculate_forces_and_torques(total_force, total_torque);
    
    // 3. Integrate Rigid Body Dynamics
    // Linear
    Eigen::Vector3f a(
        total_force(0) / config_.mass.to<float>(),
        total_force(1) / config_.mass.to<float>(),
        total_force(2) / config_.mass.to<float>()
    );
    
    state_.acceleration = a;
    
    state_.velocity += state_.acceleration * dt_sec;
    state_.position += state_.velocity * dt_sec;
    
    // Simple ground constraint (Z-Up coordinate system: Z < 0 is underground)
    if (state_.position(2) < 0.0f) {
        state_.position(2) = 0.0f; // Rest on ground
        
        if (state_.velocity(2) < 0.0f) {
            state_.velocity(2) = 0.0f; // Stop falling
            state_.acceleration(2) = 0.0f;
        }
        
        // Apply heavy friction/damping when resting on the ground
        state_.velocity(0) *= 0.9f;
        state_.velocity(1) *= 0.9f;
        state_.angular_velocity *= 0.9f;
        state_.euler_angles(0) *= 0.9f; // Force level roll
        state_.euler_angles(1) *= 0.9f; // Force level pitch
    }
    
    // Ceiling constraint (10m)
    if (state_.position(2) > 10.0f) {
        state_.position(2) = 10.0f;
        if (state_.velocity(2) > 0.0f) state_.velocity(2) = 0.0f;
    }
    
    // Wall constraints (20x20m grid -> +/- 10m on X and Y)
    if (state_.position(0) > 10.0f) {
        state_.position(0) = 10.0f;
        if (state_.velocity(0) > 0.0f) state_.velocity(0) = 0.0f;
    } else if (state_.position(0) < -10.0f) {
        state_.position(0) = -10.0f;
        if (state_.velocity(0) < 0.0f) state_.velocity(0) = 0.0f;
    }
    
    if (state_.position(1) > 10.0f) {
        state_.position(1) = 10.0f;
        if (state_.velocity(1) > 0.0f) state_.velocity(1) = 0.0f;
    } else if (state_.position(1) < -10.0f) {
        state_.position(1) = -10.0f;
        if (state_.velocity(1) < 0.0f) state_.velocity(1) = 0.0f;
    }
    
    // Angular: I * omega_dot + omega x (I * omega) = tau
    Eigen::Vector3f tau = total_torque;
    Eigen::Vector3f omega = state_.angular_velocity;
    
    // omega_dot = I^-1 * (tau - omega x (I * omega))
    Eigen::Vector3f angular_accel = config_.moment_of_inertia.inverse() * (tau - omega.cross(config_.moment_of_inertia * omega));
    
    state_.angular_acceleration = angular_accel;
    
    state_.angular_velocity += state_.angular_acceleration * dt_sec;
    
    // Euler angle integration (simple accumulation for small angles, 
    // ideally should use quaternion derivative for large rotations)
    state_.euler_angles += state_.angular_velocity * dt_sec;
    
    // 4. Update Battery Simulation
    // Assume electrical power = mechanical power / efficiency (e.g. 80%)
    float efficiency = 0.8f;
    electrical_power = mechanical_power / efficiency;
    float nominal_voltage = config_.battery_params.nominal_voltage.to<float>();
    float current_draw = nominal_voltage > 0.0f ? electrical_power / nominal_voltage : 0.0f;
    
    // Add a baseline current draw for electronics (e.g., 0.5A)
    current_draw += 0.5f;
    
    // Voltage Sag = Current * Internal Resistance
    float internal_resistance = config_.battery_params.internal_resistance.to<float>();
    float voltage_sag = current_draw * internal_resistance;
    
    state_.total_current_draw = units::current::ampere_t(current_draw);
    state_.battery_voltage = units::voltage::volt_t(nominal_voltage - voltage_sag);
    
    // Integrate capacity consumed (Amps * dt -> Amp-Seconds -> Amp-Hours)
    float consumed_ah = current_draw * (dt_sec / 3600.0f);
    state_.consumed_capacity += units::charge::ampere_hour_t(consumed_ah);
}
