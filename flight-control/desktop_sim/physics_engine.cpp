#include "physics_engine.h"
#include <iostream>

PhysicsEngine::PhysicsEngine(const Config::DroneHardwareConfig& initial_config) 
    : config_(initial_config) 
{
    current_motor_rpms_.resize(config_.rotors.size(), units::angular_velocity::revolutions_per_minute_t(0));
    target_motor_torques_.resize(config_.rotors.size(), units::torque::newton_meter_t(0));
    
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
    target_motor_torques_.resize(config_.rotors.size(), units::torque::newton_meter_t(0));
}

Config::DroneHardwareConfig PhysicsEngine::get_config() {
    std::lock_guard<std::mutex> lock(state_mutex_);
    return config_;
}

void PhysicsEngine::set_motor_output(int motor_index, units::torque::newton_meter_t torque, units::angular_velocity::revolutions_per_minute_t rpm) {
    std::lock_guard<std::mutex> lock(state_mutex_);
    if (motor_index >= 0 && motor_index < target_motor_torques_.size()) {
        target_motor_torques_[motor_index] = torque;
        // In a real simulation, we would either drive RPM directly or integrate torque.
        // If the ESC is in torque control mode, we use torque.
    }
}

PhysicsEngine::TrueState PhysicsEngine::get_true_state() {
    std::lock_guard<std::mutex> lock(state_mutex_);
    return state_;
}

void PhysicsEngine::calculate_forces_and_torques(
    Eigen::Matrix<units::force::newton_t, 3, 1>& out_force, 
    Eigen::Matrix<units::torque::newton_meter_t, 3, 1>& out_torque) 
{
    out_force.setZero();
    out_torque.setZero();
    
    // 1. Gravity (Z is down in NED, so gravity is positive Z)
    out_force(2) = units::force::newton_t((config_.mass * units::acceleration::meters_per_second_squared_t(9.81)).to<float>());
    
    // 2. Aerodynamic Drag (Simplified linear drag model)
    out_force(0) -= units::force::newton_t(config_.linear_drag_coefficient * state_.velocity(0).to<float>());
    out_force(1) -= units::force::newton_t(config_.linear_drag_coefficient * state_.velocity(1).to<float>());
    out_force(2) -= units::force::newton_t(config_.linear_drag_coefficient * state_.velocity(2).to<float>());
    
    // 3. Rotor forces and torques
    for (size_t i = 0; i < config_.rotors.size(); ++i) {
        auto rpm = current_motor_rpms_[i].to<float>();
        // Convert RPM to rad/s for calculations
        float omega = rpm * (2.0f * M_PI / 60.0f);
        
        // Thrust = k_f * w^2
        float thrust_mag = config_.rotors[i].thrust_coefficient * (omega * omega);
        Eigen::Vector3f thrust_vec = config_.rotors[i].thrust_axis * thrust_mag;
        
        // Add to total force
        out_force(0) += units::force::newton_t(thrust_vec(0));
        out_force(1) += units::force::newton_t(thrust_vec(1));
        out_force(2) += units::force::newton_t(thrust_vec(2));
        
        // Structural Torque = r x F
        Eigen::Vector3f position_vec(
            config_.rotors[i].position(0).to<float>(),
            config_.rotors[i].position(1).to<float>(),
            config_.rotors[i].position(2).to<float>()
        );
        Eigen::Vector3f structural_torque = position_vec.cross(thrust_vec);
        
        // Drag Torque (Yaw authority) = k_m * w^2
        float drag_torque_mag = config_.rotors[i].torque_coefficient * (omega * omega);
        // Reaction torque direction: if rotor spins CW, frame feels CCW torque
        float torque_direction = config_.rotors[i].spins_clockwise ? -1.0f : 1.0f;
        
        // Assuming rotors are mounted perfectly vertically for drag torque (aligned with Z)
        Eigen::Vector3f drag_torque(0.0f, 0.0f, drag_torque_mag * torque_direction);
        
        out_torque(0) += units::torque::newton_meter_t(structural_torque(0) + drag_torque(0));
        out_torque(1) += units::torque::newton_meter_t(structural_torque(1) + drag_torque(1));
        out_torque(2) += units::torque::newton_meter_t(structural_torque(2) + drag_torque(2));
    }
}

void PhysicsEngine::step(units::time::second_t dt) {
    std::lock_guard<std::mutex> lock(state_mutex_);
    
    float dt_sec = dt.to<float>();
    
    // 1. Update motor dynamics (first order lag from target to actual)
    float mechanical_power = 0.0f;
    for (size_t i = 0; i < config_.rotors.size(); ++i) {
        // If the ESC receives a torque command, we would calculate RPM based on load.
        // For a simplified simulator, assume we are directly commanding target RPM via the ESC,
        // and the ESC has a first-order response.
        // We'll treat `target_motor_torques_` as a generic command for now and map it to an RPM.
        // A more advanced sim would simulate the BLDC stator physics.
        // For now, let's assume `target_motor_torques_` maps linearly to a target RPM for demonstration.
        // Ideally, we'd add `target_motor_rpms_` to the PhysicsEngine interface.
        float target_rpm = target_motor_torques_[i].to<float>() * 1000.0f; // Hacky mapping for now
        
        // Clamp to max RPM
        if (target_rpm > config_.motor_params.max_rpm.to<float>()) {
            target_rpm = config_.motor_params.max_rpm.to<float>();
        } else if (target_rpm < 0.0f) {
            target_rpm = 0.0f;
        }
        
        // First order low pass filter
        float alpha = dt_sec / (config_.motor_params.time_constant.to<float>() + dt_sec);
        float current = current_motor_rpms_[i].to<float>();
        float next_rpm = current + alpha * (target_rpm - current);
        
        current_motor_rpms_[i] = units::angular_velocity::revolutions_per_minute_t(next_rpm);
        
        // Power = Torque * Omega (assuming torque_coefficient * w^2 is the load torque)
        float omega = next_rpm * (2.0f * M_PI / 60.0f);
        float load_torque = config_.rotors[i].torque_coefficient * (omega * omega);
        mechanical_power += (load_torque * omega);
    }
    
    // 2. Calculate sum of forces and torques
    Eigen::Matrix<units::force::newton_t, 3, 1> total_force;
    Eigen::Matrix<units::torque::newton_meter_t, 3, 1> total_torque;
    calculate_forces_and_torques(total_force, total_torque);
    
    // 3. Integrate Rigid Body Dynamics
    // Linear
    Eigen::Vector3f a(
        total_force(0).to<float>() / config_.mass.to<float>(),
        total_force(1).to<float>() / config_.mass.to<float>(),
        total_force(2).to<float>() / config_.mass.to<float>()
    );
    
    state_.acceleration(0) = units::acceleration::meters_per_second_squared_t(a(0));
    state_.acceleration(1) = units::acceleration::meters_per_second_squared_t(a(1));
    state_.acceleration(2) = units::acceleration::meters_per_second_squared_t(a(2));
    
    state_.velocity += state_.acceleration * dt_sec;
    state_.position += state_.velocity * dt_sec;
    
    // Angular: I * omega_dot + omega x (I * omega) = tau
    Eigen::Vector3f tau(total_torque(0).to<float>(), total_torque(1).to<float>(), total_torque(2).to<float>());
    Eigen::Vector3f omega(
        state_.angular_velocity(0).to<float>(),
        state_.angular_velocity(1).to<float>(),
        state_.angular_velocity(2).to<float>()
    );
    
    // omega_dot = I^-1 * (tau - omega x (I * omega))
    Eigen::Vector3f angular_accel = config_.moment_of_inertia.inverse() * (tau - omega.cross(config_.moment_of_inertia * omega));
    
    state_.angular_acceleration(0) = units::angular_acceleration::radians_per_second_squared_t(angular_accel(0));
    state_.angular_acceleration(1) = units::angular_acceleration::radians_per_second_squared_t(angular_accel(1));
    state_.angular_acceleration(2) = units::angular_acceleration::radians_per_second_squared_t(angular_accel(2));
    
    state_.angular_velocity += state_.angular_acceleration * dt_sec;
    
    // Euler angle integration (simple accumulation for small angles, 
    // ideally should use quaternion derivative for large rotations)
    state_.euler_angles += state_.angular_velocity * dt_sec;
    
    // 4. Update Battery Simulation
    // Assume electrical power = mechanical power / efficiency (e.g. 80%)
    float efficiency = 0.8f;
    float electrical_power = mechanical_power / efficiency;
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
