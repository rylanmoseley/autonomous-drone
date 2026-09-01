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
    
    // Gravity
    out_force(2) = units::force::newton_t((config_.mass * units::acceleration::meters_per_second_squared_t(9.81)).to<float>());
    
    // TODO: Aerodynamics (Drag based on state_.velocity)
    
    // Rotor forces
    for (size_t i = 0; i < config_.rotors.size(); ++i) {
        auto rpm = current_motor_rpms_[i].to<float>();
        
        // Thrust = k_f * w^2
        float thrust_mag = config_.rotors[i].thrust_coefficient * (rpm * rpm);
        Eigen::Vector3f thrust_vec = config_.rotors[i].thrust_axis * thrust_mag;
        
        // Add to total force
        out_force(0) += units::force::newton_t(thrust_vec(0));
        out_force(1) += units::force::newton_t(thrust_vec(1));
        out_force(2) += units::force::newton_t(thrust_vec(2));
        
        // Torque = r x F + Drag Torque
        // TODO: Cross product of position and thrust vector for structural torque
        // TODO: Add aerodynamic drag torque of the propeller (yaw authority)
    }
}

void PhysicsEngine::step(units::time::second_t dt) {
    std::lock_guard<std::mutex> lock(state_mutex_);
    
    // 1. Update motor dynamics (first order lag from target torque/rpm to actual)
    for (size_t i = 0; i < config_.rotors.size(); ++i) {
        // TODO: Implement motor spin-up inertia.
        // For now, instant response (if torque control is perfect):
        // (Just a placeholder, actual RPM would be derived from torque and load)
    }
    
    // 2. Calculate sum of forces and torques
    Eigen::Matrix<units::force::newton_t, 3, 1> total_force;
    Eigen::Matrix<units::torque::newton_meter_t, 3, 1> total_torque;
    calculate_forces_and_torques(total_force, total_torque);
    
    // 3. Integrate Rigid Body Dynamics (Euler integration for simplicity, RK4 preferred later)
    // a = F / m
    state_.acceleration(0) = units::acceleration::meters_per_second_squared_t(total_force(0).to<float>() / config_.mass.to<float>());
    state_.acceleration(1) = units::acceleration::meters_per_second_squared_t(total_force(1).to<float>() / config_.mass.to<float>());
    state_.acceleration(2) = units::acceleration::meters_per_second_squared_t(total_force(2).to<float>() / config_.mass.to<float>());
    
    // v = v + a*dt
    state_.velocity += state_.acceleration * dt.to<float>();
    
    // p = p + v*dt
    state_.position += state_.velocity * dt.to<float>();
    
    // TODO: Integrate angular equations of motion using inertia tensor
    // dw = I^-1 * (Torque - w x (I * w)) dt
    
    // 4. Update Battery Simulation
    // TODO: Calculate power drawn by motors -> current -> voltage sag
}
