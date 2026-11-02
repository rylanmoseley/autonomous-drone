#include "follower_simulator.h"
#include <chrono>
#include <algorithm>

namespace sim {

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

void SimulationMetrics::recordSample(double tracking_error, double dt, bool arrived) {
    total_sim_time += dt;
    sample_count++;
    if (tracking_error > max_tracking_error) {
        max_tracking_error = tracking_error;
    }
    sum_squared_error += tracking_error * tracking_error;
    rmse_position = std::sqrt(sum_squared_error / static_cast<double>(sample_count));

    if (arrived && !arrived_at_target) {
        arrived_at_target = true;
        time_to_arrival = total_sim_time;
    }
}

void SimulationMetrics::reset() {
    total_sim_time = 0.0;
    total_distance_traveled = 0.0;
    max_tracking_error = 0.0;
    sum_squared_error = 0.0;
    rmse_position = 0.0;
    sample_count = 0;
    arrived_at_target = false;
    time_to_arrival = 0.0;
}

FcuFollowerSimulator::FcuFollowerSimulator(const FollowerConfig& config)
    : config_(config) {
    if (config_.random_seed != 0) {
        prng_.seed(config_.random_seed);
    } else {
        auto now = std::chrono::high_resolution_clock::now().time_since_epoch().count();
        prng_.seed(static_cast<uint32_t>(now));
    }
    reset();
}

void FcuFollowerSimulator::reset(const Pose& initial_pose) {
    true_pose_ = initial_pose;
    feedback_pose_ = initial_pose;
    vx_ = 0.0;
    vy_ = 0.0;
    vz_ = 0.0;
    bias_x_ = 0.0;
    bias_y_ = 0.0;
    bias_z_ = 0.0;
    metrics_.reset();
    if (config_.random_seed != 0) {
        prng_.seed(config_.random_seed);
    }
}

void FcuFollowerSimulator::setConfig(const FollowerConfig& config) {
    config_ = config;
    if (config_.random_seed != 0) {
        prng_.seed(config_.random_seed);
    }
}

double FcuFollowerSimulator::clamp(double value, double min_val, double max_val) {
    if (value < min_val) return min_val;
    if (value > max_val) return max_val;
    return value;
}

double FcuFollowerSimulator::wrapAnglePi(double angle_rad) {
    while (angle_rad > M_PI) angle_rad -= 2.0 * M_PI;
    while (angle_rad < -M_PI) angle_rad += 2.0 * M_PI;
    return angle_rad;
}

void FcuFollowerSimulator::step(const FlightCommand& cmd, double dt) {
    if (dt <= 0.0) return;

    true_pose_.timestamp += dt;

    if (cmd.estop) {
        // E-Stop: Cut motors immediately, drop to ground
        vx_ = 0.0;
        vy_ = 0.0;
        vz_ = -9.81 * dt; // Gravity drop
        true_pose_.z = std::max(0.0, true_pose_.z + vz_ * dt);
        feedback_pose_ = true_pose_;
        metrics_.recordSample(999.0, dt, false);
        return;
    }

    if (!cmd.enable) {
        // Disarmed: Remain stationary or at rest on ground
        vx_ = 0.0;
        vy_ = 0.0;
        vz_ = 0.0;
        if (true_pose_.z > 0.0) {
            true_pose_.z = std::max(0.0, true_pose_.z - 2.0 * dt);
        }
        feedback_pose_ = true_pose_;
        metrics_.recordSample(0.0, dt, false);
        return;
    }

    // 1. Follower Dynamic Response: First-order velocity command toward autopilot setpoint
    double tau_xy = std::max(0.01, config_.tau_xy);
    double tau_z  = std::max(0.01, config_.tau_z);
    double tau_yaw = std::max(0.01, config_.tau_yaw);

    double des_vx = (cmd.target_x - true_pose_.x) / tau_xy;
    double des_vy = (cmd.target_y - true_pose_.y) / tau_xy;
    double des_vz = (cmd.target_z - true_pose_.z) / tau_z;

    // 2. Velocity Saturation
    double horiz_speed = std::sqrt(des_vx * des_vx + des_vy * des_vy);
    if (horiz_speed > config_.max_velocity_xy && horiz_speed > 1e-6) {
        double scale = config_.max_velocity_xy / horiz_speed;
        des_vx *= scale;
        des_vy *= scale;
    }
    des_vz = clamp(des_vz, -config_.max_velocity_z, config_.max_velocity_z);

    // 3. Acceleration Limiting
    double max_dv = config_.max_acceleration * dt;
    vx_ += clamp(des_vx - vx_, -max_dv, max_dv);
    vy_ += clamp(des_vy - vy_, -max_dv, max_dv);
    vz_ += clamp(des_vz - vz_, -max_dv, max_dv);

    // 4. Position Integration (including external wind drift)
    double step_dx = (vx_ + config_.drift_vel_x) * dt;
    double step_dy = (vy_ + config_.drift_vel_y) * dt;
    double step_dz = (vz_ + config_.drift_vel_z) * dt;

    true_pose_.x += step_dx;
    true_pose_.y += step_dy;
    true_pose_.z = std::max(0.0, true_pose_.z + step_dz);

    metrics_.total_distance_traveled += std::sqrt(step_dx * step_dx + step_dy * step_dy + step_dz * step_dz);

    // 5. Yaw Orientation Integration
    double dyaw = wrapAnglePi(cmd.target_yaw - true_pose_.yaw);
    double yaw_rate = clamp(dyaw / tau_yaw, -config_.max_yaw_rate, config_.max_yaw_rate);
    true_pose_.yaw = wrapAnglePi(true_pose_.yaw + yaw_rate * dt);

    // Roll and pitch tilted kinematically with acceleration for realism
    true_pose_.pitch = clamp(vx_ / 9.81, -0.35, 0.35);
    true_pose_.roll  = clamp(-vy_ / 9.81, -0.35, 0.35);

    // 6. Imputed Noise Generation
    double noise_x = 0.0;
    double noise_y = 0.0;
    double noise_z = 0.0;
    double noise_yaw = 0.0;

    if (config_.noise_enabled) {
        noise_x = normal_dist_(prng_) * config_.pos_noise_std_xy;
        noise_y = normal_dist_(prng_) * config_.pos_noise_std_xy;
        noise_z = normal_dist_(prng_) * config_.pos_noise_std_z;
        noise_yaw = normal_dist_(prng_) * config_.yaw_noise_std;

        if (config_.random_walk_drift_std > 0.0) {
            double rw_step = config_.random_walk_drift_std * std::sqrt(dt);
            bias_x_ += normal_dist_(prng_) * rw_step;
            bias_y_ += normal_dist_(prng_) * rw_step;
            bias_z_ += normal_dist_(prng_) * rw_step;
        }
    }

    feedback_pose_.x = true_pose_.x + noise_x + bias_x_;
    feedback_pose_.y = true_pose_.y + noise_y + bias_y_;
    feedback_pose_.z = std::max(0.0, true_pose_.z + noise_z + bias_z_);
    feedback_pose_.roll = true_pose_.roll;
    feedback_pose_.pitch = true_pose_.pitch;
    feedback_pose_.yaw = wrapAnglePi(true_pose_.yaw + noise_yaw);
    feedback_pose_.timestamp = true_pose_.timestamp;
    feedback_pose_.confidence = 1.0;

    // 7. Tracking Error & Metrics
    double err_x = cmd.target_x - true_pose_.x;
    double err_y = cmd.target_y - true_pose_.y;
    double err_z = cmd.target_z - true_pose_.z;
    double tracking_error = std::sqrt(err_x * err_x + err_y * err_y + err_z * err_z);

    bool arrived = (tracking_error <= 0.20);
    metrics_.recordSample(tracking_error, dt, arrived);
}

Telemetry FcuFollowerSimulator::getTelemetry(int battery_level, bool error_state) const {
    Telemetry telem{};
    telem.current_pose = feedback_pose_;
    telem.battery_level = battery_level;
    telem.error_state = error_state;
    return telem;
}

} // namespace sim
