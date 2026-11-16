#include "kinematic_profile_filter.h"
#include <cmath>
#include <algorithm>

KinematicProfileFilter::KinematicProfileFilter(const PlannerConfig& config)
    : config_(config) {}

double KinematicProfileFilter::clamp(double value, double min_val, double max_val) const {
    if (value < min_val) return min_val;
    if (value > max_val) return max_val;
    return value;
}

bool KinematicProfileFilter::hasArrived(const Pose& current_pose, const Pose& target_pose) const {
    PlannerConfig cfg = getConfig();
    double dx = target_pose.x - current_pose.x;
    double dy = target_pose.y - current_pose.y;
    double dz = target_pose.z - current_pose.z;
    double distance = std::sqrt(dx*dx + dy*dy + dz*dz);
    return distance <= cfg.arrival_tolerance;
}

FlightCommand KinematicProfileFilter::planStep(
    const Pose& current_pose,
    const Pose& target_pose,
    double dt,
    double apf_vx,
    double apf_vy) {

    PlannerConfig cfg = getConfig();
    FlightCommand cmd;
    cmd.enable = true;
    cmd.estop = false;

    double dx = target_pose.x - current_pose.x;
    double dy = target_pose.y - current_pose.y;
    double dz = target_pose.z - current_pose.z;

    double vx = dx * cfg.position_p_gain + apf_vx;
    double vy = dy * cfg.position_p_gain + apf_vy;
    double vz = dz * cfg.position_p_gain;

    double v_mag_horiz = std::sqrt(vx*vx + vy*vy);
    if (v_mag_horiz > cfg.max_velocity && v_mag_horiz > 1e-6) {
        vx = (vx / v_mag_horiz) * cfg.max_velocity;
        vy = (vy / v_mag_horiz) * cfg.max_velocity;
    }

    if (vz > cfg.max_vertical_velocity) vz = cfg.max_vertical_velocity;
    if (vz < -cfg.max_vertical_velocity) vz = -cfg.max_vertical_velocity;

    // Apply acceleration limits
    double max_dv = cfg.max_acceleration * dt;
    vx_ += clamp(vx - vx_, -max_dv, max_dv);
    vy_ += clamp(vy - vy_, -max_dv, max_dv);
    vz_ += clamp(vz - vz_, -max_dv, max_dv);

    cmd.target_x = current_pose.x + vx_ * dt;
    cmd.target_y = current_pose.y + vy_ * dt;
    cmd.target_z = current_pose.z + vz_ * dt;

    double dyaw = std::atan2(std::sin(target_pose.yaw - current_pose.yaw),
                             std::cos(target_pose.yaw - current_pose.yaw));

    double yaw_rate = dyaw * cfg.yaw_p_gain;
    yaw_rate = clamp(yaw_rate, -cfg.max_yaw_rate, cfg.max_yaw_rate);
    cmd.target_yaw = current_pose.yaw + yaw_rate * dt;

    return cmd;
}

PlannerConfig KinematicProfileFilter::getConfig() const {
    std::lock_guard<std::mutex> lock(cfg_mutex_);
    return config_;
}

void KinematicProfileFilter::updateConfig(const PlannerConfig& new_cfg) {
    std::lock_guard<std::mutex> lock(cfg_mutex_);
    config_ = new_cfg;
}

void KinematicProfileFilter::reset() {
    vx_ = 0.0;
    vy_ = 0.0;
    vz_ = 0.0;
}
