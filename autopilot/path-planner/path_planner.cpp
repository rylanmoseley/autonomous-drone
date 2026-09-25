#include "path_planner.h"
#include <cmath>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

PathPlanner::PathPlanner(const PlannerConfig& config) : config_(config) {}

double PathPlanner::clamp(double value, double min_val, double max_val) const {
    if (value < min_val) return min_val;
    if (value > max_val) return max_val;
    return value;
}

bool PathPlanner::has_arrived(const Pose& current_pose, const Pose& target_pose) const {
    double dx = target_pose.x - current_pose.x;
    double dy = target_pose.y - current_pose.y;
    double dz = target_pose.z - current_pose.z;
    double distance = std::sqrt(dx*dx + dy*dy + dz*dz);
    return distance <= config_.arrival_tolerance;
}

FlightCommand PathPlanner::plan(const Pose& current_pose, const Pose& target_pose, double dt) {
    FlightCommand cmd;
    cmd.enable = true;
    cmd.estop = false;

    double dx = target_pose.x - current_pose.x;
    double dy = target_pose.y - current_pose.y;
    double dz = target_pose.z - current_pose.z;

    double dist = std::sqrt(dx*dx + dy*dy + dz*dz);
    
    if (dist <= config_.arrival_tolerance) {
        // We have arrived, but do not snap the coordinates to avoid teleporting the setpoint!
        // We just let the proportional controller naturally bring vx to 0.
    }

    double vx = dx * config_.position_p_gain;
    double vy = dy * config_.position_p_gain;
    double vz = dz * config_.position_p_gain;

    double v_mag_horiz = std::sqrt(vx*vx + vy*vy);
    if (v_mag_horiz > config_.max_velocity) {
        vx = (vx / v_mag_horiz) * config_.max_velocity;
        vy = (vy / v_mag_horiz) * config_.max_velocity;
    }

    if (vz > config_.max_vertical_velocity) vz = config_.max_vertical_velocity;
    if (vz < -config_.max_vertical_velocity) vz = -config_.max_vertical_velocity;

    cmd.target_x = current_pose.x + vx * dt;
    cmd.target_y = current_pose.y + vy * dt;
    cmd.target_z = current_pose.z + vz * dt;

    double dyaw = target_pose.yaw - current_pose.yaw;
    while (dyaw > M_PI) dyaw -= 2.0 * M_PI;
    while (dyaw < -M_PI) dyaw += 2.0 * M_PI;
    
    double yaw_rate = dyaw * config_.yaw_p_gain;
    yaw_rate = clamp(yaw_rate, -config_.max_yaw_rate, config_.max_yaw_rate);
    cmd.target_yaw = current_pose.yaw + yaw_rate * dt;

    return cmd;
}
