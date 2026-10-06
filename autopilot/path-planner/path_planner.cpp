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
    PlannerConfig cfg = getConfig();
    double dx = target_pose.x - current_pose.x;
    double dy = target_pose.y - current_pose.y;
    double dz = target_pose.z - current_pose.z;
    double distance = std::sqrt(dx*dx + dy*dy + dz*dz);
    return distance <= cfg.arrival_tolerance;
}

FlightCommand PathPlanner::plan(const Pose& current_pose, const Pose& target_pose, double dt, double apf_vx, double apf_vy) {
    PlannerConfig cfg = getConfig();
    FlightCommand cmd;
    cmd.enable = true;
    cmd.estop = false;

    double dx = target_pose.x - current_pose.x;
    double dy = target_pose.y - current_pose.y;
    double dz = target_pose.z - current_pose.z;

    double dist = std::sqrt(dx*dx + dy*dy + dz*dz);
    
    if (dist <= cfg.arrival_tolerance) {
        // We have arrived, but do not snap the coordinates to avoid teleporting the setpoint!
        // We just let the proportional controller naturally bring vx to 0.
    }

    double vx = dx * cfg.position_p_gain + apf_vx;
    double vy = dy * cfg.position_p_gain + apf_vy;
    double vz = dz * cfg.position_p_gain;

    double v_mag_horiz = std::sqrt(vx*vx + vy*vy);
    if (v_mag_horiz > cfg.max_velocity) {
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

    double dyaw = target_pose.yaw - current_pose.yaw;
    while (dyaw > M_PI) dyaw -= 2.0 * M_PI;
    while (dyaw < -M_PI) dyaw += 2.0 * M_PI;
    
    double yaw_rate = dyaw * cfg.yaw_p_gain;
    yaw_rate = clamp(yaw_rate, -cfg.max_yaw_rate, cfg.max_yaw_rate);
    cmd.target_yaw = current_pose.yaw + yaw_rate * dt;

    return cmd;
}
