#include "approach_corridor_planner.h"
#include <cmath>
#include <algorithm>

ApproachCorridorPlanner::ApproachDirection ApproachCorridorPlanner::planApproachDirection(
    const Pose& target_gate,
    const Pose& last_completed_gate,
    double last_approach_dx,
    double last_approach_dy,
    bool is_subsequent_goal) const {

    double hoop_nx = std::cos(target_gate.yaw);
    double hoop_ny = std::sin(target_gate.yaw);
    double a_norm = std::hypot(hoop_nx, hoop_ny);
    if (a_norm > 1e-4) {
        hoop_nx /= a_norm;
        hoop_ny /= a_norm;
    } else {
        hoop_nx = 1.0;
        hoop_ny = 0.0;
    }

    double approach_sign = 1.0;

    if (is_subsequent_goal) {
        double seq_dx = target_gate.x - last_completed_gate.x;
        double seq_dy = target_gate.y - last_completed_gate.y;
        double prev_dir_x = last_approach_dx;
        double prev_dir_y = last_approach_dy;
        double p_norm = std::hypot(prev_dir_x, prev_dir_y);
        if (p_norm > 1e-4) { prev_dir_x /= p_norm; prev_dir_y /= p_norm; }
        else { prev_dir_x = 1.0; prev_dir_y = 0.0; }

        double delta_along = std::abs(seq_dx * prev_dir_x + seq_dy * prev_dir_y);
        double delta_perp = std::abs(-seq_dx * prev_dir_y + seq_dy * prev_dir_x);

        // Side-by-side goals: significant lateral offset (>= 0.35m) and limited longitudinal offset (<= 1.2m)
        if (delta_perp >= 0.35 && delta_along <= 1.20) {
            // Reverse traversal direction relative to previous gate
            double dot_prev = hoop_nx * prev_dir_x + hoop_ny * prev_dir_y;
            approach_sign = (dot_prev >= 0.0) ? -1.0 : 1.0;
        } else {
            // Sequential forward gates: maintain same traversal direction as previous gate
            double dot_prev = hoop_nx * prev_dir_x + hoop_ny * prev_dir_y;
            approach_sign = (dot_prev >= 0.0) ? 1.0 : -1.0;
        }
    }

    ApproachDirection dir;
    dir.dx = hoop_nx * approach_sign;
    dir.dy = hoop_ny * approach_sign;
    dir.dz = 0.0;
    dir.sign = approach_sign;
    return dir;
}

void ApproachCorridorPlanner::alignApproachWithCorridor(
    const Pose& target_gate,
    double& inout_dx,
    double& inout_dy) const {

    double hoop_nx = std::cos(target_gate.yaw);
    double hoop_ny = std::sin(target_gate.yaw);
    double a_norm = std::hypot(hoop_nx, hoop_ny);
    if (a_norm > 1e-4) {
        hoop_nx /= a_norm;
        hoop_ny /= a_norm;
    } else {
        hoop_nx = inout_dx;
        hoop_ny = inout_dy;
    }
    // Planar gate normal has +-180 deg symmetry; select half-plane aligned with approach vector
    if (hoop_nx * inout_dx + hoop_ny * inout_dy < 0.0) {
        hoop_nx = -hoop_nx;
        hoop_ny = -hoop_ny;
    }
    inout_dx = hoop_nx;
    inout_dy = hoop_ny;
}

ApproachCorridorPlanner::StandoffPlan ApproachCorridorPlanner::planStandoffTarget(
    const Pose& current_pose,
    const Pose& raw_target,
    double approach_dx,
    double approach_dy,
    double standoff_dist,
    double lateral_clearance) const {

    StandoffPlan result;
    result.entrance_standoff.x = raw_target.x - approach_dx * standoff_dist;
    result.entrance_standoff.y = raw_target.y - approach_dy * standoff_dist;
    result.entrance_standoff.z = raw_target.z;
    result.entrance_standoff.yaw = std::atan2(approach_dy, approach_dx);

    result.target_pose = result.entrance_standoff;

    double perp_dx = -approach_dy;
    double perp_dy = approach_dx;
    result.lat = (current_pose.x - raw_target.x) * perp_dx +
                 (current_pose.y - raw_target.y) * perp_dy;
    result.dot_fwd = (current_pose.x - raw_target.x) * approach_dx +
                     (current_pose.y - raw_target.y) * approach_dy;

    if (result.dot_fwd > -0.20) {
        double denom = result.dot_fwd + standoff_dist;
        double v_cross = (denom > 1e-3) ? (standoff_dist * result.lat / denom) : result.lat;
        if (std::abs(v_cross) < 0.65) {
            double side_sign = (result.lat >= 0.0) ? 1.0 : -1.0;
            result.target_pose.x = raw_target.x + perp_dx * (side_sign * lateral_clearance) - approach_dx * standoff_dist;
            result.target_pose.y = raw_target.y + perp_dy * (side_sign * lateral_clearance) - approach_dy * standoff_dist;
            result.is_avoidance_active = true;
        }
    }

    return result;
}
