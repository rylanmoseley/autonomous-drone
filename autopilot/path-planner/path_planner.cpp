#include "path_planner.h"
#include <cmath>
#include <algorithm>

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

    double dyaw = std::atan2(std::sin(target_pose.yaw - current_pose.yaw),
                             std::cos(target_pose.yaw - current_pose.yaw));
    
    double yaw_rate = dyaw * cfg.yaw_p_gain;
    yaw_rate = clamp(yaw_rate, -cfg.max_yaw_rate, cfg.max_yaw_rate);
    cmd.target_yaw = current_pose.yaw + yaw_rate * dt;

    return cmd;
}

PathPlanner::ApproachDirection PathPlanner::planApproachDirection(
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

void PathPlanner::alignApproachWithCorridor(
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

PathPlanner::StandoffPlan PathPlanner::planStandoffTarget(
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

Pose PathPlanner::planTraversalTarget(
    const Pose& current_pose,
    const Pose& raw_target,
    double approach_dx,
    double approach_dy,
    double lookahead,
    double max_punch) const {

    double dot_fwd = (current_pose.x - raw_target.x) * approach_dx +
                     (current_pose.y - raw_target.y) * approach_dy;
    double step_fwd = std::min(max_punch, std::max(-0.25, dot_fwd + lookahead));

    Pose target;
    target.x = raw_target.x + approach_dx * step_fwd;
    target.y = raw_target.y + approach_dy * step_fwd;
    target.z = raw_target.z;
    target.yaw = std::atan2(approach_dy, approach_dx);
    return target;
}

Pose PathPlanner::getPunchThroughPose(
    const Pose& raw_target,
    double approach_dx,
    double approach_dy,
    double punch_dist) const {

    Pose target;
    target.x = raw_target.x + approach_dx * punch_dist;
    target.y = raw_target.y + approach_dy * punch_dist;
    target.z = raw_target.z;
    target.yaw = std::atan2(approach_dy, approach_dx);
    return target;
}

bool PathPlanner::checkSegmentExclusionIntersection(
    const Pose& p1,
    const Pose& p2,
    const std::vector<Obstacle>& obstacles,
    double exclusion_radius) const {

    double seg_dx = p2.x - p1.x;
    double seg_dy = p2.y - p1.y;
    double seg_dz = p2.z - p1.z;
    double seg_len_sq = seg_dx * seg_dx + seg_dy * seg_dy;

    for (const auto& obs : obstacles) {
        double d_p2 = std::hypot(p2.x - obs.x, p2.y - obs.y);
        if (p2.z <= obs.z + 0.35 && d_p2 < exclusion_radius) {
            return true;
        }

        if (seg_len_sq > 1e-4) {
            double t = ((obs.x - p1.x) * seg_dx + (obs.y - p1.y) * seg_dy) / seg_len_sq;
            if (t > 0.05 && t <= 1.0) {
                double proj_x = p1.x + t * seg_dx;
                double proj_y = p1.y + t * seg_dy;
                double proj_z = p1.z + t * seg_dz;
                if (proj_z <= obs.z + 0.35) {
                    double dist_to_seg = std::hypot(proj_x - obs.x, proj_y - obs.y);
                    if (dist_to_seg < exclusion_radius) {
                        return true;
                    }
                }
            }
        }
    }
    return false;
}

PathPlanner::OrthogonalPath PathPlanner::generateOrthogonalPath(
    const Pose& goal_pose,
    const Pose& reference_pose,
    const std::vector<Obstacle>& obstacles,
    double standoff_dist,
    double preferred_sign) const {

    // 1. Calculate 3D unit normal from goal 3D orientation (yaw, pitch, roll)
    double cy = std::cos(goal_pose.yaw);
    double sy = std::sin(goal_pose.yaw);
    double cp = std::cos(goal_pose.pitch);
    double sp = std::sin(goal_pose.pitch);

    double nx = cy * cp;
    double ny = sy * cp;
    double nz = -sp;
    double n_len = std::sqrt(nx*nx + ny*ny + nz*nz);
    if (n_len > 1e-4) {
        nx /= n_len;
        ny /= n_len;
        nz /= n_len;
    } else {
        nx = 1.0; ny = 0.0; nz = 0.0;
    }

    // 2. The two ends of the line segment orthogonal to the goal, passing through center
    Pose end1;
    end1.x = goal_pose.x - nx * standoff_dist;
    end1.y = goal_pose.y - ny * standoff_dist;
    end1.z = goal_pose.z - nz * standoff_dist;
    end1.yaw = std::atan2(ny, nx);

    Pose end2;
    end2.x = goal_pose.x + nx * standoff_dist;
    end2.y = goal_pose.y + ny * standoff_dist;
    end2.z = goal_pose.z + nz * standoff_dist;
    end2.yaw = std::atan2(-ny, -nx);

    // 3. Distance from reference pose (drone current pose or last goal exit) to each end
    double dist1 = std::hypot(reference_pose.x - end1.x, reference_pose.y - end1.y, reference_pose.z - end1.z);
    double dist2 = std::hypot(reference_pose.x - end2.x, reference_pose.y - end2.y, reference_pose.z - end2.z);

    // 4. Check exclusion area intersections for paths to and through each candidate end
    bool end1_blocked = checkSegmentExclusionIntersection(reference_pose, end1, obstacles);
    bool end2_blocked = checkSegmentExclusionIntersection(reference_pose, end2, obstacles);
    for (const auto& obs : obstacles) {
        if (std::hypot(end1.x - obs.x, end1.y - obs.y) < 0.55 && end1.z <= obs.z + 0.35) {
            end1_blocked = true;
        }
        if (std::hypot(end2.x - obs.x, end2.y - obs.y) < 0.55 && end2.z <= obs.z + 0.35) {
            end2_blocked = true;
        }
    }

    bool choose_end1 = true;
    if (preferred_sign > 0.0) {
        choose_end1 = true;
        if (end1_blocked && !end2_blocked) {
            choose_end1 = false;
        }
    } else if (preferred_sign < 0.0) {
        choose_end1 = false;
        if (end2_blocked && !end1_blocked) {
            choose_end1 = true;
        }
    } else {
        if (dist1 <= dist2) {
            // end1 is closer: choose end1 unless it intersects an exclusion area and end2 is clear
            if (end1_blocked && !end2_blocked) {
                choose_end1 = false;
            } else {
                choose_end1 = true;
            }
        } else {
            // end2 is closer: choose end2 unless it intersects an exclusion area and end1 is clear
            if (end2_blocked && !end1_blocked) {
                choose_end1 = true;
            } else {
                choose_end1 = false;
            }
        }
    }

    OrthogonalPath path;
    path.gate_center = goal_pose;
    path.standoff_dist = standoff_dist;
    path.total_length = 2.0 * standoff_dist;

    if (choose_end1) {
        path.entry_point = end1;
        path.exit_point = end2;
        path.normal_x = nx;
        path.normal_y = ny;
        path.normal_z = nz;
        path.entry_point.yaw = std::atan2(ny, nx);
        path.exit_point.yaw = std::atan2(ny, nx);
        path.traversal_sign = 1.0;
    } else {
        path.entry_point = end2;
        path.exit_point = end1;
        path.normal_x = -nx;
        path.normal_y = -ny;
        path.normal_z = -nz;
        path.entry_point.yaw = std::atan2(-ny, -nx);
        path.exit_point.yaw = std::atan2(-ny, -nx);
        path.traversal_sign = -1.0;
    }

    return path;
}

Pose PathPlanner::planOrthogonalTraversalTarget(
    const Pose& current_pose,
    const OrthogonalPath& path,
    double lookahead_dist) const {

    // Project drone position onto the orthogonal line from entry_point along normal:
    double dx = current_pose.x - path.entry_point.x;
    double dy = current_pose.y - path.entry_point.y;
    double dz = current_pose.z - path.entry_point.z;
    double s = dx * path.normal_x + dy * path.normal_y + dz * path.normal_z;

    // Advance lookahead along the line:
    double s_target = std::min(path.total_length, std::max(0.0, s + lookahead_dist));

    // Target point strictly on the orthogonal line passing through gate center:
    Pose target;
    target.x = path.entry_point.x + path.normal_x * s_target;
    target.y = path.entry_point.y + path.normal_y * s_target;
    target.z = path.entry_point.z + path.normal_z * s_target;
    target.yaw = std::atan2(path.normal_y, path.normal_x);
    return target;
}

bool PathPlanner::isOrthogonalTraversalCompleted(
    const Pose& current_pose,
    const OrthogonalPath& path,
    int traverse_ticks) const {

    double dx = current_pose.x - path.entry_point.x;
    double dy = current_pose.y - path.entry_point.y;
    double dz = current_pose.z - path.entry_point.z;
    double s = dx * path.normal_x + dy * path.normal_y + dz * path.normal_z;

    double dist_to_exit = std::hypot(current_pose.x - path.exit_point.x,
                                     current_pose.y - path.exit_point.y,
                                     current_pose.z - path.exit_point.z);

    // Completed when drone reaches or exceeds the exit standoff (s >= total_length - 0.10)
    // or arrives within arrival_tolerance of exit point
    if ((traverse_ticks >= 6 && (s >= path.total_length - 0.10 || dist_to_exit < 0.20)) ||
        (traverse_ticks > 50 && s >= path.standoff_dist + 0.20) ||
        traverse_ticks > 80) {
        return true;
    }
    return false;
}

void PathPlanner::computeObstacleRepulsion(
    const Pose& current_pose,
    const std::vector<Obstacle>& obstacles,
    double des_vx,
    double des_vy,
    double& out_apf_vx,
    double& out_apf_vy,
    double& out_offset_x,
    double& out_offset_y,
    double r_nogo,
    double r_influence) const {

    out_apf_vx = 0.0;
    out_apf_vy = 0.0;
    out_offset_x = 0.0;
    out_offset_y = 0.0;

    for (const auto& obs : obstacles) {
        if (current_pose.z > obs.z + 0.40) continue;

        double dx = current_pose.x - obs.x;
        double dy = current_pose.y - obs.y;
        double dist_2d = std::hypot(dx, dy);

        if (dist_2d >= r_influence) continue;

        if (dist_2d < 0.01) {
            dx = 1.0; dy = 0.0; dist_2d = 1.0;
        }

        double pen = (r_influence - dist_2d) / (r_influence - r_nogo);
        pen = std::max(0.0, std::min(2.5, pen));

        double ux = dx / dist_2d;
        double uy = dy / dist_2d;

        double cross = dx * des_vy - dy * des_vx;
        double side = (std::abs(cross) > 1e-3) ? ((cross >= 0.0) ? 1.0 : -1.0) : ((dy >= 0.0) ? 1.0 : -1.0);

        double vx = -side * uy;
        double vy = side * ux;

        double dir_x = ux * 0.70 + vx * 0.70;
        double dir_y = uy * 0.70 + vy * 0.70;
        double dir_norm = std::hypot(dir_x, dir_y);
        if (dir_norm > 1e-4) {
            dir_x /= dir_norm;
            dir_y /= dir_norm;
        }

        out_apf_vx += dir_x * pen * 2.5;
        out_apf_vy += dir_y * pen * 2.5;
        out_offset_x += dir_x * pen * 0.45;
        out_offset_y += dir_y * pen * 0.45;
    }

    double tot_off = std::hypot(out_offset_x, out_offset_y);
    if (tot_off > 1.2) {
        out_offset_x = (out_offset_x / tot_off) * 1.2;
        out_offset_y = (out_offset_y / tot_off) * 1.2;
    }
}

bool PathPlanner::isStandoffReached(
    const Pose& current_pose,
    const Pose& entrance_standoff,
    const Pose& adjusted_target,
    double approach_dx,
    double approach_dy,
    double dot_fwd,
    int standoff_ticks) const {

    double d_standoff_x = current_pose.x - entrance_standoff.x;
    double d_standoff_y = current_pose.y - entrance_standoff.y;
    double d_standoff_z = current_pose.z - entrance_standoff.z;
    double dist_to_standoff = std::hypot(d_standoff_x, d_standoff_y, d_standoff_z);
    double lat_corr_err = std::hypot(-d_standoff_x * approach_dy + d_standoff_y * approach_dx, d_standoff_z);

    double d_adj_x = current_pose.x - adjusted_target.x;
    double d_adj_y = current_pose.y - adjusted_target.y;
    double d_adj_z = current_pose.z - adjusted_target.z;
    double dist_to_adj = std::hypot(d_adj_x, d_adj_y, d_adj_z);

    if (dot_fwd <= -0.35) {
        bool close_to_standoff = (dist_to_standoff < 0.25 && lat_corr_err < 0.15);
        bool progressive_standoff = (standoff_ticks > 25 && (dist_to_standoff < 0.40 || dist_to_adj < 0.25) && lat_corr_err < 0.25);
        bool relaxed_standoff = (standoff_ticks > 40 && (dist_to_standoff < 0.55 || dist_to_adj < 0.40) && lat_corr_err < 0.35);
        bool timeout_standoff = (standoff_ticks > 60 && (dist_to_standoff < 0.70 || dist_to_adj < 0.50));
        if (close_to_standoff || progressive_standoff || relaxed_standoff || timeout_standoff) {
            return true;
        }
    } else if (standoff_ticks > 80 && dot_fwd <= -0.20 && (dist_to_standoff < 0.80 || dist_to_adj < 0.60)) {
        return true;
    } else if (standoff_ticks > 100) {
        return true;
    }
    return false;
}

bool PathPlanner::isBoresightAligned(
    const Pose& current_pose,
    const Pose& raw_target,
    const Pose& adjusted_target,
    double approach_dx,
    double approach_dy,
    int align_ticks) const {

    double dyaw = std::atan2(std::sin(adjusted_target.yaw - current_pose.yaw),
                             std::cos(adjusted_target.yaw - current_pose.yaw));
    double d_gate_x = current_pose.x - raw_target.x;
    double d_gate_y = current_pose.y - raw_target.y;
    double e_perp = -d_gate_x * approach_dy + d_gate_y * approach_dx;
    double e_z = current_pose.z - raw_target.z;
    double lat_err = std::hypot(e_perp, e_z);
    double delta_x = current_pose.x - adjusted_target.x;
    double delta_y = current_pose.y - adjusted_target.y;
    double e_along = delta_x * approach_dx + delta_y * approach_dy;

    bool tight_aligned = (align_ticks >= 6 && std::abs(dyaw) < 0.08 && lat_err < 0.045 && std::abs(e_along) < 0.25);
    bool relaxed_aligned = (align_ticks > 15 && std::abs(dyaw) < 0.12 && lat_err < 0.075);
    bool watchdog_aligned = (align_ticks > 30 && std::abs(dyaw) < 0.18 && lat_err < 0.110);
    bool forced_watchdog = (align_ticks > 45 && std::abs(dyaw) < 0.28 && lat_err < 0.160);
    bool timeout_aligned = (align_ticks > 60);

    return (tight_aligned || relaxed_aligned || watchdog_aligned || forced_watchdog || timeout_aligned);
}

bool PathPlanner::isTraversalCompleted(
    const Pose& current_pose,
    const Pose& raw_target,
    const Pose& traverse_target,
    double approach_dx,
    double approach_dy,
    int traverse_ticks) const {

    double dot_fwd = (current_pose.x - raw_target.x) * approach_dx +
                     (current_pose.y - raw_target.y) * approach_dy;
    if ((traverse_ticks >= 6 && (dot_fwd >= 0.45 || has_arrived(current_pose, traverse_target))) ||
        (traverse_ticks > 50 && dot_fwd >= 0.20) || traverse_ticks > 80) {
        return true;
    }
    return false;
}
