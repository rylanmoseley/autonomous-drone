#include "orthogonal_path_generator.h"
#include <cmath>
#include <algorithm>

OrthogonalPathGenerator::OrthogonalPath OrthogonalPathGenerator::generateOrthogonalPath(
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
    bool end1_blocked = avoidance_.checkSegmentExclusionIntersection(reference_pose, end1, obstacles);
    bool end2_blocked = avoidance_.checkSegmentExclusionIntersection(reference_pose, end2, obstacles);
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
    } else if (preferred_sign < 0.0) {
        choose_end1 = false;
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

Pose OrthogonalPathGenerator::planOrthogonalTraversalTarget(
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

bool OrthogonalPathGenerator::isOrthogonalTraversalCompleted(
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
