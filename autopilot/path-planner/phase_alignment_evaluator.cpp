#include "phase_alignment_evaluator.h"
#include <cmath>
#include <algorithm>

bool PhaseAlignmentEvaluator::isStandoffReached(
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
        bool timeout_standoff = (standoff_ticks > 60 && (dist_to_standoff < 0.70 || dist_to_adj < 0.50) && lat_corr_err < 0.45);
        if (close_to_standoff || progressive_standoff || relaxed_standoff || timeout_standoff) {
            return true;
        }
    } else if (standoff_ticks > 80 && dot_fwd <= -0.20 && (dist_to_standoff < 0.80 || dist_to_adj < 0.60) && lat_corr_err < 0.50) {
        return true;
    } else if (standoff_ticks > 120 && lat_corr_err < 0.50) {
        return true;
    }
    return false;
}

bool PhaseAlignmentEvaluator::isBoresightAligned(
    const Pose& current_pose,
    const Pose& /*raw_target*/,
    const Pose& adjusted_target,
    double approach_dx,
    double approach_dy,
    int align_ticks,
    double approach_dz) const {

    double dyaw = std::atan2(std::sin(adjusted_target.yaw - current_pose.yaw),
                             std::cos(adjusted_target.yaw - current_pose.yaw));
    double delta_x = current_pose.x - adjusted_target.x;
    double delta_y = current_pose.y - adjusted_target.y;
    double delta_z = current_pose.z - adjusted_target.z;

    // Along-track position relative to entry_point (positive = inside the approach tunnel toward gate):
    double e_along = delta_x * approach_dx + delta_y * approach_dy + delta_z * approach_dz;

    // Perpendicular cross-track error vector from the 3D approach line:
    double e_perp_x = delta_x - e_along * approach_dx;
    double e_perp_y = delta_y - e_along * approach_dy;
    double e_perp_z = delta_z - e_along * approach_dz;
    double lat_err = std::sqrt(e_perp_x * e_perp_x + e_perp_y * e_perp_y + e_perp_z * e_perp_z);

    // Maximum allowable cross-track error is strictly bounded by 0.085m (aperture collision boundary).
    // Traversal must never be triggered if cross-track error exceeds 0.085m, regardless of tick count.
    bool tight_aligned = (align_ticks >= 6 && std::abs(dyaw) < 0.08 && lat_err < 0.045 && std::abs(e_along) < 0.20);
    bool relaxed_aligned = (align_ticks > 15 && std::abs(dyaw) < 0.12 && lat_err < 0.065 && std::abs(e_along) < 0.28);
    bool watchdog_aligned = (align_ticks > 30 && std::abs(dyaw) < 0.18 && lat_err < 0.085 && std::abs(e_along) < 0.35);
    bool timeout_aligned = (align_ticks > 70 && std::abs(dyaw) < 0.20 && lat_err < 0.085 && std::abs(e_along) < 0.40);

    return (tight_aligned || relaxed_aligned || watchdog_aligned || timeout_aligned);
}

bool PhaseAlignmentEvaluator::isTraversalCompleted(
    const Pose& current_pose,
    const Pose& raw_target,
    const Pose& traverse_target,
    double approach_dx,
    double approach_dy,
    int traverse_ticks,
    double arrival_tolerance) const {

    double dot_fwd = (current_pose.x - raw_target.x) * approach_dx +
                     (current_pose.y - raw_target.y) * approach_dy;

    double dx = traverse_target.x - current_pose.x;
    double dy = traverse_target.y - current_pose.y;
    double dz = traverse_target.z - current_pose.z;
    bool arrived = std::sqrt(dx*dx + dy*dy + dz*dz) <= arrival_tolerance;

    if ((traverse_ticks >= 6 && (dot_fwd >= 0.45 || arrived)) ||
        (traverse_ticks > 50 && dot_fwd >= 0.20) || traverse_ticks > 80) {
        return true;
    }
    return false;
}

Pose PhaseAlignmentEvaluator::planTraversalTarget(
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

Pose PhaseAlignmentEvaluator::getPunchThroughPose(
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
