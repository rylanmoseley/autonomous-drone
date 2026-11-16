#include "goal_track_filter.h"
#include <cmath>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

GoalTrackFilter::GoalTrackFilter(double alpha)
    : alpha_(alpha) {}

void GoalTrackFilter::reset() {
    outlier_jump_counts_.clear();
}

void GoalTrackFilter::clearGoal(int key) {
    outlier_jump_counts_.erase(key);
}

int GoalTrackFilter::getOutlierJumpCount(int key) const {
    auto it = outlier_jump_counts_.find(key);
    if (it != outlier_jump_counts_.end()) {
        return it->second;
    }
    return 0;
}

double GoalTrackFilter::extractYaw(const Quatf& q) {
    return std::atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z));
}

TrackedGoal GoalTrackFilter::initializeTrackedGoal(int shape_type, const PoseHypothesis& hyp, double current_time) {
    TrackedGoal tg;
    tg.shape_type = static_cast<uint8_t>(shape_type);
    tg.goal_id = shape_type;
    tg.state = TRACK_STATE_CONVERGING;
    tg.pose.x = hyp.position_cam.x;
    tg.pose.y = hyp.position_cam.y;
    tg.pose.z = hyp.position_cam.z;
    tg.pose.yaw = extractYaw(hyp.orientation_cam);
    tg.pose.confidence = hyp.confidence;
    tg.determinate = true;
    tg.last_update_time = current_time;
    return tg;
}

bool GoalTrackFilter::updateGoalTrack(TrackedGoal& goal, const PoseHypothesis& hyp, double current_time, bool timed_out) {
    Vec3f pos = hyp.position_cam;
    double yaw = extractYaw(hyp.orientation_cam);
    int key = goal.shape_type;

    if (timed_out) {
        goal.pose.x = pos.x;
        goal.pose.y = pos.y;
        goal.pose.z = pos.z;
        goal.pose.yaw = yaw;
    } else {
        // Spatial outlier rejection: if position jumps by > 0.35m from converged track, discard single-frame glitch.
        // However, if >= 3 consecutive frames consistently report the new location, the goal has legitimately moved!
        double dx = pos.x - goal.pose.x;
        double dy = pos.y - goal.pose.y;
        double dz = pos.z - goal.pose.z;
        double jump = std::sqrt(dx * dx + dy * dy + dz * dz);
        if (jump > 0.35 && goal.state == TRACK_STATE_LOCKED) {
            outlier_jump_counts_[key]++;
            if (outlier_jump_counts_[key] < 3) {
                return false;
            }
            // 3 consecutive frames confirm the goal was moved: adopt new position immediately!
            goal.pose.x = pos.x;
            goal.pose.y = pos.y;
            goal.pose.z = pos.z;
            goal.pose.yaw = yaw;
            outlier_jump_counts_[key] = 0;
        } else {
            outlier_jump_counts_[key] = 0;
            double eff_alpha = alpha_;
            if (goal.state == TRACK_STATE_LOCKED && hyp.confidence < 0.85f) {
                eff_alpha = alpha_ * 0.25; // Damp updates from marginal or edge-clipped contours
            }

            goal.pose.x = eff_alpha * pos.x + (1.0 - eff_alpha) * goal.pose.x;
            goal.pose.y = eff_alpha * pos.y + (1.0 - eff_alpha) * goal.pose.y;
            goal.pose.z = eff_alpha * pos.z + (1.0 - eff_alpha) * goal.pose.z;

            double dyaw = std::atan2(std::sin(yaw - goal.pose.yaw), std::cos(yaw - goal.pose.yaw));
            if (goal.state == TRACK_STATE_LOCKED && std::abs(dyaw) > M_PI / 2.0) {
                // Project to the same normal axis (+-180 deg equivalence for planar gates)
                if (dyaw > 0.0) {
                    dyaw -= M_PI;
                } else {
                    dyaw += M_PI;
                }
            }
            goal.pose.yaw = std::atan2(std::sin(goal.pose.yaw + eff_alpha * dyaw),
                                       std::cos(goal.pose.yaw + eff_alpha * dyaw));
        }
    }

    goal.pose.confidence = hyp.confidence;
    goal.last_update_time = current_time;
    goal.determinate = true;
    goal.state = TRACK_STATE_LOCKED;
    return true;
}
