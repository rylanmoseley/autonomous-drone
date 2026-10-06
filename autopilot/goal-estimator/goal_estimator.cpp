#include "goal_estimator.h"
#include <cmath>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif


GoalEstimator::GoalEstimator(double alpha, double timeout_sec) 
    : alpha_(alpha), timeout_sec_(timeout_sec) {}

void GoalEstimator::updateEstimates(const std::vector<VisionGoalEstimate>& new_estimates, double current_time) {
    std::lock_guard<std::mutex> lock(mutex_);
    for (const auto& est : new_estimates) {
        auto it = tracked_goals_.find(est.goal_id);
        if (it != tracked_goals_.end()) {
            it->second.pose.x = alpha_ * est.pose.x + (1.0 - alpha_) * it->second.pose.x;
            it->second.pose.y = alpha_ * est.pose.y + (1.0 - alpha_) * it->second.pose.y;
            it->second.pose.z = alpha_ * est.pose.z + (1.0 - alpha_) * it->second.pose.z;
            double dyaw = est.pose.yaw - it->second.pose.yaw;
            while (dyaw > M_PI) dyaw -= 2.0 * M_PI;
            while (dyaw < -M_PI) dyaw += 2.0 * M_PI;
            it->second.pose.yaw = it->second.pose.yaw + alpha_ * dyaw;
            
            while (it->second.pose.yaw > M_PI) it->second.pose.yaw -= 2.0 * M_PI;
            while (it->second.pose.yaw < -M_PI) it->second.pose.yaw += 2.0 * M_PI;
            it->second.last_update_time = current_time;
            it->second.determinate = est.determinate;
        } else {
            TrackedGoal tg;
            tg.pose = est.pose;
            tg.goal_id = est.goal_id;
            tg.determinate = est.determinate;
            tg.last_update_time = current_time;
            tracked_goals_[est.goal_id] = tg;
        }
    }
}

std::vector<TrackedGoal> GoalEstimator::getCurrentEstimates(double current_time) {
    std::lock_guard<std::mutex> lock(mutex_);
    std::vector<TrackedGoal> result;
    auto it = tracked_goals_.begin();
    while (it != tracked_goals_.end()) {
        if (current_time - it->second.last_update_time > timeout_sec_) {
            it = tracked_goals_.erase(it);
        } else {
            result.push_back(it->second);
            ++it;
        }
    }
    return result;
}
