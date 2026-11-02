#include "goal_estimator.h"
#include <cmath>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

GoalEstimator::GoalEstimator(double alpha, double timeout_sec) 
    : alpha_(alpha), timeout_sec_(timeout_sec) {}

void GoalEstimator::updateEstimates(const VisionPacket& packet, double current_time) {
    std::vector<GoalMeasurement> measurements;
    for (uint8_t i = 0; i < packet.num_goals_reported && i < 3; ++i) {
        measurements.push_back(packet.goals[i]);
    }
    updateEstimates(measurements, current_time);
}

void GoalEstimator::updateEstimates(const std::vector<GoalMeasurement>& measurements, double current_time) {
    std::lock_guard<std::mutex> lock(mutex_);
    for (const auto& gm : measurements) {
        if (!gm.is_detected || gm.num_hypotheses == 0) {
            continue;
        }

        const auto& hyp = gm.hypotheses[0];
        Vec3f pos = hyp.position_cam;
        Quatf q = hyp.orientation_cam;
        double yaw = std::atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z));

        int key = gm.shape_type;
        auto it = tracked_goals_.find(key);
        if (it != tracked_goals_.end()) {
            it->second.pose.x = alpha_ * pos.x + (1.0 - alpha_) * it->second.pose.x;
            it->second.pose.y = alpha_ * pos.y + (1.0 - alpha_) * it->second.pose.y;
            it->second.pose.z = alpha_ * pos.z + (1.0 - alpha_) * it->second.pose.z;

            double dyaw = yaw - it->second.pose.yaw;
            while (dyaw > M_PI) dyaw -= 2.0 * M_PI;
            while (dyaw < -M_PI) dyaw += 2.0 * M_PI;
            it->second.pose.yaw += alpha_ * dyaw;
            while (it->second.pose.yaw > M_PI) it->second.pose.yaw -= 2.0 * M_PI;
            while (it->second.pose.yaw < -M_PI) it->second.pose.yaw += 2.0 * M_PI;

            it->second.pose.confidence = hyp.confidence;
            it->second.last_update_time = current_time;
            it->second.determinate = true;
            it->second.state = TRACK_STATE_LOCKED;
        } else {
            TrackedGoal tg;
            tg.shape_type = gm.shape_type;
            tg.goal_id = gm.shape_type;
            tg.state = TRACK_STATE_CONVERGING;
            tg.pose.x = pos.x;
            tg.pose.y = pos.y;
            tg.pose.z = pos.z;
            tg.pose.yaw = yaw;
            tg.pose.confidence = hyp.confidence;
            tg.determinate = true;
            tg.last_update_time = current_time;
            tracked_goals_[key] = tg;
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

std::vector<GoalTrackTelemetry> GoalEstimator::getTelemetry(double current_time) {
    std::lock_guard<std::mutex> lock(mutex_);
    std::vector<GoalTrackTelemetry> result;
    uint8_t shapes[3] = {SHAPE_CIRCLE, SHAPE_TRIANGLE, SHAPE_SQUARE};
    for (int i = 0; i < 3; ++i) {
        GoalTrackTelemetry gtt = {};
        gtt.shape_type = shapes[i];
        auto it = tracked_goals_.find(shapes[i]);
        if (it != tracked_goals_.end() && (current_time - it->second.last_update_time <= timeout_sec_)) {
            gtt.state = it->second.state;
            gtt.world_position = {(float)it->second.pose.x, (float)it->second.pose.y, (float)it->second.pose.z};
            gtt.spatial_variance = it->second.spatial_variance;
        } else {
            gtt.state = TRACK_STATE_SEARCHING;
        }
        result.push_back(gtt);
    }
    return result;
}
