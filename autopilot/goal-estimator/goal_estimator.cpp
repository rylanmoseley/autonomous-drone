#include "goal_estimator.h"

GoalEstimator::GoalEstimator(double alpha, double timeout_sec) 
    : filter_(alpha), registry_(timeout_sec), timeout_sec_(timeout_sec) {}

void GoalEstimator::updateEstimates(const VisionPacket& packet, double current_time) {
    std::vector<GoalMeasurement> measurements;
    for (uint8_t i = 0; i < packet.num_goals_reported && i < 3; ++i) {
        measurements.push_back(packet.goals[i]);
    }
    updateEstimates(measurements, current_time);
}

void GoalEstimator::updateEstimates(const std::vector<GoalMeasurement>& measurements, double current_time) {
    std::lock_guard<std::mutex> lock(registry_.getMutex());
    auto& tracked = registry_.getTrackedGoalsRef();
    auto& last_known = registry_.getLastKnownGoalsRef();

    for (const auto& gm : measurements) {
        if (!gm.is_detected || gm.num_hypotheses == 0) {
            continue;
        }

        const auto& hyp = gm.hypotheses[0];
        int key = gm.shape_type;
        auto it = tracked.find(key);
        if (it != tracked.end()) {
            bool timed_out = (current_time - it->second.last_update_time > timeout_sec_);
            bool accepted = filter_.updateGoalTrack(it->second, hyp, current_time, timed_out);
            if (accepted) {
                last_known[key] = it->second;
            }
        } else {
            TrackedGoal tg = GoalTrackFilter::initializeTrackedGoal(gm.shape_type, hyp, current_time);
            tracked[key] = tg;
            last_known[key] = tg;
        }
    }
}

bool GoalEstimator::getLastKnownEstimate(uint8_t shape_type, TrackedGoal& out_goal) {
    return registry_.getLastKnownGoal(shape_type, out_goal);
}

std::vector<TrackedGoal> GoalEstimator::getCurrentEstimates(double current_time) {
    return registry_.getActiveEstimates(current_time);
}

std::vector<TrackedGoal> GoalEstimator::getAllKnownGoals() {
    return registry_.getAllKnownGoals();
}

std::vector<GoalTrackTelemetry> GoalEstimator::getTelemetry(double current_time) {
    std::lock_guard<std::mutex> lock(registry_.getMutex());
    return GoalTelemetryFormatter::formatTelemetry(
        registry_.getTrackedGoalsRef(),
        registry_.getLastKnownGoalsRef(),
        current_time,
        timeout_sec_);
}

void GoalEstimator::clearGoal(uint8_t shape_type) {
    filter_.clearGoal(shape_type);
    registry_.clearGoal(shape_type);
}

void GoalEstimator::reset() {
    filter_.reset();
    registry_.reset();
}
