#pragma once
#include <vector>
#include <mutex>
#include <map>
#include "vision_packet.h"

struct TrackedGoal {
    VisionPose pose;
    int goal_id;
    bool determinate;
    double last_update_time;
};

class GoalEstimator {
public:
    GoalEstimator(double alpha, double timeout_sec);
    void updateEstimates(const std::vector<VisionGoalEstimate>& new_estimates, double current_time);
    std::vector<TrackedGoal> getCurrentEstimates(double current_time);

private:
    std::map<int, TrackedGoal> tracked_goals_;
    std::mutex mutex_;
    double alpha_;
    double timeout_sec_;
};
