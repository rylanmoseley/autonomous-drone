#pragma once
#include <vector>
#include <mutex>
#include <map>
#include "vision_packet.h"
#include "telemetry_packet.h"
#include "flight_control_interface.h"

struct TrackedGoal {
    uint8_t shape_type = SHAPE_NONE;
    int goal_id = 0;
    uint8_t state = TRACK_STATE_SEARCHING;
    Pose pose = {};
    float spatial_variance = 0.0f;
    double last_update_time = 0.0;
    bool determinate = false;
};

class GoalEstimator {
public:
    GoalEstimator(double alpha, double timeout_sec);
    void updateEstimates(const VisionPacket& packet, double current_time);
    void updateEstimates(const std::vector<GoalMeasurement>& measurements, double current_time);
    std::vector<TrackedGoal> getCurrentEstimates(double current_time);
    std::vector<GoalTrackTelemetry> getTelemetry(double current_time);

private:
    std::map<int, TrackedGoal> tracked_goals_;
    std::mutex mutex_;
    double alpha_;
    double timeout_sec_;
};
