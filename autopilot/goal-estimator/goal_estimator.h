#pragma once
#include <vector>
#include <mutex>
#include <map>
#include "vision_packet.h"
#include "telemetry_packet.h"
#include "flight_control_interface.h"
#include "tracked_goal.h"
#include "goal_track_filter.h"
#include "goal_registry.h"
#include "goal_telemetry_formatter.h"

class GoalEstimator {
public:
    GoalEstimator(double alpha, double timeout_sec);
    void updateEstimates(const VisionPacket& packet, double current_time);
    void updateEstimates(const std::vector<GoalMeasurement>& measurements, double current_time);
    std::vector<TrackedGoal> getCurrentEstimates(double current_time);
    std::vector<TrackedGoal> getAllKnownGoals();
    std::vector<GoalTrackTelemetry> getTelemetry(double current_time);
    bool getLastKnownEstimate(uint8_t shape_type, TrackedGoal& out_goal);
    void clearGoal(uint8_t shape_type);
    void reset();

    // Direct access to modular subcomponents
    GoalTrackFilter& getFilter() { return filter_; }
    const GoalTrackFilter& getFilter() const { return filter_; }
    GoalRegistry& getRegistry() { return registry_; }
    const GoalRegistry& getRegistry() const { return registry_; }

private:
    GoalTrackFilter filter_;
    GoalRegistry registry_;
    double timeout_sec_;
};
