#pragma once
#include <vector>
#include <map>
#include "telemetry_packet.h"
#include "tracked_goal.h"

class GoalTelemetryFormatter {
public:
    static std::vector<GoalTrackTelemetry> formatTelemetry(
        const std::map<int, TrackedGoal>& tracked_goals,
        const std::map<int, TrackedGoal>& last_known_goals,
        double current_time,
        double timeout_sec);
};
