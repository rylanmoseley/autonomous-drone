#include "goal_telemetry_formatter.h"

std::vector<GoalTrackTelemetry> GoalTelemetryFormatter::formatTelemetry(
    const std::map<int, TrackedGoal>& tracked_goals,
    const std::map<int, TrackedGoal>& last_known_goals,
    double current_time,
    double timeout_sec)
{
    std::vector<GoalTrackTelemetry> result;
    uint8_t shapes[3] = {SHAPE_CIRCLE, SHAPE_TRIANGLE, SHAPE_SQUARE};
    for (int i = 0; i < 3; ++i) {
        GoalTrackTelemetry gtt = {};
        gtt.shape_type = shapes[i];
        auto it = tracked_goals.find(shapes[i]);
        if (it != tracked_goals.end() && (current_time - it->second.last_update_time <= timeout_sec)) {
            gtt.state = it->second.state;
            gtt.world_position = {(float)it->second.pose.x, (float)it->second.pose.y, (float)it->second.pose.z};
            gtt.spatial_variance = it->second.spatial_variance;
        } else {
            auto lk = last_known_goals.find(shapes[i]);
            if (lk != last_known_goals.end()) {
                gtt.state = TRACK_STATE_OCCLUDED;
                gtt.world_position = {(float)lk->second.pose.x, (float)lk->second.pose.y, (float)lk->second.pose.z};
                gtt.spatial_variance = 0.05f;
            } else {
                gtt.state = TRACK_STATE_SEARCHING;
            }
        }
        result.push_back(gtt);
    }
    return result;
}
