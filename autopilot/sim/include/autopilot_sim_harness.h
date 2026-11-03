#pragma once

#include "follower_simulator.h"
#include "autopilot_node.h"
#include <vector>
#include <functional>

namespace sim {

struct SimulatedGoal {
    uint8_t shape_type; // SHAPE_CIRCLE, SHAPE_TRIANGLE, SHAPE_SQUARE
    double x;           // Position in World NED
    double y;
    double z;
    double yaw;         // Heading (radians)
};

struct SimulatedTag {
    uint16_t tag_id;
    double x;           // World NED coordinates of tag anchor
    double y;
    double z;
    double roll{0.0};
    double pitch{0.0};
    double yaw{0.0};
    float confidence{1.0f};
};

struct TrajectoryPoint {
    double timestamp;
    Pose true_pose;
    Pose feedback_pose;
    FlightCommand command;
    FlightState flight_state;
};

struct MissionReport {
    bool mission_completed{false};
    double duration_s{0.0};
    size_t total_steps{0};
    FlightState final_state{FlightState::FLIGHT_STATE_STANDBY_DISARM};
    Pose final_true_pose{};
    double rmse_tracking{0.0};
    double max_tracking_error{0.0};
    double total_distance_m{0.0};
    size_t tag_readings_forwarded{0};
    double total_drift_corrected_m{0.0};
    std::string summary;
};

class AutopilotSimHarness {
public:
    AutopilotSimHarness(const FollowerConfig& follower_cfg = FollowerConfig());

    // Configure mission parameters
    void setupMission(const std::vector<uint8_t>& shape_sequence,
                      uint8_t end_action = MISSION_END_HOVER,
                      bool use_tube = false,
                      float tube_rail_y = 0.0f,
                      float tube_rail_z = 1.0f);

    // Place ground-truth physical gate goals in the simulated world
    void addSimulatedGoal(uint8_t shape_type, double x, double y, double z, double yaw = 0.0);
    void clearSimulatedGoals();

    // Place ground-truth fiducial tag anchors in the simulated world for global drift correction
    void addSimulatedTag(uint16_t tag_id, double x, double y, double z, double roll = 0.0, double pitch = 0.0, double yaw = 0.0, float confidence = 1.0f);
    void clearSimulatedTags();

    // Reset the simulation to start location
    void reset(const Pose& initial_pose = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0});

    // Advance simulation by dt seconds
    void step(double dt = 0.05);

    // Run simulation loop until termination condition or timeout
    MissionReport runUntil(double max_duration_s = 60.0,
                           double dt = 0.05,
                           std::function<bool(const AutopilotSimHarness&)> termination_check = nullptr);

    // Accessors
    AutopilotNode& getAutopilot() { return autopilot_; }
    const AutopilotNode& getAutopilot() const { return autopilot_; }
    FcuFollowerSimulator& getFollower() { return follower_; }
    const FcuFollowerSimulator& getFollower() const { return follower_; }
    const std::vector<TrajectoryPoint>& getTrajectory() const { return trajectory_; }
    const std::vector<SimulatedGoal>& getGoals() const { return simulated_goals_; }
    const std::vector<SimulatedTag>& getTags() const { return simulated_tags_; }

    void setVisionEnabled(bool enabled) { vision_enabled_ = enabled; }
    void setDriftCorrectionEnabled(bool enabled) { drift_correction_enabled_ = enabled; }

private:
    AutopilotNode autopilot_;
    FcuFollowerSimulator follower_;
    std::vector<SimulatedGoal> simulated_goals_;
    std::vector<SimulatedTag> simulated_tags_;
    std::vector<TrajectoryPoint> trajectory_;
    bool vision_enabled_{true};
    bool drift_correction_enabled_{true};
    uint32_t vision_seq_id_{0};

    void injectVisionTelemetry(double current_time);
};

} // namespace sim
