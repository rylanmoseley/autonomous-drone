#pragma once

#include "follower_simulator.h"
#include "autopilot_node.h"
#include "collision_detector.h"
#include "sim_types.h"
#include "sim_camera_perception.h"
#include <vector>
#include <functional>
#include <string>

namespace sim {

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
    size_t collision_count{0};
    double min_clearance_m{1e9};
    std::vector<CollisionRecord> collisions;
    std::vector<uint8_t> sequence_traversed;
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
    SimCameraPerception& getPerception() { return perception_; }
    const SimCameraPerception& getPerception() const { return perception_; }
    const std::vector<TrajectoryPoint>& getTrajectory() const { return trajectory_; }
    const std::vector<SimulatedGoal>& getGoals() const { return simulated_goals_; }
    const std::vector<SimulatedTag>& getTags() const { return simulated_tags_; }
    const std::vector<CollisionRecord>& getCollisionRecords() const { return collision_records_; }
    double getMinClearance() const { return min_clearance_recorded_; }

    void setVisionEnabled(bool enabled) { vision_enabled_ = enabled; }
    void setDriftCorrectionEnabled(bool enabled) { drift_correction_enabled_ = enabled; }

    // Perception simulation config forwarding
    void setPerceptionConfig(const SimPerceptionConfig& cfg) { perception_.setConfig(cfg); }
    const SimPerceptionConfig& getPerceptionConfig() const { return perception_.getConfig(); }

private:
    AutopilotNode autopilot_;
    FcuFollowerSimulator follower_;
    SimCameraPerception perception_;
    std::vector<SimulatedGoal> simulated_goals_;
    std::vector<SimulatedTag> simulated_tags_;
    std::vector<TrajectoryPoint> trajectory_;
    std::vector<CollisionRecord> collision_records_;
    std::vector<uint8_t> sequence_traversed_;
    uint8_t last_observed_seq_index_{0};
    double min_clearance_recorded_{1e9};
    bool vision_enabled_{true};
    bool drift_correction_enabled_{true};
    uint32_t vision_seq_id_{0};

    void injectVisionTelemetry(double current_time);
};

} // namespace sim
