#pragma once

#include "sim_types.h"
#include "vision_packet.h"
#include "flight_control_interface.h"
#include <vector>

namespace sim {

struct SimPerceptionConfig {
    bool fov_filtering_enabled{false};      // If true, enforce camera frustum checks
    double h_fov_deg{70.0};                 // Horizontal field of view (+-35 deg)
    double v_fov_deg{50.0};                 // Vertical field of view (+-25 deg)
    double min_range_m{0.15};               // Minimum detection distance
    double max_range_m{6.0};                // Maximum detection distance
    bool occlusion_check_enabled{false};    // Check gate-to-gate line-of-sight occlusion
    bool proximity_dropout_enabled{false};  // Camera edge clipping at close range
    double proximity_dropout_dist{0.35};    // Proximity dropout distance (m)
};

class SimCameraPerception {
public:
    explicit SimCameraPerception(const SimPerceptionConfig& config = SimPerceptionConfig());

    void setConfig(const SimPerceptionConfig& config) { config_ = config; }
    const SimPerceptionConfig& getConfig() const { return config_; }

    // Generates a complete VisionPacket from true drone pose, simulated goals, and simulated tags
    VisionPacket generateVisionPacket(
        const Pose& true_pose,
        const std::vector<SimulatedGoal>& goals,
        const std::vector<SimulatedTag>& tags,
        uint32_t sequence_id,
        double current_time) const;

private:
    SimPerceptionConfig config_;
};

} // namespace sim
