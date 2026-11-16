#pragma once

#include "sim_types.h"
#include "vision_packet.h"
#include "flight_control_interface.h"
#include <vector>
#include <string>

#if defined(HAVE_OPENCV)
  #define SIM_HAS_OPENCV 1
#endif

#ifdef SIM_HAS_OPENCV
#include "gate_detector.h"
#include <opencv2/opencv.hpp>
#endif

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

    // Running vision simulation connection options
    bool hook_running_simulation{true};     // Hook into running vision_node over UDP if available
    std::string running_sim_ip{"127.0.0.1"};// Running vision simulation IP address
    int running_sim_rx_port{14555};         // UDP port where vision_node receives frames
    int running_sim_feedback_port{14556};   // UDP port where vision_node sends feedback
};

class SimVisionHarness {
public:
    explicit SimVisionHarness(const SimPerceptionConfig& config = SimPerceptionConfig());
    ~SimVisionHarness();

    void setConfig(const SimPerceptionConfig& config) { config_ = config; }
    const SimPerceptionConfig& getConfig() const { return config_; }

    // Generates a complete VisionPacket by rendering the camera view and running the actual vision system
    VisionPacket generateVisionPacket(
        const Pose& true_pose,
        const std::vector<SimulatedGoal>& goals,
        const std::vector<SimulatedTag>& tags,
        uint32_t sequence_id,
        double current_time);

#ifdef SIM_HAS_OPENCV
    // Renders the drone camera view (320x240 BGR) from the true drone pose and simulated goals
    cv::Mat renderFrame(
        const Pose& true_pose,
        const std::vector<SimulatedGoal>& goals) const;

    // Direct access to underlying GateDetector instance
    vision::GateDetector& getDetector() { return detector_; }
    const vision::GateDetector& getDetector() const { return detector_; }
#endif

private:
    SimPerceptionConfig config_;
#ifdef SIM_HAS_OPENCV
    vision::GateDetector detector_;
    mutable cv::Mat render_buffer_;
#endif

    // UDP IPC for hooking into running vision_node
    int udp_tx_sock_{-1};
    int udp_rx_sock_{-1};

    void initSockets();
    void closeSockets();
    bool tryReceiveRunningSimulationPacket(VisionPacket& packet);
#ifdef SIM_HAS_OPENCV
    void sendFrameToRunningSimulation(const cv::Mat& frame);
#endif
};

} // namespace sim
