#pragma once
#include "mavlink_daemon.h"
#include "path_planner.h"
#include "goal_estimator.h"
#include "telemetry_logger_impl.h"
#include "flight_control_interface.h"
#include "ui_protocol.h"
#include "vision_packet.h"
#include "telemetry_packet.h"
#include "common_types.h"
#include "mission_state_machine.h"
#include "mission_sequencer.h"
#include "perception_coordinator.h"
#include "safety_supervisor.h"
#include "telemetry_bridge.h"
#include <thread>
#include <atomic>
#include <mutex>

class AutopilotNode {
public:
    MavlinkDaemon fcu;
    PathPlanner planner;
    GoalEstimator estimator;
    UdpTelemetryLogger logger;

    MissionStateMachine state_machine;
    MissionSequencer sequencer;
    PerceptionCoordinator perception;
    SafetySupervisor safety;
    TelemetryBridge telemetry_bridge;

    // References to sequencer fields for seamless backward compatibility
    uint8_t& active_sequence_len;
    uint8_t* goal_sequence;
    uint8_t& current_seq_index;
    uint8_t& configured_end_action;
    float& hover_timeout_s;
    uint8_t& use_tube_constraint;
    float& tube_rail_y;
    float& tube_rail_z;

    bool& new_goal;
    int& goal_phase;
    int& align_ticks;
    int& standoff_ticks;
    int& seek_ticks;
    int& traverse_ticks;
    double& approach_dx;
    double& approach_dy;
    double& approach_dz;
    double& approach_sign_;
    double& last_approach_dx_;
    double& last_approach_dy_;
    double transit_yaw{0.0};
    Pose& active_target_pose;
    bool& has_active_target;
    PathPlanner::OrthogonalPath& active_orthogonal_path_;
    bool& has_orthogonal_path_;
    Pose& last_completed_goal_pose_;
    Pose& hover_pose_;
    bool& has_hover_pose_;

    // References to perception fields
    std::atomic<uint32_t>& tags_received_count;
    std::atomic<uint32_t>& tags_forwarded_count;
    std::atomic<uint32_t>& surveyed_tags_verified_count_;
    TagMeasurement& last_tag_measurement;

    // Reference to safety watchdog field
    std::chrono::steady_clock::time_point& last_vision_time;

    // Flight state & UI status
    std::atomic<FlightState> state{FlightState::FLIGHT_STATE_STANDBY_DISARM};
    std::atomic<uint8_t> is_armed{0};
    std::atomic<uint8_t> estop_active{0};
    uint32_t last_acked_cmd_id{0};
    uint32_t status_seq_id{0};
    uint32_t telem_seq_id{0};

    // Sockets & destination
    int ui_sock{-1};
    int vision_sock{-1};
    int telem_sock{-1};
    struct sockaddr_in telem_dest{};

    std::atomic<bool> running{false};
    std::thread loop_thread;
    std::thread ui_thread;
    std::thread vision_thread;

    std::mutex cmd_mutex;
    mutable std::mutex pose_mutex_;
    std::mutex state_mutex;
    std::mutex vision_mutex;

    Pose last_current_pose_{0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0};

    // State machine API
    FlightState getState() const {
        return state_machine.getState();
    }
    void setState(FlightState s) {
        state_machine.setState(s);
        state = s;
    }

    // Safety watchdog API
    void setVisionTimeout(double timeout_s) { safety.setVisionTimeout(timeout_s); }
    double getVisionTimeout() const { return safety.getVisionTimeout(); }
    void enableFcuWatchdog(bool enable, double timeout_s = 1.0) {
        safety.enableFcuWatchdog(enable, timeout_s);
    }

    // Perception surveyed tag API
    void registerSurveyedTag(uint16_t tag_id, const Vec3f& world_pos, const Quatf& world_att, float size_m = 0.16f) {
        perception.registerSurveyedTag(tag_id, world_pos, world_att, size_m);
    }
    bool getSurveyedTag(uint16_t tag_id, SurveyedTagPose& out_tag) const {
        return perception.getSurveyedTag(tag_id, out_tag);
    }
    bool hasSurveyedTag(uint16_t tag_id) const {
        return perception.hasSurveyedTag(tag_id);
    }
    void clearSurveyedTags() {
        perception.clearSurveyedTags();
    }

    void updateTelemetry(const Pose& pose) {
        std::lock_guard<std::mutex> lock(pose_mutex_);
        last_current_pose_ = pose;
    }

    Pose getCurrentPose() const {
        std::lock_guard<std::mutex> lock(pose_mutex_);
        return last_current_pose_;
    }

    GoalEstimator& getEstimator() { return estimator; }

    // Subcomponent accessors
    MissionStateMachine& getStateMachine() { return state_machine; }
    const MissionStateMachine& getStateMachine() const { return state_machine; }
    MissionSequencer& getSequencer() { return sequencer; }
    const MissionSequencer& getSequencer() const { return sequencer; }
    PerceptionCoordinator& getPerceptionCoordinator() { return perception; }
    const PerceptionCoordinator& getPerceptionCoordinator() const { return perception; }
    SafetySupervisor& getSafetySupervisor() { return safety; }
    const SafetySupervisor& getSafetySupervisor() const { return safety; }
    TelemetryBridge& getTelemetryBridge() { return telemetry_bridge; }
    const TelemetryBridge& getTelemetryBridge() const { return telemetry_bridge; }

    AutopilotNode();
    ~AutopilotNode();

    void start();
    void stop();
    void run();
    void ui_loop();
    void vision_loop();
    void handleVisionPacket(const VisionPacket& pkt, double current_time);
    void handleUICommand(const UICommandPacket& pkt, const struct sockaddr_in& from, socklen_t from_len);
    void handleCommand(const UICommandPacket& pkt) {
        struct sockaddr_in dummy = {};
        handleUICommand(pkt, dummy, sizeof(dummy));
    }
    FlightCommand tick(const Pose& current_pose, double dt);
    void computeNonTargetRepulsion(
        const Pose& current_pose,
        uint8_t target_shape,
        double des_vx,
        double des_vy,
        double& out_apf_vx,
        double& out_apf_vy,
        double& out_offset_x,
        double& out_offset_y);
    void updateApproachDirection(const Pose& target_pose);
};
