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
#include <thread>
#include <atomic>
#include <mutex>
#include <sys/socket.h>
#include <netinet/in.h>

class AutopilotNode {
public:
    MavlinkDaemon fcu;
    PathPlanner planner;
    GoalEstimator estimator;
    UdpTelemetryLogger logger;

    std::atomic<bool> running{false};
    std::thread loop_thread;
    std::thread ui_thread;
    std::thread vision_thread;

    FlightState state = FlightState::FLIGHT_STATE_STANDBY_DISARM;
    std::mutex state_mutex;

    // Mission sequence & config from UI
    uint8_t active_sequence_len = 0;
    uint8_t goal_sequence[MAX_GOAL_SEQUENCE_LEN] = {};
    uint8_t current_seq_index = 0;
    uint8_t configured_end_action = MISSION_END_HOVER;
    float hover_timeout_s = 0.0f;
    uint8_t use_tube_constraint = 0;
    float tube_rail_y = 0.0f;
    float tube_rail_z = 0.0f;
    uint8_t is_armed = 0;
    uint8_t estop_active = 0;
    uint32_t last_acked_cmd_id = 0;
    uint32_t status_seq_id = 0;
    uint32_t telem_seq_id = 0;
    std::mutex cmd_mutex;

    int ui_sock;
    int vision_sock;
    int telem_sock;
    struct sockaddr_in telem_dest;
    
    bool new_goal = false;
    int goal_phase = 0;
    double approach_dx = 0.0;
    double approach_dy = 0.0;
    double approach_dz = 0.0;
    double transit_yaw = 0.0;
    
    std::chrono::steady_clock::time_point last_vision_time;
    std::mutex vision_mutex;

    AutopilotNode();
    ~AutopilotNode();

    void start();
    void stop();
    void run();
    void ui_loop();
    void vision_loop();
    void handleUICommand(const UICommandPacket& pkt, const struct sockaddr_in& from, socklen_t from_len);
    
    // For testing
    FlightState getState() const { return state; }
    void setState(FlightState s) { state = s; }
    void handleCommand(const UICommandPacket& pkt) {
        struct sockaddr_in dummy = {};
        handleUICommand(pkt, dummy, sizeof(dummy));
    }
    GoalEstimator& getEstimator() { return estimator; }
    FlightCommand tick(const Pose& current_pose, double dt);
};
