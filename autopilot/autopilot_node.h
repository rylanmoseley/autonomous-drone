#pragma once
#include "mavlink_daemon.h"
#include "path_planner.h"
#include "goal_estimator.h"
#include "telemetry_logger_impl.h"
#include "../shared/include/ui_protocol.h"
#include <thread>
#include <atomic>
#include <mutex>
#include <sys/socket.h>
#include <netinet/in.h>

enum class MissionState { IDLE, TAKEOFF, NAVIGATING, RETURN_TO_HOME, LANDING, ERROR, ESTOP };

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

    MissionState state = MissionState::IDLE;
    UIGoalSequence seq = {};
    std::mutex seq_mutex;

    int ui_sock;
    int vision_sock;
    
    size_t current_goal_index = 0;
    bool new_goal = false;
    int goal_phase = 0;
    double approach_dx = 0.0;
    double approach_dy = 0.0;
    double approach_dz = 0.0;
    double transit_yaw = 0.0;
    
    std::chrono::steady_clock::time_point last_vision_time;

    AutopilotNode();
    ~AutopilotNode();

    void start();
    void stop();
    void run();
    void ui_loop();
    void vision_loop();
    
    // For testing
    MissionState getState() const { return state; }
    void setUISequence(const UIGoalSequence& s) { std::lock_guard<std::mutex> lock(seq_mutex); seq = s; }
    FlightCommand tick(const Pose& current_pose, double dt);
};
