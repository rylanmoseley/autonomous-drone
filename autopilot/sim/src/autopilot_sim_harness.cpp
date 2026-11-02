#include "autopilot_sim_harness.h"
#include <sstream>
#include <iomanip>

namespace sim {

AutopilotSimHarness::AutopilotSimHarness(const FollowerConfig& follower_cfg)
    : follower_(follower_cfg) {
    reset();
}

void AutopilotSimHarness::setupMission(const std::vector<uint8_t>& shape_sequence,
                                      uint8_t end_action,
                                      bool use_tube,
                                      float tube_rail_y,
                                      float tube_rail_z) {
    UICommandPacket cmd_pkt{};
    cmd_pkt.msg_type = MSG_TYPE_UI_CMD;
    cmd_pkt.command_id = 1;
    cmd_pkt.command = UI_CMD_START_MISSION;
    cmd_pkt.sequence_length = static_cast<uint8_t>(std::min(shape_sequence.size(), static_cast<size_t>(MAX_GOAL_SEQUENCE_LEN)));
    for (size_t i = 0; i < cmd_pkt.sequence_length; ++i) {
        cmd_pkt.goal_sequence[i] = shape_sequence[i];
    }
    cmd_pkt.end_action = end_action;
    cmd_pkt.use_tube_constraint = use_tube ? 1 : 0;
    cmd_pkt.tube_rail_y = tube_rail_y;
    cmd_pkt.tube_rail_z = tube_rail_z;

    autopilot_.handleCommand(cmd_pkt);
}

void AutopilotSimHarness::addSimulatedGoal(uint8_t shape_type, double x, double y, double z, double yaw) {
    simulated_goals_.push_back({shape_type, x, y, z, yaw});
}

void AutopilotSimHarness::clearSimulatedGoals() {
    simulated_goals_.clear();
}

void AutopilotSimHarness::reset(const Pose& initial_pose) {
    follower_.reset(initial_pose);
    trajectory_.clear();
    vision_seq_id_ = 0;
}

void AutopilotSimHarness::injectVisionTelemetry(double current_time) {
    if (!vision_enabled_ || simulated_goals_.empty()) return;

    VisionPacket v_pkt{};
    v_pkt.msg_type = MSG_TYPE_VISION;
    v_pkt.sequence_id = ++vision_seq_id_;
    v_pkt.timestamp_ns = static_cast<uint64_t>(current_time * 1e9);
    v_pkt.num_goals_reported = static_cast<uint8_t>(std::min(simulated_goals_.size(), static_cast<size_t>(3)));

    for (size_t i = 0; i < v_pkt.num_goals_reported; ++i) {
        const auto& goal = simulated_goals_[i];
        GoalMeasurement& gm = v_pkt.goals[i];
        gm.shape_type = goal.shape_type;
        gm.is_detected = 1;
        gm.num_hypotheses = 1;
        gm.hypotheses[0].position_cam = {static_cast<float>(goal.x), static_cast<float>(goal.y), static_cast<float>(goal.z)};
        gm.hypotheses[0].orientation_cam = {
            static_cast<float>(std::cos(goal.yaw * 0.5)),
            0.0f,
            0.0f,
            static_cast<float>(std::sin(goal.yaw * 0.5))
        };
        gm.hypotheses[0].confidence = 1.0f;
    }

    autopilot_.getEstimator().updateEstimates(v_pkt, current_time);

    // Keep vision timestamp fresh in autopilot to simulate active sensor heartbeat
    {
        std::lock_guard<std::mutex> lock(autopilot_.vision_mutex);
        autopilot_.last_vision_time = std::chrono::steady_clock::now();
    }
}

void AutopilotSimHarness::step(double dt) {
    double wall_time = std::chrono::duration<double>(
        std::chrono::steady_clock::now().time_since_epoch()).count();

    // 1. Inject vision updates
    injectVisionTelemetry(wall_time);

    // 2. Autopilot ticks given the noisy follower feedback pose
    Pose feedback = follower_.getFeedbackPose();
    FlightCommand cmd = autopilot_.tick(feedback, dt);

    // 3. Follower responds kinematically to the flight command setpoints
    follower_.step(cmd, dt);

    // 4. Record trajectory point
    TrajectoryPoint pt{};
    pt.timestamp = follower_.getTruePose().timestamp;
    pt.true_pose = follower_.getTruePose();
    pt.feedback_pose = feedback;
    pt.command = cmd;
    pt.flight_state = autopilot_.getState();
    trajectory_.push_back(pt);
}

MissionReport AutopilotSimHarness::runUntil(double max_duration_s,
                                           double dt,
                                           std::function<bool(const AutopilotSimHarness&)> termination_check) {
    double elapsed = 0.0;
    while (elapsed < max_duration_s) {
        step(dt);
        elapsed += dt;

        if (termination_check && termination_check(*this)) {
            break;
        }

        FlightState state = autopilot_.getState();
        if (state == FlightState::FLIGHT_STATE_ROTOR_STOPPED ||
            state == FlightState::FLIGHT_STATE_STANDBY_DISARM ||
            state == FlightState::FLIGHT_STATE_HOVER_WAIT ||
            state == FlightState::FLIGHT_STATE_ESTOP_TRIPPED) {
            // Reached terminal or completion state
            break;
        }
    }

    MissionReport report{};
    report.duration_s = elapsed;
    report.total_steps = trajectory_.size();
    report.final_state = autopilot_.getState();
    report.final_true_pose = follower_.getTruePose();
    report.rmse_tracking = follower_.getMetrics().rmse_position;
    report.max_tracking_error = follower_.getMetrics().max_tracking_error;
    report.total_distance_m = follower_.getMetrics().total_distance_traveled;
    report.mission_completed = (report.final_state == FlightState::FLIGHT_STATE_ROTOR_STOPPED ||
                                report.final_state == FlightState::FLIGHT_STATE_HOVER_WAIT);

    std::ostringstream oss;
    oss << "Mission " << (report.mission_completed ? "COMPLETED" : "TERMINATED")
        << " in " << std::fixed << std::setprecision(2) << report.duration_s << "s ("
        << report.total_steps << " steps). "
        << "Final state: " << static_cast<int>(report.final_state) << ", "
        << "Final Pos: [" << report.final_true_pose.x << ", "
        << report.final_true_pose.y << ", " << report.final_true_pose.z << "] m. "
        << "Tracking RMSE: " << std::setprecision(4) << report.rmse_tracking << " m, "
        << "Max Error: " << report.max_tracking_error << " m.";
    report.summary = oss.str();

    return report;
}

} // namespace sim
