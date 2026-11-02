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

void AutopilotSimHarness::addSimulatedTag(uint16_t tag_id, double x, double y, double z, double roll, double pitch, double yaw, float confidence) {
    simulated_tags_.push_back({tag_id, x, y, z, roll, pitch, yaw, confidence});
}

void AutopilotSimHarness::clearSimulatedTags() {
    simulated_tags_.clear();
}

void AutopilotSimHarness::reset(const Pose& initial_pose) {
    follower_.reset(initial_pose);
    trajectory_.clear();
    vision_seq_id_ = 0;
}

void AutopilotSimHarness::injectVisionTelemetry(double current_time) {
    if (!vision_enabled_ || (simulated_goals_.empty() && simulated_tags_.empty())) return;

    VisionPacket v_pkt{};
    v_pkt.msg_type = MSG_TYPE_VISION;
    v_pkt.sequence_id = ++vision_seq_id_;
    v_pkt.timestamp_ns = static_cast<uint64_t>(current_time * 1e9);

    // 1. Populate goals
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

    // 2. Populate fiducial tags (AprilTag / ChArUco global pose readings)
    // When the drone's camera is within viewing range of a surveyed tag, the vision
    // system computes the camera's world position and attitude.
    v_pkt.num_tags_detected = 0;
    const Pose& true_p = follower_.getTruePose();
    for (const auto& tag : simulated_tags_) {
        if (v_pkt.num_tags_detected >= 4) break;
        double dx = true_p.x - tag.x;
        double dy = true_p.y - tag.y;
        double dz = true_p.z - tag.z;
        double dist = std::sqrt(dx * dx + dy * dy + dz * dz);

        // Visible when within 3.5m radius of the surveyed tag
        if (dist <= 3.5) {
            TagMeasurement& tm = v_pkt.tags[v_pkt.num_tags_detected++];
            tm.tag_id = tag.tag_id;
            tm.is_valid = 1;
            tm.camera_pos_world = {
                static_cast<float>(true_p.x),
                static_cast<float>(true_p.y),
                static_cast<float>(true_p.z)
            };

            double cy = std::cos(true_p.yaw * 0.5);
            double sy = std::sin(true_p.yaw * 0.5);
            double cp = std::cos(true_p.pitch * 0.5);
            double sp = std::sin(true_p.pitch * 0.5);
            double cr = std::cos(true_p.roll * 0.5);
            double sr = std::sin(true_p.roll * 0.5);
            tm.camera_att_world.w = static_cast<float>(cr * cp * cy + sr * cp * sy);
            tm.camera_att_world.x = static_cast<float>(sr * cp * cy - cr * sp * sy);
            tm.camera_att_world.y = static_cast<float>(cr * sp * cy + sr * cp * sy);
            tm.camera_att_world.z = static_cast<float>(cr * cp * sy - sr * sp * cy);
            tm.confidence = tag.confidence;
        }
    }

    // Route packet through production AutopilotNode (which processes goals and forwards tags to FCU)
    autopilot_.handleVisionPacket(v_pkt, current_time);

    // In closed-loop simulation: the simulated ArduPilot FCU fuses the external vision position
    if (drift_correction_enabled_) {
        for (size_t i = 0; i < v_pkt.num_tags_detected; ++i) {
            if (v_pkt.tags[i].is_valid) {
                follower_.correctDriftWithVision(v_pkt.tags[i]);
            }
        }
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
    report.tag_readings_forwarded = autopilot_.tags_forwarded_count;
    report.total_drift_corrected_m = follower_.getMetrics().total_drift_corrected_m;
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
        << "Max Error: " << report.max_tracking_error << " m. "
        << "Tags Forwarded: " << report.tag_readings_forwarded << ", "
        << "Drift Corrected: " << report.total_drift_corrected_m << " m.";
    report.summary = oss.str();

    return report;
}

} // namespace sim
