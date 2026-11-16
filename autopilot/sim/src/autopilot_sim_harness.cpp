#include "autopilot_sim_harness.h"
#include <sstream>
#include <iomanip>
#include <cmath>

namespace sim {

AutopilotSimHarness::AutopilotSimHarness(const FollowerConfig& follower_cfg)
    : follower_(follower_cfg), perception_() {
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

    // Register tag in AutopilotNode's authoritative global survey map
    double cy = std::cos(yaw * 0.5);
    double sy = std::sin(yaw * 0.5);
    double cp = std::cos(pitch * 0.5);
    double sp = std::sin(pitch * 0.5);
    double cr = std::cos(roll * 0.5);
    double sr = std::sin(roll * 0.5);
    Quatf att{
        static_cast<float>(cr * cp * cy + sr * cp * sy),
        static_cast<float>(sr * cp * cy - cr * sp * sy),
        static_cast<float>(cr * sp * cy + sr * cp * sy),
        static_cast<float>(cr * cp * sy - sr * sp * cy)
    };
    autopilot_.registerSurveyedTag(tag_id, {static_cast<float>(x), static_cast<float>(y), static_cast<float>(z)}, att);
}

void AutopilotSimHarness::clearSimulatedTags() {
    simulated_tags_.clear();
    autopilot_.clearSurveyedTags();
}

void AutopilotSimHarness::reset(const Pose& initial_pose) {
    follower_.reset(initial_pose);
    trajectory_.clear();
    collision_records_.clear();
    sequence_traversed_.clear();
    last_observed_seq_index_ = 0;
    min_clearance_recorded_ = 1e9;
    vision_seq_id_ = 0;
}

void AutopilotSimHarness::injectVisionTelemetry(double current_time) {
    if (!vision_enabled_ || (simulated_goals_.empty() && simulated_tags_.empty())) return;

    VisionPacket v_pkt = perception_.generateVisionPacket(
        follower_.getTruePose(),
        simulated_goals_,
        simulated_tags_,
        ++vision_seq_id_,
        current_time
    );

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

    Pose feedback = follower_.getFeedbackPose();
    autopilot_.updateTelemetry(feedback);

    // 1. Inject vision updates
    injectVisionTelemetry(wall_time);

    // 2. Autopilot ticks given the noisy follower feedback pose
    FlightCommand cmd = autopilot_.tick(feedback, dt);

    // 3. Follower responds kinematically to the flight command setpoints
    follower_.step(cmd, dt);

    // 4. Physical Collision and Obstacle Clearance Check
    const Pose& true_pose = follower_.getTruePose();
    CollisionCheckResult coll = CollisionDetector::check(
        true_pose.x, true_pose.y, true_pose.z, true_pose.yaw,
        simulated_goals_
    );
    if (coll.min_clearance_m < min_clearance_recorded_) {
        min_clearance_recorded_ = coll.min_clearance_m;
    }
    if (coll.is_hit) {
        CollisionRecord rec{};
        rec.timestamp = true_pose.timestamp;
        rec.shape_type = coll.shape_hit;
        rec.description = coll.hit_message;
        rec.drone_pose = true_pose;
        rec.clearance = coll.min_clearance_m;
        collision_records_.push_back(rec);
    }

    // 5. Sequence Progress Tracking
    uint8_t cur_seq = autopilot_.current_seq_index;
    if (cur_seq > last_observed_seq_index_) {
        for (uint8_t idx = last_observed_seq_index_; idx < cur_seq && idx < autopilot_.active_sequence_len; ++idx) {
            sequence_traversed_.push_back(autopilot_.goal_sequence[idx]);
        }
        last_observed_seq_index_ = cur_seq;
    }

    // 6. Record trajectory point
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

    // Final check for traversed sequence if mission finished
    if (autopilot_.current_seq_index >= autopilot_.active_sequence_len &&
        sequence_traversed_.size() < autopilot_.active_sequence_len) {
        for (size_t idx = sequence_traversed_.size(); idx < autopilot_.active_sequence_len; ++idx) {
            sequence_traversed_.push_back(autopilot_.goal_sequence[idx]);
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
    report.collision_count = collision_records_.size();
    report.min_clearance_m = min_clearance_recorded_;
    report.collisions = collision_records_;
    report.sequence_traversed = sequence_traversed_;
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
        << "Drift Corrected: " << report.total_drift_corrected_m << " m. "
        << "Collisions: " << report.collision_count << ", "
        << "Min Clearance: " << std::setprecision(3) << report.min_clearance_m << " m.";
    report.summary = oss.str();

    return report;
}

} // namespace sim
