#include "autopilot_node.h"
#include "generated_surveyed_tags.h"
#include <iostream>
#include <chrono>
#include <cmath>
#include <arpa/inet.h>
#include <unistd.h>
#include <cstring>
#include <set>

AutopilotNode::AutopilotNode()
    : fcu(14550, "127.0.0.1", 14551),
      planner([](){
          PlannerConfig cfg = {};
          cfg.max_velocity = 2.0;
          cfg.max_vertical_velocity = 1.0;
          cfg.max_acceleration = 100.0;
          cfg.position_p_gain = 1.0;
          cfg.max_yaw_rate = 1.0;
          cfg.yaw_p_gain = 1.0;
          cfg.arrival_tolerance = 0.2;
          return cfg;
      }()),
      estimator(0.5, 2.0),
      logger("127.0.0.1", 14554)
{
    // Load pre-compiled surveyed tag map generated at compile time from surveyed_tags.json
    loadCompiledSurveyedTags(*this);

    ui_sock = socket(AF_INET, SOCK_DGRAM, 0);
    struct sockaddr_in ui_addr;
    memset(&ui_addr, 0, sizeof(ui_addr));
    ui_addr.sin_family = AF_INET;
    ui_addr.sin_port = htons(14552);
    ui_addr.sin_addr.s_addr = INADDR_ANY;
    bind(ui_sock, (struct sockaddr *)&ui_addr, sizeof(ui_addr));

    struct timeval tv;
    tv.tv_sec = 0; tv.tv_usec = 100000;
    setsockopt(ui_sock, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));

    vision_sock = socket(AF_INET, SOCK_DGRAM, 0);
    struct sockaddr_in vision_addr;
    memset(&vision_addr, 0, sizeof(vision_addr));
    vision_addr.sin_family = AF_INET;
    vision_addr.sin_port = htons(14553);
    vision_addr.sin_addr.s_addr = INADDR_ANY;
    bind(vision_sock, (struct sockaddr *)&vision_addr, sizeof(vision_addr));
    setsockopt(vision_sock, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));

    telem_sock = socket(AF_INET, SOCK_DGRAM, 0);
    memset(&telem_dest, 0, sizeof(telem_dest));
    telem_dest.sin_family = AF_INET;
    telem_dest.sin_port = htons(14554);
    inet_pton(AF_INET, "127.0.0.1", &telem_dest.sin_addr);
    
    last_vision_time = std::chrono::steady_clock::now();
}

AutopilotNode::~AutopilotNode() { stop(); }

void AutopilotNode::start() {
    running = true;
    fcu.start();
    ui_thread = std::thread(&AutopilotNode::ui_loop, this);
    vision_thread = std::thread(&AutopilotNode::vision_loop, this);
    loop_thread = std::thread(&AutopilotNode::run, this);
}

void AutopilotNode::stop() {
    running = false;
    fcu.stop();
    if (loop_thread.joinable()) loop_thread.join();
    if (ui_thread.joinable()) ui_thread.join();
    if (vision_thread.joinable()) vision_thread.join();
    if (ui_sock >= 0) { close(ui_sock); ui_sock = -1; }
    if (vision_sock >= 0) { close(vision_sock); vision_sock = -1; }
    if (telem_sock >= 0) { close(telem_sock); telem_sock = -1; }
}

void AutopilotNode::handleUICommand(const UICommandPacket& pkt, const struct sockaddr_in& from, socklen_t from_len) {
    std::lock_guard<std::mutex> lock(cmd_mutex);
    switch (pkt.command) {
        case UI_CMD_ARM:
            is_armed = 1;
            estop_active = 0;
            if (state == FlightState::FLIGHT_STATE_STANDBY_DISARM || state == FlightState::FLIGHT_STATE_ROTOR_STOPPED || state == FlightState::FLIGHT_STATE_ESTOP_TRIPPED) {
                state = FlightState::FLIGHT_STATE_ARMED;
            }
            break;
        case UI_CMD_DISARM:
            is_armed = 0;
            estop_active = 0;
            state = FlightState::FLIGHT_STATE_STANDBY_DISARM;
            current_seq_index = 0;
            new_goal = true;
            has_active_target = false;
            goal_phase = 0;
            align_ticks = 0;
            standoff_ticks = 0;
            seek_ticks = 0;
            active_target_pose = {};
            estimator.reset();
            planner.reset();
            break;
        case UI_CMD_START_MISSION:
            is_armed = 1;
            estop_active = 0;
            if (state == FlightState::FLIGHT_STATE_HOVER_WAIT && last_current_pose_.z > 0.4) {
                state = FlightState::FLIGHT_STATE_PATH_FOLLOWING;
            } else {
                state = FlightState::FLIGHT_STATE_TAKEOFF;
            }
            current_seq_index = 0;
            new_goal = true;
            last_approach_dx_ = 1.0;
            last_approach_dy_ = 0.0;
            approach_sign_ = 1.0;
            has_active_target = false;
            goal_phase = 0;
            align_ticks = 0;
            standoff_ticks = 0;
            seek_ticks = 0;
            active_target_pose = {};
            // Preserve estimator goal positions so drone knows where targets are!
            planner.reset();
            active_sequence_len = pkt.sequence_length;
            memcpy(goal_sequence, pkt.goal_sequence, sizeof(goal_sequence));
            configured_end_action = pkt.end_action;
            hover_timeout_s = pkt.hover_timeout_s;
            use_tube_constraint = pkt.use_tube_constraint;
            tube_rail_y = pkt.tube_rail_y;
            tube_rail_z = pkt.tube_rail_z;
            break;
        case UI_CMD_SET_SEQUENCE:
            active_sequence_len = pkt.sequence_length;
            memcpy(goal_sequence, pkt.goal_sequence, sizeof(goal_sequence));
            configured_end_action = pkt.end_action;
            hover_timeout_s = pkt.hover_timeout_s;
            current_seq_index = 0;
            if (is_armed && (state == FlightState::FLIGHT_STATE_HOVER_WAIT || state == FlightState::FLIGHT_STATE_PATH_FOLLOWING || state == FlightState::FLIGHT_STATE_GOAL_SEEK)) {
                state = (last_current_pose_.z > 0.4) ? FlightState::FLIGHT_STATE_PATH_FOLLOWING : FlightState::FLIGHT_STATE_TAKEOFF;
            }
            new_goal = true;
            last_approach_dx_ = 1.0;
            last_approach_dy_ = 0.0;
            approach_sign_ = 1.0;
            has_active_target = false;
            goal_phase = 0;
            align_ticks = 0;
            standoff_ticks = 0;
            seek_ticks = 0;
            active_target_pose = {};
            // Preserve estimator goal positions so drone knows where targets are!
            break;
        case UI_CMD_SET_CONSTRAINTS:
            use_tube_constraint = pkt.use_tube_constraint;
            tube_rail_y = pkt.tube_rail_y;
            tube_rail_z = pkt.tube_rail_z;
            break;
        case UI_CMD_PAUSE_MISSION:
            if (state == FlightState::FLIGHT_STATE_PATH_FOLLOWING || state == FlightState::FLIGHT_STATE_GOAL_SEEK) {
                state = FlightState::FLIGHT_STATE_HOVER_WAIT;
            }
            break;
        case UI_CMD_RESUME_MISSION:
            if (state == FlightState::FLIGHT_STATE_HOVER_WAIT) {
                state = FlightState::FLIGHT_STATE_PATH_FOLLOWING;
            }
            break;
        case UI_CMD_LAND_NOW:
            state = FlightState::FLIGHT_STATE_LANDING;
            break;
        case UI_CMD_ESTOP:
            state = FlightState::FLIGHT_STATE_ESTOP_TRIPPED;
            estop_active = 1;
            break;
        default:
            break;
    }

    last_acked_cmd_id = pkt.command_id;

    UIStatusPacket status = {};
    status.msg_type = MSG_TYPE_UI_STATUS;
    status.sequence_id = status_seq_id++;
    auto now_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::steady_clock::now().time_since_epoch()).count();
    status.timestamp_ns = now_ns;
    status.flight_state = (uint8_t)state;
    status.is_armed = is_armed;
    status.estop_active = estop_active;
    status.active_sequence_len = active_sequence_len;
    status.current_seq_index = current_seq_index;
    status.current_target_shape = (current_seq_index < active_sequence_len) ? goal_sequence[current_seq_index] : (uint8_t)SHAPE_NONE;
    status.configured_end_action = configured_end_action;
    status.battery_voltage = 12.0f;
    status.battery_remaining = 90.0f;
    status.active_error_flags = 0;
    status.last_acked_cmd_id = pkt.command_id;
    status.cmd_ack_result = 0;

    if (from_len > 0 && ui_sock >= 0) {
        sendto(ui_sock, &status, sizeof(status), 0, (struct sockaddr *)&from, from_len);
    }
}

void AutopilotNode::ui_loop() {
    char buffer[4096];
    while (running) {
        struct sockaddr_in from; socklen_t from_len = sizeof(from);
        int n = recvfrom(ui_sock, buffer, sizeof(buffer), 0, (struct sockaddr *)&from, &from_len);
        if (n == sizeof(UICommandPacket)) {
            UICommandPacket* pkt = reinterpret_cast<UICommandPacket*>(buffer);
            if (pkt->msg_type == MSG_TYPE_UI_CMD) {
                handleUICommand(*pkt, from, from_len);
            }
        }
    }
}

void AutopilotNode::registerSurveyedTag(uint16_t tag_id, const Vec3f& world_pos, const Quatf& world_att, float size_m) {
    std::lock_guard<std::mutex> lock(tag_map_mutex_);
    SurveyedTagPose tag{};
    tag.tag_id = tag_id;
    tag.world_pos = world_pos;
    tag.world_att = world_att;
    tag.size_m = size_m;
    tag.is_known = true;
    surveyed_tags_[tag_id] = tag;
}

bool AutopilotNode::getSurveyedTag(uint16_t tag_id, SurveyedTagPose& out_tag) const {
    std::lock_guard<std::mutex> lock(tag_map_mutex_);
    auto it = surveyed_tags_.find(tag_id);
    if (it != surveyed_tags_.end()) {
        out_tag = it->second;
        return true;
    }
    return false;
}

bool AutopilotNode::hasSurveyedTag(uint16_t tag_id) const {
    std::lock_guard<std::mutex> lock(tag_map_mutex_);
    return surveyed_tags_.find(tag_id) != surveyed_tags_.end();
}

void AutopilotNode::clearSurveyedTags() {
    std::lock_guard<std::mutex> lock(tag_map_mutex_);
    surveyed_tags_.clear();
}

void AutopilotNode::handleVisionPacket(const VisionPacket& pkt, double current_time) {
    VisionPacket transformed_pkt = pkt;
    Pose cur;
    {
        std::lock_guard<std::mutex> lock(pose_mutex_);
        cur = last_current_pose_;
    }
    if (cur.z == 0.0 && cur.x == 0.0 && cur.y == 0.0) {
        cur = fcu.getTelemetry().current_pose;
    }

    double cy = std::cos(cur.yaw), sy = std::sin(cur.yaw);
    double cp = std::cos(cur.pitch), sp = std::sin(cur.pitch);
    double cr = std::cos(cur.roll), sr = std::sin(cur.roll);

    // Three.js drone rotation matrix (order YZX): R = Ry(yaw) * Rz(-pitch) * Rx(roll)
    double r00 = cy * cp;
    double r01 = cy * sp * cr + sy * sr;
    double r02 = -cy * sp * sr + sy * cr;

    double r10 = -sp;
    double r11 = cp * cr;
    double r12 = -cp * sr;

    double r20 = -sy * cp;
    double r21 = -sy * sp * cr + cy * sr;
    double r22 = sy * sp * sr + cy * cr;

    // Camera origin in world coordinates (mount offset: 0.18m forward, 0.04m up):
    double p_cam_tx = r00 * 0.18 + r01 * 0.04;
    double p_cam_ty = r10 * 0.18 + r11 * 0.04;
    double p_cam_tz = r20 * 0.18 + r21 * 0.04;

    double cam_x = cur.x + p_cam_tx;
    double cam_y = cur.y - p_cam_tz;
    double cam_z = cur.z + p_cam_ty;

    // Camera basis vectors in world coordinates (forward = +Z_cam, right = +X_cam, down = +Y_cam):
    double fwd_x = r00,   fwd_y = -r20,  fwd_z = r10;
    double right_x = r02, right_y = -r22, right_z = r12;
    double down_x = -r01, down_y = r21,  down_z = -r11;

    for (uint8_t i = 0; i < transformed_pkt.num_goals_reported && i < 3; ++i) {
        GoalMeasurement& gm = transformed_pkt.goals[i];
        if (gm.is_detected && gm.num_hypotheses > 0) {
            PoseHypothesis& hyp = gm.hypotheses[0];
            double z_cam = hyp.position_cam.z;
            double x_cam = hyp.position_cam.x;
            double y_cam = hyp.position_cam.y;

            // Full 3D attitude-compensated projection:
            double world_x = cam_x + z_cam * fwd_x + x_cam * right_x + y_cam * down_x;
            double world_y = cam_y + z_cam * fwd_y + x_cam * right_y + y_cam * down_y;
            double world_z = cam_z + z_cam * fwd_z + x_cam * right_z + y_cam * down_z;

            // Plausibility check: gates in the arena safety cage are strictly bounded
            if (world_z > 2.30 || world_z < 0.20 || z_cam > 5.5 || z_cam < 0.20 || world_x > 8.0 || world_x < -2.0 || std::abs(world_y) > 4.0) {
                gm.is_detected = 0;
                continue;
            }

            hyp.position_cam.x = static_cast<float>(world_x);
            hyp.position_cam.y = static_cast<float>(world_y);
            hyp.position_cam.z = static_cast<float>(world_z);

            Quatf q = hyp.orientation_cam;
            double world_yaw = std::atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z));
            while (world_yaw > M_PI) world_yaw -= 2.0 * M_PI;
            while (world_yaw < -M_PI) world_yaw += 2.0 * M_PI;

            double cy = std::cos(world_yaw * 0.5);
            double sy = std::sin(world_yaw * 0.5);
            hyp.orientation_cam = Quatf{static_cast<float>(cy), 0.0f, 0.0f, static_cast<float>(sy)};
        }
    }

    estimator.updateEstimates(transformed_pkt, current_time);
    {
        std::lock_guard<std::mutex> lock(vision_mutex);
        last_vision_time = std::chrono::steady_clock::now();
    }

    // Pass through global tag measurements (AprilTag / ChArUco) to ArduPilot FCU via MAVLink
    for (uint8_t i = 0; i < pkt.num_tags_detected && i < 4; ++i) {
        const TagMeasurement& tag = pkt.tags[i];
        if (tag.is_valid) {
            tags_received_count++;
            uint64_t usec = (pkt.timestamp_ns > 0)
                ? (pkt.timestamp_ns / 1000)
                : static_cast<uint64_t>(current_time * 1e6);
            fcu.sendVisionPositionEstimate(tag, usec);
            tags_forwarded_count++;
            last_tag_measurement = tag;
        }
    }
}

void AutopilotNode::vision_loop() {
    char buffer[2048];
    while (running) {
        struct sockaddr_in from; socklen_t from_len = sizeof(from);
        int n = recvfrom(vision_sock, buffer, sizeof(buffer), 0, (struct sockaddr *)&from, &from_len);
        if (n == sizeof(VisionPacket)) {
            VisionPacket* pkt = reinterpret_cast<VisionPacket*>(buffer);
            if (pkt->msg_type == MSG_TYPE_VISION) {
                double current_time = std::chrono::duration<double>(
                    std::chrono::steady_clock::now().time_since_epoch()).count();
                handleVisionPacket(*pkt, current_time);
            }
        }
    }
}

void AutopilotNode::computeNonTargetRepulsion(
    const Pose& current_pose,
    uint8_t target_shape,
    double des_vx,
    double des_vy,
    double& out_apf_vx,
    double& out_apf_vy,
    double& out_offset_x,
    double& out_offset_y) {
    out_apf_vx = 0.0;
    out_apf_vy = 0.0;
    out_offset_x = 0.0;
    out_offset_y = 0.0;

    struct ObstacleInfo {
        double x;
        double y;
        double z;
        uint8_t shape_type;
    };
    std::vector<ObstacleInfo> obstacles;
    std::set<uint8_t> seen_shapes;

    // 1. All known goals from GoalEstimator (retains all shapes seen so far)
    std::vector<TrackedGoal> known_goals = estimator.getAllKnownGoals();
    for (const auto& g : known_goals) {
        if (g.shape_type == target_shape) continue;
        obstacles.push_back({g.pose.x, g.pose.y, g.pose.z, g.shape_type});
        seen_shapes.insert(g.shape_type);
    }

    const double R_nogo = 0.55;       // Visual No-Go boundary cylinder (0.55m)
    const double R_influence = 0.70;  // Repulsion influence zone (0.70m)

    for (const auto& obs : obstacles) {
        // Height check: gate structure and pylon extend from ground z=0 to obs.z + 0.35m
        if (current_pose.z > obs.z + 0.40) continue;

        double dx = current_pose.x - obs.x;
        double dy = current_pose.y - obs.y;
        double dist_2d = std::hypot(dx, dy);

        if (dist_2d >= R_influence) continue;

        if (dist_2d < 0.01) {
            dx = 1.0; dy = 0.0; dist_2d = 1.0;
        }

        // Penetration ratio: 0.0 at R_influence, 1.0 at R_nogo, >1.0 inside boundary
        double pen = (R_influence - dist_2d) / (R_influence - R_nogo);
        pen = std::max(0.0, std::min(2.5, pen));

        // Unit radial vector pointing away from obstacle center
        double ux = dx / dist_2d;
        double uy = dy / dist_2d;

        // Cross product with desired travel direction to determine detour side
        double cross = dx * des_vy - dy * des_vx;
        double side = (std::abs(cross) > 1e-3) ? ((cross >= 0.0) ? 1.0 : -1.0) : ((dy >= 0.0) ? 1.0 : -1.0);

        // Tangential unit vector around obstacle
        double vx = -side * uy;
        double vy = side * ux;

        // Combined repulsive + tangential direction
        double dir_x = ux * 0.70 + vx * 0.70;
        double dir_y = uy * 0.70 + vy * 0.70;
        double dir_norm = std::hypot(dir_x, dir_y);
        if (dir_norm > 1e-4) {
            dir_x /= dir_norm;
            dir_y /= dir_norm;
        }

        out_apf_vx += dir_x * pen * 2.5;
        out_apf_vy += dir_y * pen * 2.5;
        out_offset_x += dir_x * pen * 0.45;
        out_offset_y += dir_y * pen * 0.45;
    }

    double tot_off = std::hypot(out_offset_x, out_offset_y);
    if (tot_off > 1.2) {
        out_offset_x = (out_offset_x / tot_off) * 1.2;
        out_offset_y = (out_offset_y / tot_off) * 1.2;
    }
}

void AutopilotNode::updateApproachDirection(const Pose& target_pose) {
    double hoop_nx = std::cos(target_pose.yaw);
    double hoop_ny = std::sin(target_pose.yaw);
    double a_norm = std::hypot(hoop_nx, hoop_ny);
    if (a_norm > 1e-4) {
        hoop_nx /= a_norm;
        hoop_ny /= a_norm;
    } else {
        hoop_nx = 1.0;
        hoop_ny = 0.0;
    }

    approach_sign_ = 1.0;

    if (current_seq_index > 0) {
        double seq_dx = target_pose.x - last_completed_goal_pose_.x;
        double seq_dy = target_pose.y - last_completed_goal_pose_.y;
        double prev_nx = std::cos(last_completed_goal_pose_.yaw);
        double prev_ny = std::sin(last_completed_goal_pose_.yaw);
        double p_norm = std::hypot(prev_nx, prev_ny);
        if (p_norm > 1e-4) { prev_nx /= p_norm; prev_ny /= p_norm; }
        else { prev_nx = 1.0; prev_ny = 0.0; }

        double dot_normal = hoop_nx * prev_nx + hoop_ny * prev_ny;
        double delta_along = std::abs(seq_dx * prev_nx + seq_dy * prev_ny);
        double delta_perp = std::abs(-seq_dx * prev_ny + seq_dy * prev_nx);

        // Side-by-side goals: significant lateral offset (>= 0.35m), limited longitudinal offset (<= 1.2m),
        // and gate normal vectors roughly parallel (|dot_normal| > 0.5)
        if (delta_perp >= 0.35 && delta_along <= 1.20 && std::abs(dot_normal) > 0.5) {
            // Exit position from previously completed goal
            double exit_x = last_completed_goal_pose_.x + last_approach_dx_ * 0.55;
            double exit_y = last_completed_goal_pose_.y + last_approach_dy_ * 0.55;
            double dot_exit = (exit_x - target_pose.x) * hoop_nx + (exit_y - target_pose.y) * hoop_ny;

            if (dot_exit > 0.10) {
                // Drone exited previous goal on the positive normal side of target gate (behind the gate).
                // Traverse this gate "backwards" (-hoop_n)!
                approach_sign_ = -1.0;
            } else if (dot_exit < -0.10) {
                // Drone exited in front of target gate; traverse forward (+hoop_n)!
                approach_sign_ = 1.0;
            } else {
                // Exited level with gate: reverse orientation relative to previous gate traversal
                double dot_prev = last_approach_dx_ * hoop_nx + last_approach_dy_ * hoop_ny;
                approach_sign_ = (dot_prev >= 0.0) ? -1.0 : 1.0;
            }
        }
    }

    approach_dx = hoop_nx * approach_sign_;
    approach_dy = hoop_ny * approach_sign_;
}

FlightCommand AutopilotNode::tick(const Pose& current_pose, double dt) {
    updateTelemetry(current_pose);
    FlightCommand cmd = {};
    cmd.enable = false;
    cmd.estop = false;
    
    // Safety
    auto now = std::chrono::steady_clock::now();
    double time_since_vision = 0;
    {
        std::lock_guard<std::mutex> lock(vision_mutex);
        time_since_vision = std::chrono::duration<double>(now - last_vision_time).count();
    }
    if (time_since_vision > 2.5 && state != FlightState::FLIGHT_STATE_ESTOP_TRIPPED 
        && state != FlightState::FLIGHT_STATE_STANDBY_DISARM && state != FlightState::FLIGHT_STATE_ROTOR_STOPPED) {
        state = FlightState::FLIGHT_STATE_ESTOP_TRIPPED;
        estop_active = 1;
    }

    std::lock_guard<std::mutex> lock(cmd_mutex);

    switch(state) {
        case FlightState::FLIGHT_STATE_ESTOP_TRIPPED:
            cmd.estop = true;
            cmd.enable = false;
            break;

        case FlightState::FLIGHT_STATE_TAKEOFF: {
            cmd.enable = true;
            double takeoff_z = (use_tube_constraint && tube_rail_z != 0.0f) ? std::abs(tube_rail_z) : 1.0;
            Pose takeoff_target = {current_pose.x, current_pose.y, takeoff_z, 0, 0, current_pose.yaw, 0, 1.0};
            planner.plan(current_pose, takeoff_target, dt);
            cmd.target_x = takeoff_target.x;
            cmd.target_y = takeoff_target.y;
            cmd.target_z = takeoff_target.z;
            cmd.target_yaw = takeoff_target.yaw;
            if (planner.has_arrived(current_pose, takeoff_target)) {
                state = FlightState::FLIGHT_STATE_PATH_FOLLOWING;
                new_goal = true;
                last_completed_goal_pose_ = {current_pose.x, current_pose.y, takeoff_z, 0, 0, current_pose.yaw, 0, 1.0};
            }
            break;
        }

        case FlightState::FLIGHT_STATE_PATH_FOLLOWING:
        case FlightState::FLIGHT_STATE_GOAL_SEEK: {
            if (current_seq_index < active_sequence_len) {
                uint8_t target_shape = goal_sequence[current_seq_index];
                double current_time = std::chrono::duration<double>(
                    std::chrono::steady_clock::now().time_since_epoch()).count();
                std::vector<TrackedGoal> ests = estimator.getCurrentEstimates(current_time);
                
                bool found = false;
                Pose estimated_pose = {};
                for (const auto& tg : ests) {
                    if (tg.shape_type == target_shape) {
                        estimated_pose = tg.pose;
                        found = true;
                        break;
                    }
                }
                
                if (found) {
                    if (goal_phase == 0 || goal_phase == 1) {
                        active_target_pose = estimated_pose;
                        has_active_target = true;
                    }
                } else if (has_active_target) {
                    // Vision dropped detection (e.g. gate rim clipped by frame edges when close)
                    // If we already have a locked active target pose, retain it!
                    double dist_to_target = std::hypot(current_pose.x - active_target_pose.x,
                                                       current_pose.y - active_target_pose.y);
                    if (dist_to_target < 2.5 || goal_phase >= 1) {
                        estimated_pose = active_target_pose;
                        found = true;
                    }
                } else {
                    // Check if GoalEstimator has a recent last known estimate within proximity
                    TrackedGoal stale_g;
                    if (estimator.getLastKnownEstimate(target_shape, stale_g)) {
                        double dist_to_stale = std::hypot(current_pose.x - stale_g.pose.x,
                                                          current_pose.y - stale_g.pose.y);
                        if (dist_to_stale < 1.8) {
                            active_target_pose = stale_g.pose;
                            has_active_target = true;
                            estimated_pose = active_target_pose;
                            found = true;
                        }
                    }
                }
                
                if (found) {
                    state = FlightState::FLIGHT_STATE_PATH_FOLLOWING;
                    cmd.enable = true;
                    Pose raw_target = estimated_pose;
                    if (use_tube_constraint) {
                        raw_target.y = tube_rail_y;
                        if (tube_rail_z != 0.0f) raw_target.z = std::abs(tube_rail_z);
                    }
                    
                    if (new_goal) {
                        new_goal = false;
                        goal_phase = 0;
                        standoff_ticks = 0;
                        align_ticks = 0;
                        seek_ticks = 0;
                        updateApproachDirection(raw_target);
                    } else if (goal_phase < 2) {
                        double hoop_nx = cos(raw_target.yaw);
                        double hoop_ny = sin(raw_target.yaw);
                        double a_norm = std::hypot(hoop_nx, hoop_ny);
                        if (a_norm > 1e-4) {
                            hoop_nx /= a_norm;
                            hoop_ny /= a_norm;
                        } else {
                            hoop_nx = 1.0;
                            hoop_ny = 0.0;
                        }
                        approach_dx = hoop_nx * approach_sign_;
                        approach_dy = hoop_ny * approach_sign_;
                    }
                    Pose adjusted_target = raw_target;

                    if (goal_phase == 0) {
                        standoff_ticks++;
                        adjusted_target.x = raw_target.x - approach_dx * 0.75;
                        adjusted_target.y = raw_target.y - approach_dy * 0.75;
                        adjusted_target.z = raw_target.z;
                        adjusted_target.yaw = atan2(approach_dy, approach_dx);

                        // If drone is behind the target gate (along gate normal corridor), route safely around the gate frame
                        double dot_fwd = (current_pose.x - raw_target.x) * approach_dx +
                                         (current_pose.y - raw_target.y) * approach_dy;
                        if (dot_fwd > -0.20) {
                            double perp_dx = -approach_dy;
                            double perp_dy = approach_dx;
                            double lat = (current_pose.x - raw_target.x) * perp_dx +
                                         (current_pose.y - raw_target.y) * perp_dy;
                            double side_sign = (lat >= 0.0) ? 1.0 : -1.0;
                            adjusted_target.x = raw_target.x + perp_dx * (side_sign * 0.65) - approach_dx * 0.35;
                            adjusted_target.y = raw_target.y + perp_dy * (side_sign * 0.65) - approach_dy * 0.35;
                        }

                        // Repulsion avoidance from non-target goals (No-Go Keep-Out Zones)
                        double des_vx = adjusted_target.x - current_pose.x;
                        double des_vy = adjusted_target.y - current_pose.y;
                        double apf_vx = 0.0, apf_vy = 0.0, apf_off_x = 0.0, apf_off_y = 0.0;
                        computeNonTargetRepulsion(current_pose, target_shape, des_vx, des_vy,
                                                  apf_vx, apf_vy, apf_off_x, apf_off_y);

                        adjusted_target.x += apf_off_x;
                        adjusted_target.y += apf_off_y;

                        planner.plan(current_pose, adjusted_target, dt, apf_vx, apf_vy);
                        cmd.target_x = adjusted_target.x;
                        cmd.target_y = adjusted_target.y;
                        cmd.target_z = adjusted_target.z;
                        cmd.target_yaw = adjusted_target.yaw;

                        double delta_x = current_pose.x - adjusted_target.x;
                        double delta_y = current_pose.y - adjusted_target.y;
                        double delta_z = current_pose.z - adjusted_target.z;
                        double standoff_dist = std::hypot(delta_x, delta_y, delta_z);
                        double lat_offset = std::hypot(-delta_x * approach_dy + delta_y * approach_dx, delta_z);

                        if (dot_fwd <= -0.20) {
                            if (standoff_dist < 0.20 && lat_offset < 0.08) {
                                goal_phase = 1;
                                align_ticks = 0;
                            } else if (standoff_ticks > 35 && standoff_dist < 0.35 && lat_offset < 0.12) {
                                goal_phase = 1;
                                align_ticks = 0;
                            }
                        }
                    } else if (goal_phase == 1) {
                        align_ticks++;
                        adjusted_target.x = raw_target.x - approach_dx * 0.75;
                        adjusted_target.y = raw_target.y - approach_dy * 0.75;
                        adjusted_target.z = raw_target.z;
                        adjusted_target.yaw = atan2(approach_dy, approach_dx);

                        double apf_vx = 0.0, apf_vy = 0.0, apf_off_x = 0.0, apf_off_y = 0.0;
                        computeNonTargetRepulsion(current_pose, target_shape, 0.0, 0.0,
                                                  apf_vx, apf_vy, apf_off_x, apf_off_y);
                        adjusted_target.x += apf_off_x;
                        adjusted_target.y += apf_off_y;

                        planner.plan(current_pose, adjusted_target, dt, apf_vx, apf_vy);
                        cmd.target_x = adjusted_target.x;
                        cmd.target_y = adjusted_target.y;
                        cmd.target_z = adjusted_target.z;
                        cmd.target_yaw = adjusted_target.yaw;

                        double dyaw = adjusted_target.yaw - current_pose.yaw;
                        while (dyaw > M_PI) dyaw -= 2.0 * M_PI;
                        while (dyaw < -M_PI) dyaw += 2.0 * M_PI;

                        // Lateral error perpendicular to the approach corridor normal
                        double delta_x = current_pose.x - adjusted_target.x;
                        double delta_y = current_pose.y - adjusted_target.y;
                        double e_perp = -delta_x * approach_dy + delta_y * approach_dx;
                        double e_z = current_pose.z - raw_target.z;
                        double lat_err = std::hypot(e_perp, e_z);
                        double e_along = delta_x * approach_dx + delta_y * approach_dy;

                        // Require hold for at least 6 ticks (~300ms) to ensure lateral and angular momentum has settled
                        bool tight_aligned = (align_ticks >= 6 && std::abs(dyaw) < 0.06 && lat_err < 0.035 && std::abs(e_along) < 0.20);
                        bool relaxed_aligned = (align_ticks > 20 && std::abs(dyaw) < 0.08 && lat_err < 0.050);
                        bool watchdog_aligned = (align_ticks > 40 && std::abs(dyaw) < 0.12 && lat_err < 0.065);

                        if (tight_aligned || relaxed_aligned || watchdog_aligned) {
                            goal_phase = 2;
                            align_ticks = 0;
                        }
                    } else {
                        // Traversal punch: steadily advance along normal corridor while keeping cross-track centered
                        double dot_fwd = (current_pose.x - raw_target.x) * approach_dx +
                                         (current_pose.y - raw_target.y) * approach_dy;
                        double step_fwd = std::min(0.55, std::max(-0.25, dot_fwd + 0.35));

                        cmd.target_x = raw_target.x + approach_dx * step_fwd;
                        cmd.target_y = raw_target.y + approach_dy * step_fwd;
                        cmd.target_z = raw_target.z;
                        cmd.target_yaw = atan2(approach_dy, approach_dx);

                        Pose traverse_target = {raw_target.x + approach_dx * 0.55,
                                                raw_target.y + approach_dy * 0.55,
                                                raw_target.z, 0, 0, cmd.target_yaw, 0, 1.0};
                        planner.plan(current_pose, traverse_target, dt, 0.0, 0.0);

                        if (dot_fwd >= 0.45 || planner.has_arrived(current_pose, traverse_target)) {
                            last_completed_goal_pose_ = raw_target;
                            last_approach_dx_ = approach_dx;
                            last_approach_dy_ = approach_dy;
                            current_seq_index++;
                            new_goal = true;
                            has_active_target = false;
                            goal_phase = 0;
                            align_ticks = 0;
                            standoff_ticks = 0;
                        }
                    }
                } else {
                    // Search behavior: orient and localize search using stale estimate if available
                    state = FlightState::FLIGHT_STATE_GOAL_SEEK;
                    cmd.enable = true;
                    seek_ticks++;

                    TrackedGoal stale_goal;
                    bool has_stale = estimator.getLastKnownEstimate(target_shape, stale_goal);

                    if (has_stale) {
                        if (new_goal) {
                            updateApproachDirection(stale_goal.pose);
                        }
                        double gn_x = approach_dx;
                        double gn_y = approach_dy;
                        double norm_len = std::hypot(gn_x, gn_y);
                        if (norm_len < 1e-3) { gn_x = 1.0; gn_y = 0.0; }
                        else { gn_x /= norm_len; gn_y /= norm_len; }

                        // Check if drone is behind the gate entrance plane
                        double dot_fwd_seek = (current_pose.x - stale_goal.pose.x) * gn_x +
                                              (current_pose.y - stale_goal.pose.y) * gn_y;

                        // Standoff position in front of the gate entrance
                        double standoff_x = stale_goal.pose.x - gn_x * 0.75;
                        double standoff_y = stale_goal.pose.y - gn_y * 0.75;
                        double standoff_z = stale_goal.pose.z;

                        if (dot_fwd_seek > -0.20) {
                            // Drone is behind or level with the gate; steer around the outside frame
                            double perp_x = -gn_y;
                            double perp_y = gn_x;
                            double lat_off = (current_pose.x - stale_goal.pose.x) * perp_x +
                                             (current_pose.y - stale_goal.pose.y) * perp_y;
                            double side_sign = (lat_off >= 0.0) ? 1.0 : -1.0;
                            standoff_x = stale_goal.pose.x + perp_x * (side_sign * 0.65) - gn_x * 0.35;
                            standoff_y = stale_goal.pose.y + perp_y * (side_sign * 0.65) - gn_y * 0.35;
                        }

                        double dist_to_standoff = std::hypot(current_pose.x - standoff_x,
                                                             current_pose.y - standoff_y);
                        double dist_to_goal = std::hypot(current_pose.x - stale_goal.pose.x,
                                                         current_pose.y - stale_goal.pose.y);

                        // If close to standoff (< 0.60m) or within proximity in front (< 1.2m), adopt target!
                        if (dist_to_standoff < 0.60 || (dist_to_goal < 1.2 && dot_fwd_seek <= -0.20)) {
                            active_target_pose = stale_goal.pose;
                            has_active_target = true;
                            state = FlightState::FLIGHT_STATE_PATH_FOLLOWING;
                            goal_phase = 0;
                            align_ticks = 0;
                            standoff_ticks = 0;
                            seek_ticks = 0;
                            new_goal = true;
                            updateApproachDirection(stale_goal.pose);
                            Pose standoff_target = stale_goal.pose;
                            standoff_target.x = stale_goal.pose.x - approach_dx * 0.75;
                            standoff_target.y = stale_goal.pose.y - approach_dy * 0.75;
                            standoff_target.z = stale_goal.pose.z;
                            standoff_target.yaw = std::atan2(approach_dy, approach_dx);
                            cmd = planner.plan(current_pose, standoff_target, dt);
                            return cmd;
                        } else {
                            // Fly toward the standoff position in front of the gate entrance
                            double des_vx = standoff_x - current_pose.x;
                            double des_vy = standoff_y - current_pose.y;
                            double apf_vx = 0.0, apf_vy = 0.0, apf_off_x = 0.0, apf_off_y = 0.0;
                            computeNonTargetRepulsion(current_pose, target_shape, des_vx, des_vy,
                                                      apf_vx, apf_vy, apf_off_x, apf_off_y);

                            cmd.target_x = standoff_x + apf_off_x;
                            cmd.target_y = standoff_y + apf_off_y;
                            cmd.target_z = standoff_z;
                            // Orient toward the goal so the camera acquires it along the flight path
                            cmd.target_yaw = std::atan2(stale_goal.pose.y - current_pose.y,
                                                        stale_goal.pose.x - current_pose.x);
                        }
                    } else {
                        // Fallback: smooth scan sweep to acquire initial lock on the goal
                        double apf_vx = 0.0, apf_vy = 0.0, apf_off_x = 0.0, apf_off_y = 0.0;
                        computeNonTargetRepulsion(current_pose, target_shape, 0.0, 0.0,
                                                  apf_vx, apf_vy, apf_off_x, apf_off_y);

                        cmd.target_x = current_pose.x + apf_off_x;
                        cmd.target_y = current_pose.y + apf_off_y;
                        cmd.target_z = current_pose.z;
                        cmd.target_yaw = current_pose.yaw + 0.35 * dt;
                    }

                    planner.plan(current_pose, {cmd.target_x, cmd.target_y, cmd.target_z, 0, 0, cmd.target_yaw, 0, 1.0}, dt);
                    return cmd;
                }
            } else {
                if (configured_end_action == MISSION_END_LAND) {
                    state = FlightState::FLIGHT_STATE_LANDING;
                } else {
                    state = FlightState::FLIGHT_STATE_HOVER_WAIT;
                }
            }
            break;
        }

        case FlightState::FLIGHT_STATE_HOVER_WAIT:
            cmd.enable = true;
            cmd.target_x = current_pose.x; cmd.target_y = current_pose.y; cmd.target_z = current_pose.z; cmd.target_yaw = current_pose.yaw;
            planner.plan(current_pose, {cmd.target_x, cmd.target_y, cmd.target_z, 0, 0, cmd.target_yaw, 0, 1.0}, dt);
            break;

        case FlightState::FLIGHT_STATE_LANDING:
            cmd.enable = true;
            cmd.target_x = current_pose.x; cmd.target_y = current_pose.y; cmd.target_z = 0.0; cmd.target_yaw = current_pose.yaw;
            planner.plan(current_pose, {cmd.target_x, cmd.target_y, cmd.target_z, 0, 0, cmd.target_yaw, 0, 1.0}, dt);
            if (current_pose.z <= 0.05) {
                state = FlightState::FLIGHT_STATE_ROTOR_STOPPED;
                is_armed = 0;
            }
            break;

        case FlightState::FLIGHT_STATE_ROTOR_STOPPED:
        case FlightState::FLIGHT_STATE_STANDBY_DISARM:
        case FlightState::FLIGHT_STATE_ARMED:
        default:
            cmd.enable = false;
            cmd.estop = false;
            cmd.target_x = current_pose.x;
            cmd.target_y = current_pose.y;
            cmd.target_z = current_pose.z;
            cmd.target_yaw = current_pose.yaw;
            break;
    }

    return cmd;
}

void AutopilotNode::run() {
    auto last_time = std::chrono::steady_clock::now();
    while (running) {
        auto now = std::chrono::steady_clock::now();
        double dt = std::chrono::duration<double>(now - last_time).count();
        last_time = now;
        
        if (dt > 0.1) dt = 0.1;

        Telemetry telem = fcu.getTelemetry();
        FlightCommand cmd = tick(telem.current_pose, dt);
        fcu.sendFlightCommand(cmd);

        // Publish TelemetryPacket
        TelemetryPacket telem_pkt = {};
        telem_pkt.msg_type = MSG_TYPE_TELEMETRY;
        telem_pkt.sequence_id = telem_seq_id++;
        auto now_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
            now.time_since_epoch()).count();
        telem_pkt.timestamp_ns = now_ns;
        telem_pkt.drone_pos_ned = {(float)telem.current_pose.x, (float)telem.current_pose.y, (float)telem.current_pose.z};
        telem_pkt.target_pos_ned = {(float)cmd.target_x, (float)cmd.target_y, (float)cmd.target_z};
        telem_pkt.optical_flow_quality = 200.0f;
        telem_pkt.cpu_temperature_c = 45.0f;

        double current_time = std::chrono::duration<double>(now.time_since_epoch()).count();
        auto goals = estimator.getTelemetry(current_time);
        for (size_t i = 0; i < 3 && i < goals.size(); ++i) {
            telem_pkt.goals[i] = goals[i];
        }

        if (telem_sock >= 0) {
            sendto(telem_sock, &telem_pkt, sizeof(telem_pkt), 0, (struct sockaddr *)&telem_dest, sizeof(telem_dest));
        }

        logger.log("state", (int)state);
        logger.log("cmd_tx", cmd.target_x);
        logger.log("cmd_ty", cmd.target_y);
        logger.log("cmd_tz", cmd.target_z);
        logger.log("telem_x", telem.current_pose.x);
        logger.log("dt", dt);
        logger.flush();

        std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }
}
