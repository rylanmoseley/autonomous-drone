#include "autopilot_node.h"
#include "generated_surveyed_tags.h"
#include <iostream>
#include <chrono>
#include <cmath>
#include <arpa/inet.h>
#include <unistd.h>
#include <cstring>

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
            if (state == FlightState::FLIGHT_STATE_STANDBY_DISARM || state == FlightState::FLIGHT_STATE_ROTOR_STOPPED) {
                state = FlightState::FLIGHT_STATE_ARMED;
            }
            break;
        case UI_CMD_DISARM:
            is_armed = 0;
            state = FlightState::FLIGHT_STATE_STANDBY_DISARM;
            break;
        case UI_CMD_START_MISSION:
            is_armed = 1;
            state = FlightState::FLIGHT_STATE_TAKEOFF;
            current_seq_index = 0;
            new_goal = true;
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
    estimator.updateEstimates(pkt, current_time);
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

FlightCommand AutopilotNode::tick(const Pose& current_pose, double dt) {
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
    if (time_since_vision > 0.2 && state != FlightState::FLIGHT_STATE_ESTOP_TRIPPED 
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
            cmd = planner.plan(current_pose, takeoff_target, dt);
            if (planner.has_arrived(current_pose, takeoff_target)) {
                state = FlightState::FLIGHT_STATE_PATH_FOLLOWING;
                new_goal = true;
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
                    state = FlightState::FLIGHT_STATE_PATH_FOLLOWING;
                    Pose raw_target = estimated_pose;
                    if (use_tube_constraint) {
                        raw_target.y = tube_rail_y;
                        if (tube_rail_z != 0.0f) raw_target.z = std::abs(tube_rail_z);
                    }
                    
                    double hoop_nx = cos(raw_target.yaw);
                    double hoop_ny = sin(raw_target.yaw);
                    if (new_goal) {
                        double dx = raw_target.x - current_pose.x;
                        double dy = raw_target.y - current_pose.y;
                        if (dx * hoop_nx + dy * hoop_ny > 0) { approach_dx = hoop_nx; approach_dy = hoop_ny; }
                        else { approach_dx = -hoop_nx; approach_dy = -hoop_ny; }
                        double dest_x = raw_target.x - approach_dx * 0.70;
                        double dest_y = raw_target.y - approach_dy * 0.70;
                        transit_yaw = atan2(dest_y - current_pose.y, dest_x - current_pose.x);
                        new_goal = false; goal_phase = 0;
                    }
                    Pose adjusted_target = raw_target;
                    
                    // APF obstacle avoidance from other detected shapes (No-Go Keep-Out Zones)
                    double apf_vx = 0.0, apf_vy = 0.0;
                    for (const auto& other_tg : ests) {
                        if (other_tg.shape_type == target_shape) continue;
                        double gx = other_tg.pose.x; double gy = other_tg.pose.y; double gz = other_tg.pose.z;
                        double dx = current_pose.x - gx; double dy = current_pose.y - gy; double dz = current_pose.z - gz;
                        double dist_3d = std::sqrt(dx*dx + dy*dy + dz*dz);
                        double dist_2d = std::sqrt(dx*dx + dy*dy);
                        if (dist_3d < 0.65) {
                            if (dist_2d < 0.001) { dx = 1.0; dy = 0.0; dist_2d = 1.0; }
                            double force = (0.65 - dist_3d) * 8.0;
                            apf_vx += (dx / dist_2d) * force;
                            apf_vy += (dy / dist_2d) * force;
                            apf_vx += -(dy / dist_2d) * force * 0.8; // Tangential deflection around obstacle cylinder
                            apf_vy += (dx / dist_2d) * force * 0.8;
                        }
                    }

                    if (goal_phase == 0) {
                        adjusted_target.x = raw_target.x - approach_dx * 0.70;
                        adjusted_target.y = raw_target.y - approach_dy * 0.70;
                        adjusted_target.yaw = transit_yaw;
                        cmd = planner.plan(current_pose, adjusted_target, dt, apf_vx, apf_vy);
                        if (planner.has_arrived(current_pose, adjusted_target)) goal_phase = 1;
                    } else if (goal_phase == 1) {
                        adjusted_target.x = raw_target.x - approach_dx * 0.70;
                        adjusted_target.y = raw_target.y - approach_dy * 0.70;
                        adjusted_target.yaw = atan2(approach_dy, approach_dx);
                        cmd = planner.plan(current_pose, adjusted_target, dt, apf_vx, apf_vy);
                        double dyaw = adjusted_target.yaw - current_pose.yaw;
                        while (dyaw > M_PI) dyaw -= 2.0 * M_PI;
                        while (dyaw < -M_PI) dyaw += 2.0 * M_PI;
                        if (std::abs(dyaw) < 0.087) goal_phase = 2;
                    } else {
                        adjusted_target.x = raw_target.x + approach_dx * 0.70;
                        adjusted_target.y = raw_target.y + approach_dy * 0.70;
                        adjusted_target.yaw = atan2(approach_dy, approach_dx);
                        cmd = planner.plan(current_pose, adjusted_target, dt, apf_vx, apf_vy);
                        if (planner.has_arrived(current_pose, adjusted_target)) {
                            current_seq_index++;
                            new_goal = true;
                        }
                    }
                } else {
                    // Search behavior: hover and spin to find the goal
                    state = FlightState::FLIGHT_STATE_GOAL_SEEK;
                    cmd.enable = true;
                    cmd.target_x = current_pose.x;
                    cmd.target_y = current_pose.y;
                    cmd.target_z = current_pose.z;
                    cmd.target_yaw = current_pose.yaw + 1.0;
                    cmd = planner.plan(current_pose, {cmd.target_x, cmd.target_y, cmd.target_z, 0, 0, cmd.target_yaw, 0, 1.0}, dt);
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
            cmd = planner.plan(current_pose, {cmd.target_x, cmd.target_y, cmd.target_z, 0, 0, cmd.target_yaw, 0, 1.0}, dt);
            break;

        case FlightState::FLIGHT_STATE_LANDING:
            cmd.enable = true;
            cmd.target_x = current_pose.x; cmd.target_y = current_pose.y; cmd.target_z = 0.0; cmd.target_yaw = current_pose.yaw;
            cmd = planner.plan(current_pose, {cmd.target_x, cmd.target_y, cmd.target_z, 0, 0, cmd.target_yaw, 0, 1.0}, dt);
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
