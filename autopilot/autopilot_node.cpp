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
      logger("127.0.0.1", 14557),
      state_machine(),
      sequencer(),
      perception(),
      safety(2.5),
      telemetry_bridge(),
      active_sequence_len(sequencer.active_sequence_len),
      goal_sequence(sequencer.goal_sequence),
      current_seq_index(sequencer.current_seq_index),
      configured_end_action(sequencer.configured_end_action),
      hover_timeout_s(sequencer.hover_timeout_s),
      use_tube_constraint(sequencer.use_tube_constraint),
      tube_rail_y(sequencer.tube_rail_y),
      tube_rail_z(sequencer.tube_rail_z),
      new_goal(sequencer.new_goal),
      goal_phase(sequencer.goal_phase),
      align_ticks(sequencer.align_ticks),
      standoff_ticks(sequencer.standoff_ticks),
      seek_ticks(sequencer.seek_ticks),
      traverse_ticks(sequencer.traverse_ticks),
      approach_dx(sequencer.approach_dx),
      approach_dy(sequencer.approach_dy),
      approach_dz(sequencer.approach_dz),
      approach_sign_(sequencer.approach_sign_),
      last_approach_dx_(sequencer.last_approach_dx_),
      last_approach_dy_(sequencer.last_approach_dy_),
      active_target_pose(sequencer.active_target_pose),
      has_active_target(sequencer.has_active_target),
      active_orthogonal_path_(sequencer.active_orthogonal_path_),
      has_orthogonal_path_(sequencer.has_orthogonal_path_),
      last_completed_goal_pose_(sequencer.last_completed_goal_pose_),
      hover_pose_(sequencer.hover_pose_),
      has_hover_pose_(sequencer.has_hover_pose_),
      tags_received_count(perception.tags_received_count),
      tags_forwarded_count(perception.tags_forwarded_count),
      surveyed_tags_verified_count_(perception.surveyed_tags_verified_count_),
      last_tag_measurement(perception.last_tag_measurement),
      last_vision_time(safety.last_vision_time)
{
    loadCompiledSurveyedTags(*this);

    telemetry_bridge.openSockets(14552, 14553, "127.0.0.1", 14554);
    ui_sock = telemetry_bridge.getUiSocket();
    vision_sock = telemetry_bridge.getVisionSocket();
    telem_sock = telemetry_bridge.getTelemSocket();
    telem_dest = telemetry_bridge.getTelemDest();

    safety.recordVisionPacket();
}

AutopilotNode::~AutopilotNode() {
    stop();
}

void AutopilotNode::start() {
    running = true;
    safety.enableFcuWatchdog(true, 1.0);
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
    telemetry_bridge.closeSockets();
    ui_sock = -1;
    vision_sock = -1;
    telem_sock = -1;
}

void AutopilotNode::handleUICommand(const UICommandPacket& pkt, const struct sockaddr_in& from, socklen_t from_len) {
    std::lock_guard<std::mutex> lock(cmd_mutex);
    switch (pkt.command) {
        case UI_CMD_ARM:
            state_machine.handleArmCommand();
            is_armed = state_machine.isArmed() ? 1 : 0;
            estop_active = state_machine.isEstopActive() ? 1 : 0;
            state = state_machine.getState();
            break;
        case UI_CMD_DISARM: {
            bool in_air = (getCurrentPose().z > 0.4);
            if (in_air) {
                // Reject in-flight disarm unless land or estop
                break;
            }
            state_machine.handleDisarmCommand();
            is_armed = state_machine.isArmed() ? 1 : 0;
            estop_active = state_machine.isEstopActive() ? 1 : 0;
            state = state_machine.getState();
            if (!state_machine.isArmed()) {
                sequencer.reset();
                estimator.reset();
                planner.reset();
            }
            break;
        }
        case UI_CMD_START_MISSION: {
            Pose cur_p = getCurrentPose();
            bool in_air = (cur_p.z > 0.4);
            state_machine.handleStartMissionCommand(in_air);
            is_armed = 1;
            estop_active = 0;
            state = state_machine.getState();
            sequencer.startMission(pkt, cur_p);
            planner.reset();
            break;
        }
        case UI_CMD_SET_SEQUENCE: {
            Pose cur_p = getCurrentPose();
            sequencer.updateSequence(pkt, cur_p);
            if (is_armed && (state == FlightState::FLIGHT_STATE_HOVER_WAIT ||
                             state == FlightState::FLIGHT_STATE_PATH_FOLLOWING ||
                             state == FlightState::FLIGHT_STATE_GOAL_SEEK)) {
                state = (cur_p.z > 0.4) ? FlightState::FLIGHT_STATE_PATH_FOLLOWING : FlightState::FLIGHT_STATE_TAKEOFF;
                state_machine.setState(state);
            }
            planner.reset();
            break;
        }
        case UI_CMD_SET_CONSTRAINTS:
            sequencer.setConstraints(pkt.use_tube_constraint, pkt.tube_rail_y, pkt.tube_rail_z);
            break;
        case UI_CMD_PAUSE_MISSION:
            state_machine.handlePauseMissionCommand();
            state = state_machine.getState();
            break;
        case UI_CMD_RESUME_MISSION:
            state_machine.handleResumeMissionCommand();
            state = state_machine.getState();
            break;
        case UI_CMD_LAND_NOW:
            state_machine.handleLandCommand();
            state = state_machine.getState();
            break;
        case UI_CMD_ESTOP:
            state_machine.handleEstopCommand();
            estop_active = 1;
            state = state_machine.getState();
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
    status.flight_state = static_cast<uint8_t>(state_machine.getState());
    status.is_armed = is_armed;
    status.estop_active = estop_active;
    status.active_sequence_len = sequencer.active_sequence_len;
    status.current_seq_index = sequencer.current_seq_index;
    status.current_target_shape = sequencer.getCurrentTargetShape();
    status.configured_end_action = sequencer.configured_end_action;
    status.battery_voltage = 12.0f;
    status.battery_remaining = 90.0f;
    status.active_error_flags = 0;
    status.last_acked_cmd_id = pkt.command_id;
    status.cmd_ack_result = 0;

    telemetry_bridge.sendUIStatus(from, from_len, status);
}

void AutopilotNode::ui_loop() {
    char buffer[4096];
    while (running) {
        int sock = telemetry_bridge.getUiSocket();
        if (sock < 0) {
            std::this_thread::sleep_for(std::chrono::milliseconds(50));
            continue;
        }
        struct sockaddr_in from; socklen_t from_len = sizeof(from);
        int n = recvfrom(sock, buffer, sizeof(buffer), 0, (struct sockaddr *)&from, &from_len);
        if (n == sizeof(UICommandPacket)) {
            UICommandPacket* pkt = reinterpret_cast<UICommandPacket*>(buffer);
            if (pkt->msg_type == MSG_TYPE_UI_CMD) {
                handleUICommand(*pkt, from, from_len);
            }
        } else if (n < 0 && (errno != EAGAIN && errno != EWOULDBLOCK)) {
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
    }
}

void AutopilotNode::handleVisionPacket(const VisionPacket& pkt, double current_time) {
    Pose cur = getCurrentPose();
    if (cur.z == 0.0 && cur.x == 0.0 && cur.y == 0.0) {
        cur = fcu.getTelemetry().current_pose;
    }

    VisionPacket transformed_pkt = PerceptionCoordinator::transformVisionGoals(pkt, cur);
    estimator.updateEstimates(transformed_pkt, current_time);

    safety.recordVisionPacket();
    perception.processTagForwarding(pkt, current_time, fcu);
}

void AutopilotNode::vision_loop() {
    char buffer[2048];
    while (running) {
        int sock = telemetry_bridge.getVisionSocket();
        if (sock < 0) {
            std::this_thread::sleep_for(std::chrono::milliseconds(50));
            continue;
        }
        struct sockaddr_in from; socklen_t from_len = sizeof(from);
        int n = recvfrom(sock, buffer, sizeof(buffer), 0, (struct sockaddr *)&from, &from_len);
        if (n == sizeof(VisionPacket)) {
            VisionPacket* pkt = reinterpret_cast<VisionPacket*>(buffer);
            if (pkt->msg_type == MSG_TYPE_VISION) {
                double current_time = std::chrono::duration<double>(
                    std::chrono::steady_clock::now().time_since_epoch()).count();
                handleVisionPacket(*pkt, current_time);
            }
        } else if (n < 0 && (errno != EAGAIN && errno != EWOULDBLOCK)) {
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
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
    double& out_offset_y)
{
    std::vector<TrackedGoal> known_goals = estimator.getAllKnownGoals();
    std::vector<PathPlanner::Obstacle> obstacles;
    obstacles.reserve(known_goals.size());
    for (const auto& g : known_goals) {
        if (g.shape_type == target_shape) continue;
        obstacles.push_back({g.pose.x, g.pose.y, g.pose.z, g.shape_type});
    }

    planner.computeObstacleRepulsion(current_pose, obstacles, des_vx, des_vy,
                                     out_apf_vx, out_apf_vy, out_offset_x, out_offset_y);
}

void AutopilotNode::updateApproachDirection(const Pose& target_pose) {
    std::vector<TrackedGoal> known_goals = estimator.getAllKnownGoals();
    std::vector<PathPlanner::Obstacle> obstacles;
    obstacles.reserve(known_goals.size());
    uint8_t target_shape = sequencer.getCurrentTargetShape();
    for (const auto& g : known_goals) {
        if (g.shape_type == target_shape) continue;
        obstacles.push_back({g.pose.x, g.pose.y, g.pose.z, g.shape_type});
    }

    Pose ref_pose = getCurrentPose();
    active_orthogonal_path_ = planner.generateOrthogonalPath(target_pose, ref_pose, obstacles, 0.75);
    has_orthogonal_path_ = true;

    approach_dx = active_orthogonal_path_.normal_x;
    approach_dy = active_orthogonal_path_.normal_y;
    approach_dz = active_orthogonal_path_.normal_z;
    approach_sign_ = active_orthogonal_path_.traversal_sign;
}

FlightCommand AutopilotNode::tick(const Pose& current_pose, double dt) {
    updateTelemetry(current_pose);
    FlightCommand cmd = {};
    cmd.enable = false;
    cmd.estop = false;

    // Safety watchdog evaluation
    if (safety.shouldTripEstop(state, fcu.getTimeSinceLastTelemetry())) {
        state_machine.handleEstopCommand();
        estop_active = 1;
        state = state_machine.getState();
    }

    std::lock_guard<std::mutex> lock(cmd_mutex);

    switch(state) {
        case FlightState::FLIGHT_STATE_ESTOP_TRIPPED:
            return SafetySupervisor::createEstopCommand();

        case FlightState::FLIGHT_STATE_TAKEOFF: {
            cmd.enable = true;
            Pose takeoff_target = MissionStateMachine::computeTakeoffTarget(
                current_pose, sequencer.use_tube_constraint, sequencer.tube_rail_z);
            planner.plan(current_pose, takeoff_target, dt);
            cmd.target_x = takeoff_target.x;
            cmd.target_y = takeoff_target.y;
            cmd.target_z = takeoff_target.z;
            cmd.target_yaw = takeoff_target.yaw;
            if (planner.has_arrived(current_pose, takeoff_target)) {
                state_machine.setState(FlightState::FLIGHT_STATE_PATH_FOLLOWING);
                state = state_machine.getState();
                sequencer.new_goal = true;
                sequencer.last_completed_goal_pose_ = takeoff_target;
            }
            break;
        }

        case FlightState::FLIGHT_STATE_PATH_FOLLOWING:
        case FlightState::FLIGHT_STATE_GOAL_SEEK: {
            if (!sequencer.isSequenceComplete()) {
                uint8_t target_shape = sequencer.getCurrentTargetShape();
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
                    active_target_pose = estimated_pose;
                    has_active_target = true;
                } else if (has_active_target) {
                    estimated_pose = active_target_pose;
                    found = true;
                } else {
                    TrackedGoal stale_g;
                    if (estimator.getLastKnownEstimate(target_shape, stale_g)) {
                        active_target_pose = stale_g.pose;
                        has_active_target = true;
                        estimated_pose = active_target_pose;
                        found = true;
                    }
                }

                if (found) {
                    state_machine.setState(FlightState::FLIGHT_STATE_PATH_FOLLOWING);
                    state = state_machine.getState();
                    cmd.enable = true;
                    Pose raw_target = estimated_pose;
                    if (sequencer.use_tube_constraint) {
                        raw_target.y = sequencer.tube_rail_y;
                        if (sequencer.tube_rail_z != 0.0f) raw_target.z = std::abs(sequencer.tube_rail_z);
                        raw_target.pitch = 0.0;
                        raw_target.roll = 0.0;
                    }

                    if (sequencer.new_goal || !has_orthogonal_path_) {
                        sequencer.new_goal = false;
                        sequencer.goal_phase = 0;
                        sequencer.standoff_ticks = 0;
                        sequencer.align_ticks = 0;
                        sequencer.seek_ticks = 0;
                        sequencer.traverse_ticks = 0;
                        updateApproachDirection(raw_target);
                    } else {
                        std::vector<TrackedGoal> known_goals = estimator.getAllKnownGoals();
                        std::vector<PathPlanner::Obstacle> obstacles;
                        obstacles.reserve(known_goals.size());
                        for (const auto& g : known_goals) {
                            if (g.shape_type == target_shape) continue;
                            obstacles.push_back({g.pose.x, g.pose.y, g.pose.z, g.shape_type});
                        }
                        Pose ref_pose = current_pose;
                        active_orthogonal_path_ = planner.generateOrthogonalPath(
                            raw_target, ref_pose, obstacles, 0.75, active_orthogonal_path_.traversal_sign);
                        approach_dx = active_orthogonal_path_.normal_x;
                        approach_dy = active_orthogonal_path_.normal_y;
                        approach_dz = active_orthogonal_path_.normal_z;
                    }

                    if (sequencer.goal_phase == 0) {
                        sequencer.standoff_ticks++;
                        PathPlanner::StandoffPlan plan_res = planner.planStandoffTarget(
                            current_pose, raw_target, approach_dx, approach_dy);
                        Pose adjusted_target = plan_res.target_pose;

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

                        if (planner.isStandoffReached(current_pose, active_orthogonal_path_.entry_point, adjusted_target,
                                                      approach_dx, approach_dy, plan_res.dot_fwd, sequencer.standoff_ticks)) {
                            sequencer.goal_phase = 1;
                            sequencer.align_ticks = 0;
                        }
                    } else if (sequencer.goal_phase == 1) {
                        sequencer.align_ticks++;
                        Pose adjusted_target = active_orthogonal_path_.entry_point;

                        planner.plan(current_pose, adjusted_target, dt, 0.0, 0.0);
                        cmd.target_x = adjusted_target.x;
                        cmd.target_y = adjusted_target.y;
                        cmd.target_z = adjusted_target.z;
                        cmd.target_yaw = adjusted_target.yaw;

                        if (planner.isBoresightAligned(current_pose, raw_target, adjusted_target,
                                                       approach_dx, approach_dy, sequencer.align_ticks, approach_dz)) {
                            sequencer.goal_phase = 2;
                            sequencer.align_ticks = 0;
                            sequencer.traverse_ticks = 0;
                        }
                    } else {
                        sequencer.traverse_ticks++;
                        Pose step_target = planner.planOrthogonalTraversalTarget(current_pose, active_orthogonal_path_, 0.35);
                        cmd.target_x = step_target.x;
                        cmd.target_y = step_target.y;
                        cmd.target_z = step_target.z;
                        cmd.target_yaw = step_target.yaw;

                        planner.plan(current_pose, active_orthogonal_path_.exit_point, dt, 0.0, 0.0);

                        if (planner.isOrthogonalTraversalCompleted(current_pose, active_orthogonal_path_, sequencer.traverse_ticks)) {
                            sequencer.advanceSequence(raw_target);
                        }
                    }

                    if (sequencer.use_tube_constraint) {
                        cmd.target_y = sequencer.tube_rail_y;
                        if (sequencer.tube_rail_z != 0.0f) cmd.target_z = std::abs(sequencer.tube_rail_z);
                    }
                } else {
                    state_machine.setState(FlightState::FLIGHT_STATE_GOAL_SEEK);
                    state = state_machine.getState();
                    cmd.enable = true;
                    sequencer.seek_ticks++;

                    TrackedGoal stale_goal;
                    bool has_stale = estimator.getLastKnownEstimate(target_shape, stale_goal);

                    if (has_stale) {
                        if (sequencer.new_goal) {
                            updateApproachDirection(stale_goal.pose);
                            sequencer.new_goal = false;
                        }
                        PathPlanner::StandoffPlan seek_plan = planner.planStandoffTarget(
                            current_pose, stale_goal.pose, approach_dx, approach_dy);
                        Pose standoff_target = seek_plan.target_pose;

                        double dist_to_standoff = std::hypot(current_pose.x - standoff_target.x,
                                                             current_pose.y - standoff_target.y);
                        double dist_to_goal = std::hypot(current_pose.x - stale_goal.pose.x,
                                                         current_pose.y - stale_goal.pose.y);

                        if (dist_to_standoff < 0.60 || (dist_to_goal < 1.2 && seek_plan.dot_fwd <= -0.20)) {
                            active_target_pose = stale_goal.pose;
                            has_active_target = true;
                            state_machine.setState(FlightState::FLIGHT_STATE_PATH_FOLLOWING);
                            state = state_machine.getState();
                            sequencer.goal_phase = 0;
                            sequencer.align_ticks = 0;
                            sequencer.standoff_ticks = 0;
                            sequencer.seek_ticks = 0;
                            sequencer.traverse_ticks = 0;
                            sequencer.new_goal = true;
                            updateApproachDirection(stale_goal.pose);
                            Pose standoff_target_entry = planner.planStandoffTarget(
                                current_pose, stale_goal.pose, approach_dx, approach_dy).entrance_standoff;
                            cmd = planner.plan(current_pose, standoff_target_entry, dt);
                            return cmd;
                        } else {
                            double des_vx = standoff_target.x - current_pose.x;
                            double des_vy = standoff_target.y - current_pose.y;
                            double apf_vx = 0.0, apf_vy = 0.0, apf_off_x = 0.0, apf_off_y = 0.0;
                            computeNonTargetRepulsion(current_pose, target_shape, des_vx, des_vy,
                                                      apf_vx, apf_vy, apf_off_x, apf_off_y);

                            cmd.target_x = standoff_target.x + apf_off_x;
                            cmd.target_y = standoff_target.y + apf_off_y;
                            cmd.target_z = standoff_target.z;
                            cmd.target_yaw = std::atan2(stale_goal.pose.y - current_pose.y,
                                                        stale_goal.pose.x - current_pose.x);
                        }
                    } else {
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
                if (sequencer.configured_end_action == MISSION_END_LAND) {
                    state_machine.handleLandCommand();
                } else {
                    state_machine.handlePauseMissionCommand();
                    hover_pose_ = current_pose;
                    has_hover_pose_ = true;
                }
                state = state_machine.getState();
            }
            break;
        }

        case FlightState::FLIGHT_STATE_HOVER_WAIT: {
            cmd.enable = true;
            Pose hover_target = MissionStateMachine::computeHoverHold(current_pose, has_hover_pose_, hover_pose_);
            hover_pose_ = hover_target;
            has_hover_pose_ = true;
            cmd.target_x = hover_pose_.x;
            cmd.target_y = hover_pose_.y;
            cmd.target_z = hover_pose_.z;
            cmd.target_yaw = hover_pose_.yaw;
            planner.plan(current_pose, hover_pose_, dt);
            break;
        }

        case FlightState::FLIGHT_STATE_LANDING: {
            cmd.enable = true;
            bool touchdown = false;
            Pose land_target = MissionStateMachine::computeLandingStep(current_pose, dt, touchdown);
            cmd.target_x = land_target.x;
            cmd.target_y = land_target.y;
            cmd.target_z = land_target.z;
            cmd.target_yaw = land_target.yaw;
            planner.plan(current_pose, land_target, dt);
            if (touchdown) {
                state_machine.setState(FlightState::FLIGHT_STATE_ROTOR_STOPPED);
                state = state_machine.getState();
                is_armed = 0;
            }
            break;
        }

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

        telemetry_bridge.sendTelemetry(telem_pkt);

        logger.log("state", static_cast<int>(state));
        logger.log("cmd_tx", cmd.target_x);
        logger.log("cmd_ty", cmd.target_y);
        logger.log("cmd_tz", cmd.target_z);
        logger.log("telem_x", telem.current_pose.x);
        logger.log("dt", dt);
        logger.flush();

        std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }
}
