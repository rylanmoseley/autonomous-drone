#include "autopilot_node.h"
#include <iostream>
#include <chrono>
#include <cmath>

AutopilotNode::AutopilotNode()
    : fcu(14550, "127.0.0.1", 14551),
      planner([](){
          PlannerConfig cfg = {};
          cfg.max_velocity = 2.0;
          cfg.max_vertical_velocity = 1.0;
          cfg.max_acceleration = 100.0; // High for test/sim
          cfg.position_p_gain = 1.0;
          cfg.max_yaw_rate = 1.0;
          cfg.yaw_p_gain = 1.0;
          cfg.arrival_tolerance = 0.2;
          return cfg;
      }()),
      estimator(0.5, 2.0),
      logger("127.0.0.1", 14554)
{
    ui_sock = socket(AF_INET, SOCK_DGRAM, 0);
    struct sockaddr_in ui_addr;
    ui_addr.sin_family = AF_INET;
    ui_addr.sin_port = htons(14552);
    ui_addr.sin_addr.s_addr = INADDR_ANY;
    bind(ui_sock, (struct sockaddr *)&ui_addr, sizeof(ui_addr));

    struct timeval tv;
    tv.tv_sec = 0; tv.tv_usec = 100000;
    setsockopt(ui_sock, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));

    vision_sock = socket(AF_INET, SOCK_DGRAM, 0);
    struct sockaddr_in vision_addr;
    vision_addr.sin_family = AF_INET;
    vision_addr.sin_port = htons(14553);
    vision_addr.sin_addr.s_addr = INADDR_ANY;
    bind(vision_sock, (struct sockaddr *)&vision_addr, sizeof(vision_addr));
    setsockopt(vision_sock, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));
    
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
    if(loop_thread.joinable()) loop_thread.join();
    if(ui_thread.joinable()) ui_thread.join();
    if(vision_thread.joinable()) vision_thread.join();
    close(ui_sock);
    close(vision_sock);
}

void AutopilotNode::ui_loop() {
    char buffer[4096];
    while (running) {
        struct sockaddr_in from; socklen_t from_len = sizeof(from);
        int n = recvfrom(ui_sock, buffer, sizeof(buffer), 0, (struct sockaddr *)&from, &from_len);
        if (n == sizeof(UIGoalSequence)) {
            UIGoalSequence* packet = reinterpret_cast<UIGoalSequence*>(buffer);
            std::lock_guard<std::mutex> lock(seq_mutex);
            seq = *packet;
            
            UIAckPacket ack;
            ack.received = true;
            sendto(ui_sock, &ack, sizeof(ack), 0, (struct sockaddr *)&from, from_len);
        } else if (n == sizeof(UIConfigPacket)) {
            UIConfigPacket* cfg = reinterpret_cast<UIConfigPacket*>(buffer);
            PlannerConfig pcfg = planner.getConfig();
            pcfg.max_velocity = cfg->max_velocity;
            pcfg.max_vertical_velocity = cfg->max_vertical_velocity;
            pcfg.max_yaw_rate = cfg->max_yaw_rate;
            planner.updateConfig(pcfg);
        }
    }
}

void AutopilotNode::vision_loop() {
    char buffer[1024];
    while (running) {
        struct sockaddr_in from; socklen_t from_len = sizeof(from);
        int n = recvfrom(vision_sock, buffer, sizeof(buffer), 0, (struct sockaddr *)&from, &from_len);
        if (n == sizeof(VisionGoalEstimate)) {
            VisionGoalEstimate* est = reinterpret_cast<VisionGoalEstimate*>(buffer);
            std::vector<VisionGoalEstimate> vec = {*est};
            double current_time = std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch()).count();
            estimator.updateEstimates(vec, current_time);
            {
                std::lock_guard<std::mutex> lock(vision_mutex);
                last_vision_time = std::chrono::steady_clock::now();
            }
        }
    }
}

FlightCommand AutopilotNode::tick(const Pose& current_pose, double dt) {
    FlightCommand cmd = {0};
    cmd.enable = false;
    cmd.estop = false;
    
    // Safety
    auto now = std::chrono::steady_clock::now();
    double time_since_vision = 0;
    {
        std::lock_guard<std::mutex> lock(vision_mutex);
        time_since_vision = std::chrono::duration<double>(now - last_vision_time).count();
    }
    if (time_since_vision > 0.2 && state != MissionState::ESTOP) {
        state = MissionState::ERROR;
    }

    UIGoalSequence current_seq;
    {
        std::lock_guard<std::mutex> lock(seq_mutex);
        current_seq = seq;
        if (seq.estop) state = MissionState::ESTOP;
        else if (seq.start && state == MissionState::IDLE) {
            seq.start = false;
            state = MissionState::TAKEOFF;
            current_goal_index = 0;
            new_goal = true;
        } else if (seq.stop && state != MissionState::IDLE) {
            state = MissionState::LANDING;
            seq.stop = false;
        }
    }

    switch(state) {
        case MissionState::ESTOP: cmd.estop = true; break;
        case MissionState::ERROR: cmd.estop = true; break;
        case MissionState::TAKEOFF:
            cmd.enable = true;
            cmd.target_x = current_pose.x;
            cmd.target_y = current_pose.y;
            cmd.target_z = 1.0;
            cmd.target_yaw = current_pose.yaw;
            cmd = planner.plan(current_pose, {cmd.target_x, cmd.target_y, cmd.target_z, 0,0,cmd.target_yaw,0,1.0}, dt);
            if (planner.has_arrived(current_pose, {current_pose.x, current_pose.y, 1.0, 0,0,current_pose.yaw,0,1.0})) {
                state = MissionState::NAVIGATING;
                new_goal = true;
            }
            break;
        case MissionState::NAVIGATING:
            if (current_goal_index < current_seq.num_goals) {
                Pose raw_target = {
                    current_seq.goals[current_goal_index].x,
                    current_seq.goals[current_goal_index].y,
                    current_seq.goals[current_goal_index].z,
                    current_seq.goals[current_goal_index].roll,
                    current_seq.goals[current_goal_index].pitch,
                    current_seq.goals[current_goal_index].yaw,
                    current_seq.goals[current_goal_index].timestamp,
                    current_seq.goals[current_goal_index].confidence
                };
                
                double current_time = std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch()).count();
                std::vector<TrackedGoal> ests = estimator.getCurrentEstimates(current_time);
                bool found = false;
                VisionPose estimated_pose;
                for (const auto& tg : ests) {
                    if (tg.goal_id == current_seq.goals[current_goal_index].id) {
                        estimated_pose = tg.pose;
                        found = true;
                        break;
                    }
                }
                
                if (found) {
                    raw_target.x = estimated_pose.x;
                    raw_target.y = estimated_pose.y;
                    raw_target.z = estimated_pose.z;
                    raw_target.yaw = estimated_pose.yaw;
                } else {
                    // Search behavior: hover and spin to find the goal
                    cmd.enable = true;
                    cmd.target_x = current_pose.x;
                    cmd.target_y = current_pose.y;
                    cmd.target_z = current_pose.z;
                    cmd.target_yaw = current_pose.yaw + 1.0; // Pass a large yaw delta so planner spins
                    cmd = planner.plan(current_pose, {cmd.target_x, cmd.target_y, cmd.target_z, 0,0,cmd.target_yaw,0,1.0}, dt);
                    
                    // Return early so we don't run regular navigation or APF
                    return cmd;
                }
                
                double hoop_nx = cos(raw_target.yaw);
                double hoop_ny = sin(raw_target.yaw);
                if (new_goal) {
                    double dx = raw_target.x - current_pose.x;
                    double dy = raw_target.y - current_pose.y;
                    if (dx * hoop_nx + dy * hoop_ny > 0) { approach_dx = hoop_nx; approach_dy = hoop_ny; }
                    else { approach_dx = -hoop_nx; approach_dy = -hoop_ny; }
                    double dest_x = raw_target.x - approach_dx * 0.5;
                    double dest_y = raw_target.y - approach_dy * 0.5;
                    transit_yaw = atan2(dest_y - current_pose.y, dest_x - current_pose.x);
                    new_goal = false; goal_phase = 0;
                }
                Pose adjusted_target = raw_target;
                
                double apf_vx = 0.0, apf_vy = 0.0;
                for (size_t i = 0; i < current_seq.num_goals; ++i) {
                    if (i == current_goal_index) continue;
                    double gx = current_seq.goals[i].x; double gy = current_seq.goals[i].y; double gz = current_seq.goals[i].z;
                    double dx = current_pose.x - gx; double dy = current_pose.y - gy; double dz = current_pose.z - gz;
                    double dist_3d = std::sqrt(dx*dx + dy*dy + dz*dz);
                    double dist_2d = std::sqrt(dx*dx + dy*dy);
                    if (dist_3d < 0.4) {
                        if (dist_2d < 0.001) { dx = 1.0; dy = 0.0; dist_2d = 1.0; }
                        double force = (0.4 - dist_3d) * 5.0; // P_gain for APF
                        apf_vx += (dx / dist_2d) * force;
                        apf_vy += (dy / dist_2d) * force;
                        apf_vx += -(dy / dist_2d) * force * 0.5; // vortex
                        apf_vy += (dx / dist_2d) * force * 0.5;
                    }
                }

                if (goal_phase == 0) {
                    adjusted_target.x = raw_target.x - approach_dx * 0.5;
                    adjusted_target.y = raw_target.y - approach_dy * 0.5;
                    adjusted_target.yaw = transit_yaw;
                    cmd = planner.plan(current_pose, adjusted_target, dt, apf_vx, apf_vy);
                    if (planner.has_arrived(current_pose, adjusted_target)) goal_phase = 1;
                } else if (goal_phase == 1) {
                    adjusted_target.x = raw_target.x - approach_dx * 0.5;
                    adjusted_target.y = raw_target.y - approach_dy * 0.5;
                    adjusted_target.yaw = atan2(approach_dy, approach_dx);
                    cmd = planner.plan(current_pose, adjusted_target, dt, apf_vx, apf_vy);
                    double dyaw = adjusted_target.yaw - current_pose.yaw;
                    while (dyaw > M_PI) dyaw -= 2.0 * M_PI;
                    while (dyaw < -M_PI) dyaw += 2.0 * M_PI;
                    if (std::abs(dyaw) < 0.087) goal_phase = 2;
                } else {
                    adjusted_target.x = raw_target.x + approach_dx * 0.5;
                    adjusted_target.y = raw_target.y + approach_dy * 0.5;
                    adjusted_target.yaw = atan2(approach_dy, approach_dx);
                    cmd = planner.plan(current_pose, adjusted_target, dt, apf_vx, apf_vy);
                    if (planner.has_arrived(current_pose, adjusted_target)) {
                        current_goal_index++;
                        new_goal = true;
                    }
                }
            } else { state = MissionState::RETURN_TO_HOME; }
            break;
        case MissionState::RETURN_TO_HOME:
            cmd.enable = true;
            cmd.target_x = 0.0; cmd.target_y = 0.0; cmd.target_z = 0.4; cmd.target_yaw = current_pose.yaw;
            {
                double rth_dx = 0.0 - current_pose.x; double rth_dy = 0.0 - current_pose.y;
                if (std::sqrt(rth_dx*rth_dx + rth_dy*rth_dy) > 0.5) cmd.target_yaw = std::atan2(rth_dy, rth_dx);
            }
            cmd = planner.plan(current_pose, {cmd.target_x, cmd.target_y, cmd.target_z, 0,0,cmd.target_yaw,0,1.0}, dt);
            if (planner.has_arrived(current_pose, {0,0,0.4,0,0,0,0,1})) state = MissionState::LANDING;
            break;
        case MissionState::LANDING:
            cmd.enable = true;
            cmd.target_x = current_pose.x; cmd.target_y = current_pose.y; cmd.target_z = 0.0;
            cmd = planner.plan(current_pose, {cmd.target_x, cmd.target_y, cmd.target_z, 0,0,current_pose.yaw,0,1.0}, dt);
            if (current_pose.z <= 0.05) state = MissionState::IDLE;
            break;
        case MissionState::IDLE:
        default: break;
    }


    return cmd;
}

void AutopilotNode::run() {
    auto last_time = std::chrono::steady_clock::now();
    while (running) {
        auto now = std::chrono::steady_clock::now();
        double dt = std::chrono::duration<double>(now - last_time).count();
        last_time = now;
        
        // Prevent huge dt if thread starves
        if (dt > 0.1) dt = 0.1;

        Telemetry telem = fcu.getTelemetry();
        FlightCommand cmd = tick(telem.current_pose, dt);
        
        fcu.sendFlightCommand(cmd);
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
