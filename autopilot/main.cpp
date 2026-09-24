#include <iostream>
#include <thread>
#include <chrono>
#include <atomic>
#include <mutex>
#include <sys/socket.h>
#include <netinet/in.h>
#include <unistd.h>
#include <cstring>
#include <cmath>
#include "flight_control_interface.h"
#include "ui_protocol.h"
#include "vision_packet.h"
#include "goal_estimator.h"
#include "path_planner.h"
#include "mavlink_daemon.h"
#include "telemetry_logger_impl.h"

enum class MissionState {
    IDLE,
    TAKEOFF,
    NAVIGATING,
    RETURN_TO_HOME,
    LANDING,
    ERROR,
    ESTOP
};

std::atomic<bool> running{true};
UIGoalSequence current_goal_sequence;
std::mutex sequence_mutex;

std::atomic<double> last_vision_time{0.0};

void ui_receiver_thread() {
    int sock = socket(AF_INET, SOCK_DGRAM, 0);
    struct sockaddr_in addr;
    addr.sin_family = AF_INET;
    addr.sin_port = htons(14552);
    addr.sin_addr.s_addr = INADDR_ANY;
    bind(sock, (struct sockaddr*)&addr, sizeof(addr));
    
    struct timeval tv;
    tv.tv_sec = 1; tv.tv_usec = 0;
    setsockopt(sock, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));
    
    char buffer[2048];
    while(running) {
        struct sockaddr_in src_addr;
        socklen_t src_len = sizeof(src_addr);
        int n = recvfrom(sock, buffer, sizeof(buffer), 0, (struct sockaddr*)&src_addr, &src_len);
        std::cout << "Received UDP UI packet of size " << n << std::endl; if (n == sizeof(UIGoalSequence)) {
            {
                std::lock_guard<std::mutex> lock(sequence_mutex);
                memcpy(&current_goal_sequence, buffer, sizeof(UIGoalSequence)); std::cout << "seq.start: " << current_goal_sequence.start << " seq.num_goals: " << current_goal_sequence.num_goals << std::endl;
            }
            
            UIAckPacket ack;
            ack.received = true;
            sendto(sock, &ack, sizeof(UIAckPacket), 0, (struct sockaddr*)&src_addr, src_len);
        }
    }
    close(sock);
}

void vision_receiver_thread(GoalEstimator* estimator) {
    int sock = socket(AF_INET, SOCK_DGRAM, 0);
    struct sockaddr_in addr;
    addr.sin_family = AF_INET;
    addr.sin_port = htons(14553);
    addr.sin_addr.s_addr = INADDR_ANY;
    bind(sock, (struct sockaddr*)&addr, sizeof(addr));
    
    struct timeval tv;
    tv.tv_sec = 1; tv.tv_usec = 0;
    setsockopt(sock, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));
    
    char buffer[1024];
    while(running) {
        int n = recv(sock, buffer, sizeof(buffer), 0);
        if (n > 0) std::cout << "Vision n: " << n << std::endl;
        if (n == sizeof(VisionGoalEstimate)) {
            VisionGoalEstimate est;
            memcpy(&est, buffer, sizeof(VisionGoalEstimate));
            std::vector<VisionGoalEstimate> updates = {est};
            auto now = std::chrono::steady_clock::now().time_since_epoch();
            double t = std::chrono::duration<double>(now).count();
            last_vision_time = t;
            estimator->updateEstimates(updates, t);
        }
    }
    close(sock);
}

int AutopilotNode::run_once() {
    PlannerConfig config;
    config.max_velocity = 0.6; // ER 3 requires >= 0.6 m/s, reduced for 2x2x2 space
    config.max_vertical_velocity = 0.25; 
    config.max_acceleration = 0.5;
    config.max_yaw_rate = 0.5;
    config.position_p_gain = 1.5;
    config.yaw_p_gain = 1.0;
    config.arrival_tolerance = 0.1; // 10cm tolerance
    
    PathPlanner planner(config);
    GoalEstimator estimator(0.5, 5.0);
    MavlinkDaemon fcu(14550, "127.0.0.1", 14551);
    UdpTelemetryLogger logger("127.0.0.1", 14554);
    
    fcu.start();
    
    std::thread ui_thread(ui_receiver_thread);
    std::thread vision_thread(vision_receiver_thread, &estimator);
    
    MissionState state = MissionState::IDLE;
    size_t current_goal_index = 0;
    
    bool new_goal = true;
    int goal_phase = 0;
    double approach_dx = 0.0, approach_dy = 0.0, approach_dz = 0.0;
    double transit_yaw = 0.0;
    
    std::cout << "Autopilot started." << std::endl;
    
    auto start_time_tp = std::chrono::steady_clock::now();
    auto last_time = start_time_tp;
    last_vision_time = std::chrono::duration<double>(start_time_tp.time_since_epoch()).count();
    
    while(running) {
        auto current_time_tp = std::chrono::steady_clock::now();
        double dt = std::chrono::duration<double>(current_time_tp - last_time).count();
        last_time = current_time_tp;
        double current_time_sec = std::chrono::duration<double>(current_time_tp.time_since_epoch()).count();

        Telemetry telem = fcu.getTelemetry();
        auto tracked_goals = estimator.getCurrentEstimates(current_time_sec);
        
        UIGoalSequence seq;
        {
            std::lock_guard<std::mutex> lock(sequence_mutex);
            seq = current_goal_sequence;
        }
        
        bool sensor_disconnected = (current_time_sec - last_vision_time > 0.200);
        
        if (seq.estop) {
            state = MissionState::ESTOP;
        } else if (telem.error_state || (sensor_disconnected && state != MissionState::IDLE)) {
            if (state != MissionState::ERROR) {
                std::cout << "Entering ERROR. telem.error_state=" << telem.error_state << " disconnected=" << sensor_disconnected << " time_diff=" << (current_time_sec - last_vision_time) << std::endl;
            }
            state = MissionState::ERROR;
        } else if (seq.start && state == MissionState::IDLE) {
            state = MissionState::TAKEOFF;
            current_goal_index = 0;
            new_goal = true;
        } else if (seq.stop && state != MissionState::IDLE) {
            state = MissionState::LANDING;
        }
        
        FlightCommand cmd = {0};
        cmd.enable = false;
        cmd.estop = false;
        
        switch(state) {
            case MissionState::ESTOP:
                cmd.estop = true;
                break;
            case MissionState::ERROR:
                cmd.estop = true; 
                break;
            case MissionState::TAKEOFF:
                cmd.enable = true;
                cmd.target_x = telem.current_pose.x;
                cmd.target_y = telem.current_pose.y;
                cmd.target_z = 1.0;
                cmd.target_yaw = telem.current_pose.yaw;
                if (planner.has_arrived(telem.current_pose, {cmd.target_x, cmd.target_y, cmd.target_z, 0,0,0,0,1.0})) {
                    std::cout << "Switching to NAVIGATING" << std::endl; state = MissionState::NAVIGATING;
                    new_goal = true;
                }
                break;
            case MissionState::NAVIGATING:
                if (current_goal_index < seq.num_goals) {
                    Pose raw_target = {
                        seq.goals[current_goal_index].x,
                        seq.goals[current_goal_index].y,
                        seq.goals[current_goal_index].z,
                        seq.goals[current_goal_index].roll,
                        seq.goals[current_goal_index].pitch,
                        seq.goals[current_goal_index].yaw,
                        seq.goals[current_goal_index].timestamp,
                        seq.goals[current_goal_index].confidence
                    };
                    
                    double hoop_nx = cos(raw_target.yaw);
                    double hoop_ny = sin(raw_target.yaw);
                    
                    if (new_goal) {
                        double dx = raw_target.x - telem.current_pose.x;
                        double dy = raw_target.y - telem.current_pose.y;
                        if (dx * hoop_nx + dy * hoop_ny > 0) {
                            approach_dx = hoop_nx;
                            approach_dy = hoop_ny;
                        } else {
                            approach_dx = -hoop_nx;
                            approach_dy = -hoop_ny;
                        }
                        approach_dz = 0.0;
                        
                        double dest_x = raw_target.x - approach_dx * 0.5;
                        double dest_y = raw_target.y - approach_dy * 0.5;
                        transit_yaw = atan2(dest_y - telem.current_pose.y, dest_x - telem.current_pose.x);
                        
                        new_goal = false;
                        goal_phase = 0;
                    }
                    
                    Pose adjusted_target = raw_target;
                    
                    if (goal_phase == 0) {
                        adjusted_target.x = raw_target.x - approach_dx * 0.5;
                        adjusted_target.y = raw_target.y - approach_dy * 0.5;
                        // Face stable flight path calculated at the start of the leg
                        adjusted_target.yaw = transit_yaw;
                        
                        std::cout << "Calling plan..." << std::endl; cmd = planner.plan(telem.current_pose, adjusted_target, dt);
                        std::cout << "Phase 0. telem_x: " << telem.current_pose.x << " dest_x: " << adjusted_target.x << " cmd_tx: " << cmd.target_x << std::endl;
                        
                        if (planner.has_arrived(telem.current_pose, adjusted_target)) {
                            goal_phase = 1;
                        }
                    } else if (goal_phase == 1) {
                        adjusted_target.x = raw_target.x - approach_dx * 0.5;
                        adjusted_target.y = raw_target.y - approach_dy * 0.5;
                        
                        // Turn to face the hoop
                        adjusted_target.yaw = atan2(approach_dy, approach_dx);
                        
                        std::cout << "Planning. Phase: " << goal_phase << " tx: " << adjusted_target.x << " current_x: " << telem.current_pose.x << std::endl; std::cout << "Calling plan..." << std::endl; cmd = planner.plan(telem.current_pose, adjusted_target, dt);
                        
                        double dyaw = adjusted_target.yaw - telem.current_pose.yaw;
                        while (dyaw > M_PI) dyaw -= 2.0 * M_PI;
                        while (dyaw < -M_PI) dyaw += 2.0 * M_PI;
                        
                        // Only advance when yaw is perfectly aligned (< 5 degrees)
                        if (std::abs(dyaw) < 0.087) {
                            goal_phase = 2;
                        }
                    } else {
                        adjusted_target.x = raw_target.x + approach_dx * 0.5;
                        adjusted_target.y = raw_target.y + approach_dy * 0.5;
                        
                        // Keep facing the hoop
                        adjusted_target.yaw = atan2(approach_dy, approach_dx);
                        
                        std::cout << "Planning. Phase: " << goal_phase << " tx: " << adjusted_target.x << " current_x: " << telem.current_pose.x << std::endl; std::cout << "Calling plan..." << std::endl; cmd = planner.plan(telem.current_pose, adjusted_target, dt);
                        if (planner.has_arrived(telem.current_pose, adjusted_target)) {
                            current_goal_index++;
                            new_goal = true;
                        }
                    }
                } else {
                    state = MissionState::RETURN_TO_HOME;
                }
                break;
            case MissionState::RETURN_TO_HOME:
                cmd.enable = true;
                cmd.target_x = 0.0;
                cmd.target_y = 0.0;
                cmd.target_z = 1.0;
                cmd.target_yaw = telem.current_pose.yaw;
                {
                    double rth_dx = 0.0 - telem.current_pose.x;
                    double rth_dy = 0.0 - telem.current_pose.y;
                    if (std::sqrt(rth_dx*rth_dx + rth_dy*rth_dy) > 0.5) {
                        cmd.target_yaw = std::atan2(rth_dy, rth_dx);
                    } else {
                        cmd.target_yaw = telem.current_pose.yaw; // Stop updating yaw when close
                    }
                }
                cmd = planner.plan(telem.current_pose, {cmd.target_x, cmd.target_y, cmd.target_z, 0,0,cmd.target_yaw,0,1.0}, dt);
                if (planner.has_arrived(telem.current_pose, {0,0,1,0,0,0,0,1})) {
                    state = MissionState::LANDING;
                }
                break;
            case MissionState::LANDING:
                cmd.enable = true;
                cmd.target_x = telem.current_pose.x;
                cmd.target_y = telem.current_pose.y;
                cmd.target_z = 0.0;
                cmd = planner.plan(telem.current_pose, {cmd.target_x, cmd.target_y, cmd.target_z, 0,0,telem.current_pose.yaw,0,1.0}, dt);
                if (telem.current_pose.z <= 0.05) {
                    state = MissionState::IDLE;
                }
                break;
            case MissionState::IDLE:
            default:
                break;
        }
        
        fcu.sendFlightCommand(cmd);
        logger.log("state", (int)state);
        logger.log("cmd_tx", cmd.target_x);
        logger.log("cmd_ty", cmd.target_y);
        logger.log("cmd_tz", cmd.target_z);
        logger.log("telem_x", telem.current_pose.x);
        logger.flush();
        std::cout << "State: " << (int)state << " current_z: " << telem.current_pose.z << std::endl; std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }
    
    fcu.stop();
    ui_thread.join();
    vision_thread.join();
    return 0;
}
