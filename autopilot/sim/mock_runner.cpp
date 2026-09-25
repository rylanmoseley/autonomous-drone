#include <iostream>
#include <thread>
#include <chrono>
#include <atomic>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <cstring>
#include "flight_control_interface.h"
#include "ui_protocol.h"
#include "vision_packet.h"

std::atomic<bool> running{true};

void mock_ui() {
    int sock = socket(AF_INET, SOCK_DGRAM, 0);
    struct sockaddr_in addr;
    addr.sin_family = AF_INET;
    addr.sin_port = htons(14552);
    inet_pton(AF_INET, "127.0.0.1", &addr.sin_addr);
    
    struct timeval tv;
    tv.tv_sec = 2; tv.tv_usec = 0;
    setsockopt(sock, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));

    std::this_thread::sleep_for(std::chrono::seconds(1));

    UIGoalSequence seq;
    seq.start = true;
    seq.stop = false;
    seq.estop = false;
    seq.goals[0] = {1, 1.0, 0.0, 1.0, 0, 0, 0, 0, 1.0};
    seq.goals[1] = {2, 1.0, 1.0, 1.0, 0, 0, 0, 0, 1.0};
    seq.num_goals = 2;

    sendto(sock, &seq, sizeof(UIGoalSequence), 0, (struct sockaddr*)&addr, sizeof(addr));
    std::cout << "[Mock UI] Sent start command and goal sequence." << std::endl;
    
    UIAckPacket ack;
    struct sockaddr_in src_addr;
    socklen_t src_len = sizeof(src_addr);
    int n = recvfrom(sock, &ack, sizeof(UIAckPacket), 0, (struct sockaddr*)&src_addr, &src_len);
    if (n == sizeof(UIAckPacket) && ack.received) {
        std::cout << "[Mock UI] Received ACK!" << std::endl;
    } else {
        std::cout << "[Mock UI] Failed to receive ACK." << std::endl;
    }
    
    close(sock);
}

void mock_vision() {
    int sock = socket(AF_INET, SOCK_DGRAM, 0);
    struct sockaddr_in addr;
    addr.sin_family = AF_INET;
    addr.sin_port = htons(14553);
    inet_pton(AF_INET, "127.0.0.1", &addr.sin_addr);

    while(running) {
        VisionGoalEstimate est;
        est.goal_id = 1;
        est.determinate = true;
        est.pose = {1.0, 0.0, 1.0, 0, 0, 0, 0, 1.0};
        sendto(sock, &est, sizeof(VisionGoalEstimate), 0, (struct sockaddr*)&addr, sizeof(addr));
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
    close(sock);
}

void mock_fcu() {
    int sock = socket(AF_INET, SOCK_DGRAM, 0);
    struct sockaddr_in addr;
    addr.sin_family = AF_INET;
    addr.sin_port = htons(14551);
    addr.sin_addr.s_addr = INADDR_ANY;
    bind(sock, (struct sockaddr*)&addr, sizeof(addr));

    struct timeval tv;
    tv.tv_sec = 0; tv.tv_usec = 100000;
    setsockopt(sock, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));

    struct sockaddr_in remote_addr;
    remote_addr.sin_family = AF_INET;
    remote_addr.sin_port = htons(14550);
    inet_pton(AF_INET, "127.0.0.1", &remote_addr.sin_addr);

    Pose drone_pose = {0,0,0,0,0,0,0,1};
    
    while(running) {
        FlightCommand cmd;
        struct sockaddr_in src_addr;
        socklen_t src_len = sizeof(src_addr);
        int n = recvfrom(sock, &cmd, sizeof(FlightCommand), 0, (struct sockaddr*)&src_addr, &src_len);
        
        if (n == sizeof(FlightCommand)) {
            if (cmd.enable) {
                drone_pose.x = cmd.target_x;
                drone_pose.y = cmd.target_y;
                drone_pose.z = cmd.target_z;
                std::cout << "[Mock FCU] Drone moved to (" << drone_pose.x << ", " << drone_pose.y << ", " << drone_pose.z << ")" << std::endl;
            }
            if (cmd.estop) {
                std::cout << "[Mock FCU] ESTOP / MOTOR CUT received!" << std::endl;
            }
        }

        Telemetry telem;
        telem.current_pose = drone_pose;
        telem.battery_level = 90;
        telem.error_state = false;
        sendto(sock, &telem, sizeof(Telemetry), 0, (struct sockaddr*)&remote_addr, sizeof(remote_addr));
        
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }
    close(sock);
}

int main() {
    std::cout << "Starting Mock Runner..." << std::endl;
    std::thread ui_thread(mock_ui);
    std::thread vision_thread(mock_vision);
    std::thread fcu_thread(mock_fcu);

    std::this_thread::sleep_for(std::chrono::seconds(10));
    running = false;

    ui_thread.join();
    vision_thread.join();
    fcu_thread.join();
    
    std::cout << "Mock Runner finished." << std::endl;
    return 0;
}
