#include "mavlink_daemon.h"
#include <iostream>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <cstring>

MavlinkDaemon::MavlinkDaemon(int local_port, const std::string& remote_ip, int remote_port)
    : local_port_(local_port), remote_ip_(remote_ip), remote_port_(remote_port), socket_fd_(-1), running_(false) {
    current_telemetry_.error_state = false;
    current_telemetry_.battery_level = 100;
}

MavlinkDaemon::~MavlinkDaemon() {
    stop();
}

void MavlinkDaemon::start() {
    socket_fd_ = socket(AF_INET, SOCK_DGRAM, 0);
    if (socket_fd_ < 0) {
        std::cerr << "Failed to create socket" << std::endl;
        return;
    }

    struct sockaddr_in local_addr;
    memset(&local_addr, 0, sizeof(local_addr));
    local_addr.sin_family = AF_INET;
    local_addr.sin_addr.s_addr = INADDR_ANY;
    local_addr.sin_port = htons(local_port_);

    if (bind(socket_fd_, (struct sockaddr *)&local_addr, sizeof(local_addr)) < 0) {
        std::cerr << "Failed to bind socket" << std::endl;
        close(socket_fd_);
        socket_fd_ = -1;
        return;
    }

    struct timeval tv;
    tv.tv_sec = 0;
    tv.tv_usec = 100000;
    setsockopt(socket_fd_, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));

    running_ = true;
    rx_thread_ = std::thread(&MavlinkDaemon::receiveLoop, this);
}

void MavlinkDaemon::stop() {
    running_ = false;
    if (rx_thread_.joinable()) {
        rx_thread_.join();
    }
    if (socket_fd_ >= 0) {
        close(socket_fd_);
        socket_fd_ = -1;
    }
}

void MavlinkDaemon::receiveLoop() {
    char buffer[1024];
    while (running_) {
        struct sockaddr_in remote_addr;
        socklen_t addr_len = sizeof(remote_addr);
        int n = recvfrom(socket_fd_, buffer, sizeof(buffer), 0, (struct sockaddr *)&remote_addr, &addr_len);
        if (n > 0) {
            // Simplified parsing for tests: expect directly serialized Telemetry struct
            std::cout << "Mavlink recv: " << n << std::endl; if (n == sizeof(Telemetry)) {
                std::lock_guard<std::mutex> lock(telem_mutex_);
                memcpy(&current_telemetry_, buffer, sizeof(Telemetry));
            }
        }
    }
}

void MavlinkDaemon::sendFlightCommand(const FlightCommand& cmd) {
    if (socket_fd_ < 0) return;
    struct sockaddr_in remote_addr;
    memset(&remote_addr, 0, sizeof(remote_addr));
    remote_addr.sin_family = AF_INET;
    remote_addr.sin_port = htons(remote_port_);
    inet_pton(AF_INET, remote_ip_.c_str(), &remote_addr.sin_addr);

    sendto(socket_fd_, &cmd, sizeof(FlightCommand), 0, (struct sockaddr *)&remote_addr, sizeof(remote_addr));
}

Telemetry MavlinkDaemon::getTelemetry() {
    std::lock_guard<std::mutex> lock(telem_mutex_);
    return current_telemetry_;
}
