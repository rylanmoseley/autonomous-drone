#include "mavlink_network_channel.h"
#include <iostream>
#include <cstring>
#include <arpa/inet.h>
#include <unistd.h>

MavlinkNetworkChannel::MavlinkNetworkChannel(int local_port, const std::string& remote_ip, int remote_port)
    : local_port_(local_port), remote_ip_(remote_ip), remote_port_(remote_port),
      socket_fd_(-1), running_(false) {}

MavlinkNetworkChannel::~MavlinkNetworkChannel() {
    closeChannel();
}

bool MavlinkNetworkChannel::openChannel() {
    if (socket_fd_ >= 0) return true;

    socket_fd_ = socket(AF_INET, SOCK_DGRAM, 0);
    if (socket_fd_ < 0) {
        std::cerr << "MavlinkNetworkChannel: Failed to create socket" << std::endl;
        return false;
    }

    struct sockaddr_in local_addr;
    std::memset(&local_addr, 0, sizeof(local_addr));
    local_addr.sin_family = AF_INET;
    local_addr.sin_addr.s_addr = INADDR_ANY;
    local_addr.sin_port = htons(local_port_);

    if (bind(socket_fd_, (struct sockaddr *)&local_addr, sizeof(local_addr)) < 0) {
        std::cerr << "MavlinkNetworkChannel: Failed to bind socket on port " << local_port_ << std::endl;
        close(socket_fd_);
        socket_fd_ = -1;
        return false;
    }

    struct timeval tv;
    tv.tv_sec = 0;
    tv.tv_usec = 100000;
    setsockopt(socket_fd_, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));

    return true;
}

void MavlinkNetworkChannel::closeChannel() {
    stopReceiver();
    if (socket_fd_ >= 0) {
        close(socket_fd_);
        socket_fd_ = -1;
    }
}

bool MavlinkNetworkChannel::startReceiver(PacketCallback callback) {
    if (!openChannel()) return false;
    callback_ = std::move(callback);
    running_ = true;
    rx_thread_ = std::thread(&MavlinkNetworkChannel::receiveLoop, this);
    return true;
}

void MavlinkNetworkChannel::stopReceiver() {
    running_ = false;
    if (rx_thread_.joinable()) {
        rx_thread_.join();
    }
}

void MavlinkNetworkChannel::receiveLoop() {
    uint8_t buffer[2048];
    while (running_) {
        struct sockaddr_in remote_addr;
        socklen_t addr_len = sizeof(remote_addr);
        int n = recvfrom(socket_fd_, buffer, sizeof(buffer), 0, (struct sockaddr *)&remote_addr, &addr_len);
        if (n <= 0) continue;

        if (callback_) {
            callback_(buffer, static_cast<size_t>(n));
        }
    }
}

bool MavlinkNetworkChannel::sendData(const void* data, size_t len) {
    if (socket_fd_ < 0 || !data || len == 0) return false;

    struct sockaddr_in remote_addr;
    std::memset(&remote_addr, 0, sizeof(remote_addr));
    remote_addr.sin_family = AF_INET;
    remote_addr.sin_port = htons(remote_port_);
    inet_pton(AF_INET, remote_ip_.c_str(), &remote_addr.sin_addr);

    ssize_t sent = sendto(socket_fd_, data, len, 0, (struct sockaddr *)&remote_addr, sizeof(remote_addr));
    return (sent == static_cast<ssize_t>(len));
}
