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
    uint8_t buffer[2048];
    while (running_) {
        struct sockaddr_in remote_addr;
        socklen_t addr_len = sizeof(remote_addr);
        int n = recvfrom(socket_fd_, buffer, sizeof(buffer), 0, (struct sockaddr *)&remote_addr, &addr_len);
        if (n <= 0) continue;

        // 1. Direct mock Telemetry struct (used by tests and in-process simulator)
        if (n == static_cast<int>(sizeof(Telemetry))) {
            std::lock_guard<std::mutex> lock(telem_mutex_);
            memcpy(&current_telemetry_, buffer, sizeof(Telemetry));
            last_telemetry_time_ = std::chrono::steady_clock::now();
            has_telemetry_ = true;
            continue;
        }

        // 2. Standard MAVLink v2 packet from ArduPilot FCU
        if (n >= 12 && buffer[0] == MAVLINK_STX_V2) {
            uint8_t payload_len = buffer[1];
            if (n < 10 + payload_len + 2) continue; // Incomplete packet

            uint32_t msgid = static_cast<uint32_t>(buffer[7]) |
                             (static_cast<uint32_t>(buffer[8]) << 8) |
                             (static_cast<uint32_t>(buffer[9]) << 16);

            // Verify 16-bit CRC over packet
            uint16_t expected_crc = static_cast<uint16_t>(buffer[10 + payload_len]) |
                                    (static_cast<uint16_t>(buffer[10 + payload_len + 1]) << 8);
            uint8_t crc_extra = 0;
            if (msgid == MAVLINK_MSG_ID_LOCAL_POSITION_NED) crc_extra = MAVLINK_CRC_EXTRA_LOCAL_POSITION_NED;
            else if (msgid == MAVLINK_MSG_ID_ATTITUDE) crc_extra = MAVLINK_CRC_EXTRA_ATTITUDE;
            else if (msgid == MAVLINK_MSG_ID_VISION_POSITION_ESTIMATE) crc_extra = MAVLINK_CRC_EXTRA_VISION_POSITION_ESTIMATE;
            else if (msgid == MAVLINK_MSG_ID_SET_POSITION_TARGET_LOCAL_NED) crc_extra = MAVLINK_CRC_EXTRA_SET_POSITION_TARGET_LOCAL_NED;

            uint16_t calc_crc = 0xFFFF;
            mavlink_crc_accumulate_buffer(buffer + 1, 9 + payload_len, &calc_crc);
            mavlink_crc_accumulate(crc_extra, &calc_crc);

            if (calc_crc != expected_crc) {
                continue; // Corrupted CRC, discard malformed packet
            }

            const uint8_t* payload = buffer + 10;

            if (msgid == MAVLINK_MSG_ID_LOCAL_POSITION_NED && payload_len >= sizeof(MavlinkLocalPositionNed)) {
                const auto* pos = reinterpret_cast<const MavlinkLocalPositionNed*>(payload);
                std::lock_guard<std::mutex> lock(telem_mutex_);
                current_telemetry_.current_pose.x = pos->x;
                current_telemetry_.current_pose.y = pos->y;
                current_telemetry_.current_pose.z = -pos->z; // Convert ArduPilot NED Down to internal NEU altitude!
                last_telemetry_time_ = std::chrono::steady_clock::now();
                has_telemetry_ = true;
            } else if (msgid == MAVLINK_MSG_ID_ATTITUDE && payload_len >= sizeof(MavlinkAttitude)) {
                const auto* att = reinterpret_cast<const MavlinkAttitude*>(payload);
                std::lock_guard<std::mutex> lock(telem_mutex_);
                current_telemetry_.current_pose.roll = att->roll;
                current_telemetry_.current_pose.pitch = att->pitch;
                current_telemetry_.current_pose.yaw = att->yaw;
                last_telemetry_time_ = std::chrono::steady_clock::now();
                has_telemetry_ = true;
            } else if (msgid == MAVLINK_MSG_ID_VISION_POSITION_ESTIMATE && payload_len >= sizeof(MavlinkVisionPositionEstimate)) {
                const auto* vpe = reinterpret_cast<const MavlinkVisionPositionEstimate*>(payload);
                std::lock_guard<std::mutex> lock(vision_mutex_);
                last_vision_est_ = *vpe;
                has_vision_est_ = true;
                vision_est_count_++;
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

    // 1. Direct FlightCommand struct for test suite compatibility
    sendto(socket_fd_, &cmd, sizeof(FlightCommand), 0, (struct sockaddr *)&remote_addr, sizeof(remote_addr));

    // 2. MAVLink Msg #84 SET_POSITION_TARGET_LOCAL_NED for standard ArduPilot FCU
    MavlinkSetPositionTargetLocalNed target_pkt = {};
    target_pkt.time_boot_ms = static_cast<uint32_t>(
        std::chrono::duration_cast<std::chrono::milliseconds>(
            std::chrono::steady_clock::now().time_since_epoch()).count());
    target_pkt.x = static_cast<float>(cmd.target_x);
    target_pkt.y = static_cast<float>(cmd.target_y);
    target_pkt.z = static_cast<float>(-cmd.target_z); // NED convention: positive altitude is negative Z
    target_pkt.yaw = static_cast<float>(cmd.target_yaw);
    target_pkt.type_mask = 0x0DF8; // Control position & yaw; ignore velocity & acceleration
    target_pkt.target_system = 1;
    target_pkt.target_component = MAV_COMP_ID_AUTOPILOT1;
    target_pkt.coordinate_frame = 1; // MAV_FRAME_LOCAL_NED

    uint8_t packet_buffer[128];
    size_t encoded_len = mavlink_encode_packet(
        packet_buffer,
        sizeof(packet_buffer),
        tx_seq_++,
        1,                                          // sysid
        MAV_COMP_ID_ONBOARD_COMPUTER,               // compid: 191
        MAVLINK_MSG_ID_SET_POSITION_TARGET_LOCAL_NED, // msgid: 84
        MAVLINK_CRC_EXTRA_SET_POSITION_TARGET_LOCAL_NED, // crc_extra: 143
        &target_pkt,
        sizeof(target_pkt)
    );
    if (encoded_len > 0) {
        sendto(socket_fd_, packet_buffer, encoded_len, 0, (struct sockaddr *)&remote_addr, sizeof(remote_addr));
    }
}

void MavlinkDaemon::sendVisionPositionEstimate(const TagMeasurement& tag, uint64_t timestamp_us) {
    if (!tag.is_valid) return;
    MavlinkVisionPositionEstimate est = tag_to_mavlink_vision_position(tag, timestamp_us);
    sendVisionPositionEstimate(est);
}

void MavlinkDaemon::sendVisionPositionEstimate(const MavlinkVisionPositionEstimate& est) {
    {
        std::lock_guard<std::mutex> lock(vision_mutex_);
        last_vision_est_ = est;
        has_vision_est_ = true;
        vision_est_count_++;
    }

    if (socket_fd_ < 0) return;

    struct sockaddr_in remote_addr;
    memset(&remote_addr, 0, sizeof(remote_addr));
    remote_addr.sin_family = AF_INET;
    remote_addr.sin_port = htons(remote_port_);
    inet_pton(AF_INET, remote_ip_.c_str(), &remote_addr.sin_addr);

    uint8_t packet_buffer[256];
    size_t encoded_len = mavlink_encode_packet(
        packet_buffer,
        sizeof(packet_buffer),
        tx_seq_++,
        1,                                          // sysid: 1 (Drone)
        MAV_COMP_ID_VISUAL_INERTIAL_ODOMETRY,       // compid: 197 (VIO / Vision Tag)
        MAVLINK_MSG_ID_VISION_POSITION_ESTIMATE,   // msgid: 102
        MAVLINK_CRC_EXTRA_VISION_POSITION_ESTIMATE, // crc_extra: 158
        &est,
        sizeof(est)
    );

    if (encoded_len > 0) {
        sendto(socket_fd_, packet_buffer, encoded_len, 0, (struct sockaddr *)&remote_addr, sizeof(remote_addr));
    }
}

Telemetry MavlinkDaemon::getTelemetry() {
    std::lock_guard<std::mutex> lock(telem_mutex_);
    return current_telemetry_;
}

bool MavlinkDaemon::hasVisionPositionEstimate() const {
    std::lock_guard<std::mutex> lock(vision_mutex_);
    return has_vision_est_;
}

MavlinkVisionPositionEstimate MavlinkDaemon::getLastVisionPositionEstimate() const {
    std::lock_guard<std::mutex> lock(vision_mutex_);
    return last_vision_est_;
}

uint32_t MavlinkDaemon::getVisionEstimateCount() const {
    std::lock_guard<std::mutex> lock(vision_mutex_);
    return vision_est_count_;
}

double MavlinkDaemon::getTimeSinceLastTelemetry() const {
    std::lock_guard<std::mutex> lock(telem_mutex_);
    if (!has_telemetry_) return 999.0;
    auto now = std::chrono::steady_clock::now();
    return std::chrono::duration<double>(now - last_telemetry_time_).count();
}

bool MavlinkDaemon::isFcuConnected(double timeout_s) const {
    return getTimeSinceLastTelemetry() <= timeout_s;
}
