#include "mavlink_message_dispatcher.h"
#include <cstring>

MavlinkMessageDispatcher::MavlinkMessageDispatcher() {
    current_telemetry_.error_state = false;
    current_telemetry_.battery_level = 100;
}

bool MavlinkMessageDispatcher::dispatchPacket(const uint8_t* buffer, size_t len) {
    if (!buffer || len == 0) return false;

    // 1. Direct mock Telemetry struct (used by tests and in-process simulator)
    if (len == sizeof(Telemetry) && buffer[0] != MAVLINK_STX_V2) {
        Telemetry telem;
        std::memcpy(&telem, buffer, sizeof(Telemetry));
        dispatchTelemetryStruct(telem);
        return true;
    }

    // 2. Standard MAVLink v2 packet
    uint32_t msgid = 0;
    const uint8_t* payload = nullptr;
    uint8_t payload_len = 0;

    if (!MavlinkPacketCodec::verifyAndExtract(buffer, len, msgid, payload, payload_len)) {
        return false;
    }

    if (msgid == MAVLINK_MSG_ID_LOCAL_POSITION_NED && payload_len >= sizeof(MavlinkLocalPositionNed)) {
        MavlinkLocalPositionNed pos{};
        std::memcpy(&pos, payload, sizeof(pos));
        dispatchLocalPositionNed(pos);
        return true;
    } else if (msgid == MAVLINK_MSG_ID_ATTITUDE && payload_len >= sizeof(MavlinkAttitude)) {
        MavlinkAttitude att{};
        std::memcpy(&att, payload, sizeof(att));
        dispatchAttitude(att);
        return true;
    } else if (msgid == MAVLINK_MSG_ID_VISION_POSITION_ESTIMATE && payload_len >= sizeof(MavlinkVisionPositionEstimate)) {
        MavlinkVisionPositionEstimate vpe{};
        std::memcpy(&vpe, payload, sizeof(vpe));
        dispatchVisionPositionEstimate(vpe);
        return true;
    } else if (msgid == MAVLINK_MSG_ID_HEARTBEAT) {
        std::lock_guard<std::mutex> lock(telem_mutex_);
        last_telemetry_time_ = std::chrono::steady_clock::now();
        has_telemetry_ = true;
        return true;
    }

    return false;
}

void MavlinkMessageDispatcher::dispatchTelemetryStruct(const Telemetry& telem) {
    std::lock_guard<std::mutex> lock(telem_mutex_);
    current_telemetry_ = telem;
    last_telemetry_time_ = std::chrono::steady_clock::now();
    has_telemetry_ = true;
}

void MavlinkMessageDispatcher::dispatchLocalPositionNed(const MavlinkLocalPositionNed& pos) {
    std::lock_guard<std::mutex> lock(telem_mutex_);
    current_telemetry_.current_pose.x = pos.x;
    current_telemetry_.current_pose.y = pos.y;
    current_telemetry_.current_pose.z = -pos.z; // Convert ArduPilot NED Down to internal NEU altitude!
    current_telemetry_.current_pose.timestamp = pos.time_boot_ms * 1e-3;
    current_telemetry_.current_pose.confidence = 1.0;
    last_telemetry_time_ = std::chrono::steady_clock::now();
    has_telemetry_ = true;
}

void MavlinkMessageDispatcher::dispatchAttitude(const MavlinkAttitude& att) {
    std::lock_guard<std::mutex> lock(telem_mutex_);
    current_telemetry_.current_pose.roll = att.roll;
    current_telemetry_.current_pose.pitch = att.pitch;
    current_telemetry_.current_pose.yaw = att.yaw;
    if (current_telemetry_.current_pose.timestamp == 0.0) {
        current_telemetry_.current_pose.timestamp = att.time_boot_ms * 1e-3;
    }
    current_telemetry_.current_pose.confidence = 1.0;
    last_telemetry_time_ = std::chrono::steady_clock::now();
    has_telemetry_ = true;
}

void MavlinkMessageDispatcher::dispatchVisionPositionEstimate(const MavlinkVisionPositionEstimate& vpe) {
    std::lock_guard<std::mutex> lock(vision_mutex_);
    last_vision_est_ = vpe;
    has_vision_est_ = true;
    vision_est_count_++;
}

void MavlinkMessageDispatcher::recordVisionEstimateSent(const MavlinkVisionPositionEstimate& est) {
    std::lock_guard<std::mutex> lock(vision_mutex_);
    last_vision_est_ = est;
    has_vision_est_ = true;
    vision_est_count_++;
}

Telemetry MavlinkMessageDispatcher::getTelemetry() const {
    std::lock_guard<std::mutex> lock(telem_mutex_);
    return current_telemetry_;
}

std::chrono::steady_clock::time_point MavlinkMessageDispatcher::getLastTelemetryTime() const {
    std::lock_guard<std::mutex> lock(telem_mutex_);
    return last_telemetry_time_;
}

bool MavlinkMessageDispatcher::hasTelemetry() const {
    std::lock_guard<std::mutex> lock(telem_mutex_);
    return has_telemetry_;
}

double MavlinkMessageDispatcher::getTimeSinceLastTelemetry() const {
    std::lock_guard<std::mutex> lock(telem_mutex_);
    if (!has_telemetry_) return 999.0;
    auto now = std::chrono::steady_clock::now();
    return std::chrono::duration<double>(now - last_telemetry_time_).count();
}

bool MavlinkMessageDispatcher::hasVisionPositionEstimate() const {
    std::lock_guard<std::mutex> lock(vision_mutex_);
    return has_vision_est_;
}

MavlinkVisionPositionEstimate MavlinkMessageDispatcher::getLastVisionPositionEstimate() const {
    std::lock_guard<std::mutex> lock(vision_mutex_);
    return last_vision_est_;
}

uint32_t MavlinkMessageDispatcher::getVisionEstimateCount() const {
    std::lock_guard<std::mutex> lock(vision_mutex_);
    return vision_est_count_;
}
