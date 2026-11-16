#pragma once
#include <mutex>
#include <chrono>
#include "flight_control_interface.h"
#include "mavlink_protocol.h"
#include "mavlink_packet_codec.h"

class MavlinkMessageDispatcher {
public:
    MavlinkMessageDispatcher();

    // Dispatches raw packet bytes: handles raw Telemetry struct, or MAVLink v2 packet
    // Returns true if a packet was recognized and processed
    bool dispatchPacket(const uint8_t* buffer, size_t len);

    void dispatchTelemetryStruct(const Telemetry& telem);
    void dispatchLocalPositionNed(const MavlinkLocalPositionNed& pos);
    void dispatchAttitude(const MavlinkAttitude& att);
    void dispatchVisionPositionEstimate(const MavlinkVisionPositionEstimate& vpe);

    Telemetry getTelemetry() const;
    std::chrono::steady_clock::time_point getLastTelemetryTime() const;
    bool hasTelemetry() const;
    double getTimeSinceLastTelemetry() const;

    bool hasVisionPositionEstimate() const;
    MavlinkVisionPositionEstimate getLastVisionPositionEstimate() const;
    uint32_t getVisionEstimateCount() const;

    void recordVisionEstimateSent(const MavlinkVisionPositionEstimate& est);

private:
    Telemetry current_telemetry_{};
    std::chrono::steady_clock::time_point last_telemetry_time_{};
    bool has_telemetry_{false};
    mutable std::mutex telem_mutex_;

    MavlinkVisionPositionEstimate last_vision_est_{};
    bool has_vision_est_{false};
    uint32_t vision_est_count_{0};
    mutable std::mutex vision_mutex_;
};
