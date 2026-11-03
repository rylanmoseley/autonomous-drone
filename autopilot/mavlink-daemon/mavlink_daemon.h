#pragma once
#include "flight_control_interface.h"
#include "mavlink_protocol.h"
#include "vision_packet.h"
#include <string>
#include <mutex>
#include <thread>
#include <atomic>

class MavlinkDaemon {
public:
    MavlinkDaemon(int local_port, const std::string& remote_ip, int remote_port);
    ~MavlinkDaemon();
    
    void sendFlightCommand(const FlightCommand& cmd);
    void sendVisionPositionEstimate(const TagMeasurement& tag, uint64_t timestamp_us);
    void sendVisionPositionEstimate(const MavlinkVisionPositionEstimate& est);

    Telemetry getTelemetry();
    bool hasVisionPositionEstimate() const;
    MavlinkVisionPositionEstimate getLastVisionPositionEstimate() const;
    uint32_t getVisionEstimateCount() const;

    void start();
    void stop();

private:
    void receiveLoop();

    int local_port_;
    std::string remote_ip_;
    int remote_port_;
    int socket_fd_;
    
    std::atomic<bool> running_;
    std::thread rx_thread_;
    
    Telemetry current_telemetry_{};
    mutable std::mutex telem_mutex_;

    MavlinkVisionPositionEstimate last_vision_est_{};
    bool has_vision_est_{false};
    uint32_t vision_est_count_{0};
    uint8_t tx_seq_{0};
    mutable std::mutex vision_mutex_;
};
