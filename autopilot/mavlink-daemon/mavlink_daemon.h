#pragma once
#include "flight_control_interface.h"
#include "mavlink_protocol.h"
#include "vision_packet.h"
#include "mavlink_packet_codec.h"
#include "mavlink_message_dispatcher.h"
#include "mavlink_network_channel.h"
#include "mavlink_fcu_watchdog.h"
#include <string>

class MavlinkDaemon {
public:
    MavlinkDaemon(int local_port, const std::string& remote_ip, int remote_port);
    ~MavlinkDaemon();
    
    void sendFlightCommand(const FlightCommand& cmd);
    void sendVisionPositionEstimate(const TagMeasurement& tag, uint64_t timestamp_us);
    void sendVisionPositionEstimate(const MavlinkVisionPositionEstimate& est);

    Telemetry getTelemetry();
    double getTimeSinceLastTelemetry() const;
    bool isFcuConnected(double timeout_s = 1.0) const;
    bool hasVisionPositionEstimate() const;
    MavlinkVisionPositionEstimate getLastVisionPositionEstimate() const;
    uint32_t getVisionEstimateCount() const;

    void start();
    void stop();

    // Accessors for modular subcomponents
    MavlinkNetworkChannel& getChannel() { return channel_; }
    const MavlinkNetworkChannel& getChannel() const { return channel_; }
    MavlinkMessageDispatcher& getDispatcher() { return dispatcher_; }
    const MavlinkMessageDispatcher& getDispatcher() const { return dispatcher_; }

private:
    MavlinkNetworkChannel channel_;
    MavlinkMessageDispatcher dispatcher_;
    uint8_t tx_seq_{0};
};
