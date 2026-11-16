#include "mavlink_daemon.h"

MavlinkDaemon::MavlinkDaemon(int local_port, const std::string& remote_ip, int remote_port)
    : channel_(local_port, remote_ip, remote_port),
      dispatcher_(),
      tx_seq_(0) {}

MavlinkDaemon::~MavlinkDaemon() {
    stop();
}

void MavlinkDaemon::start() {
    channel_.startReceiver([this](const uint8_t* buffer, size_t len) {
        dispatcher_.dispatchPacket(buffer, len);
    });
}

void MavlinkDaemon::stop() {
    channel_.stopReceiver();
    channel_.closeChannel();
}

void MavlinkDaemon::sendFlightCommand(const FlightCommand& cmd) {
    // 1. Direct FlightCommand struct for test suite and legacy harness compatibility
    channel_.sendData(&cmd, sizeof(FlightCommand));

    // 2. MAVLink Msg #84 SET_POSITION_TARGET_LOCAL_NED for standard ArduPilot FCU
    uint8_t packet_buffer[128];
    size_t encoded_len = MavlinkPacketCodec::encodeSetPositionTarget(
        cmd,
        tx_seq_++,
        packet_buffer,
        sizeof(packet_buffer)
    );
    if (encoded_len > 0) {
        channel_.sendData(packet_buffer, encoded_len);
    }
}

void MavlinkDaemon::sendVisionPositionEstimate(const TagMeasurement& tag, uint64_t timestamp_us) {
    if (!tag.is_valid) return;
    MavlinkVisionPositionEstimate est = tag_to_mavlink_vision_position(tag, timestamp_us);
    sendVisionPositionEstimate(est);
}

void MavlinkDaemon::sendVisionPositionEstimate(const MavlinkVisionPositionEstimate& est) {
    dispatcher_.recordVisionEstimateSent(est);

    uint8_t packet_buffer[256];
    size_t encoded_len = MavlinkPacketCodec::encodeVisionPositionEstimate(
        est,
        tx_seq_++,
        packet_buffer,
        sizeof(packet_buffer)
    );

    if (encoded_len > 0) {
        channel_.sendData(packet_buffer, encoded_len);
    }
}

Telemetry MavlinkDaemon::getTelemetry() {
    return dispatcher_.getTelemetry();
}

bool MavlinkDaemon::hasVisionPositionEstimate() const {
    return dispatcher_.hasVisionPositionEstimate();
}

MavlinkVisionPositionEstimate MavlinkDaemon::getLastVisionPositionEstimate() const {
    return dispatcher_.getLastVisionPositionEstimate();
}

uint32_t MavlinkDaemon::getVisionEstimateCount() const {
    return dispatcher_.getVisionEstimateCount();
}

double MavlinkDaemon::getTimeSinceLastTelemetry() const {
    return dispatcher_.getTimeSinceLastTelemetry();
}

bool MavlinkDaemon::isFcuConnected(double timeout_s) const {
    return MavlinkFcuWatchdog::isConnected(getTimeSinceLastTelemetry(), timeout_s);
}
