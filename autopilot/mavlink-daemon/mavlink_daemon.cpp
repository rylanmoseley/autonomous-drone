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
    // 1. If E-Stop or disarmed, immediately issue MAVLink Msg #76 COMMAND_LONG emergency disarm
    if (cmd.estop || !cmd.enable) {
        uint8_t cmd_buffer[128];
        MavlinkCommandLong disarm_cmd = {};
        disarm_cmd.param1 = 0.0f;       // 0 = Disarm
        disarm_cmd.param2 = 21196.0f;   // 21196 = Force emergency disarm immediately
        disarm_cmd.command = MAV_CMD_COMPONENT_ARM_DISARM;
        disarm_cmd.target_system = 1;
        disarm_cmd.target_component = MAV_COMP_ID_AUTOPILOT1;

        size_t cmd_len = MavlinkPacketCodec::encodeCommandLong(
            disarm_cmd,
            tx_seq_++,
            cmd_buffer,
            sizeof(cmd_buffer)
        );
        if (cmd_len > 0) {
            channel_.sendData(cmd_buffer, cmd_len);
        }
    }

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
