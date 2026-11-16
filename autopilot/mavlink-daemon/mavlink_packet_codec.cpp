#include "mavlink_packet_codec.h"
#include <chrono>
#include <cstring>

size_t MavlinkPacketCodec::encodeSetPositionTarget(
    const FlightCommand& cmd,
    uint8_t seq,
    uint8_t* out_buffer,
    size_t out_max_size)
{
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

    return mavlink_encode_packet(
        out_buffer,
        out_max_size,
        seq,
        1,                                            // sysid
        MAV_COMP_ID_ONBOARD_COMPUTER,                 // compid: 191
        MAVLINK_MSG_ID_SET_POSITION_TARGET_LOCAL_NED, // msgid: 84
        MAVLINK_CRC_EXTRA_SET_POSITION_TARGET_LOCAL_NED, // crc_extra: 143
        &target_pkt,
        sizeof(target_pkt)
    );
}

size_t MavlinkPacketCodec::encodeVisionPositionEstimate(
    const MavlinkVisionPositionEstimate& est,
    uint8_t seq,
    uint8_t* out_buffer,
    size_t out_max_size)
{
    return mavlink_encode_packet(
        out_buffer,
        out_max_size,
        seq,
        1,                                          // sysid: 1 (Drone)
        MAV_COMP_ID_VISUAL_INERTIAL_ODOMETRY,       // compid: 197 (VIO / Vision Tag)
        MAVLINK_MSG_ID_VISION_POSITION_ESTIMATE,   // msgid: 102
        MAVLINK_CRC_EXTRA_VISION_POSITION_ESTIMATE, // crc_extra: 158
        &est,
        sizeof(est)
    );
}

bool MavlinkPacketCodec::verifyAndExtract(
    const uint8_t* buffer,
    size_t len,
    uint32_t& out_msgid,
    const uint8_t*& out_payload,
    uint8_t& out_payload_len)
{
    if (len < 12 || buffer[0] != MAVLINK_STX_V2) {
        return false;
    }

    uint8_t payload_len = buffer[1];
    if (len < static_cast<size_t>(10 + payload_len + 2)) {
        return false; // Incomplete packet
    }

    uint32_t msgid = static_cast<uint32_t>(buffer[7]) |
                     (static_cast<uint32_t>(buffer[8]) << 8) |
                     (static_cast<uint32_t>(buffer[9]) << 16);

    uint16_t expected_crc = static_cast<uint16_t>(buffer[10 + payload_len]) |
                            (static_cast<uint16_t>(buffer[10 + payload_len + 1]) << 8);

    uint8_t crc_extra = 0;
    if (msgid == MAVLINK_MSG_ID_LOCAL_POSITION_NED) crc_extra = MAVLINK_CRC_EXTRA_LOCAL_POSITION_NED;
    else if (msgid == MAVLINK_MSG_ID_ATTITUDE) crc_extra = MAVLINK_CRC_EXTRA_ATTITUDE;
    else if (msgid == MAVLINK_MSG_ID_VISION_POSITION_ESTIMATE) crc_extra = MAVLINK_CRC_EXTRA_VISION_POSITION_ESTIMATE;
    else if (msgid == MAVLINK_MSG_ID_SET_POSITION_TARGET_LOCAL_NED) crc_extra = MAVLINK_CRC_EXTRA_SET_POSITION_TARGET_LOCAL_NED;
    else return false; // Unknown message ID

    uint16_t calc_crc = 0xFFFF;
    mavlink_crc_accumulate_buffer(buffer + 1, 9 + payload_len, &calc_crc);
    mavlink_crc_accumulate(crc_extra, &calc_crc);

    if (calc_crc != expected_crc) {
        return false; // Corrupted CRC
    }

    out_msgid = msgid;
    out_payload_len = payload_len;
    out_payload = buffer + 10;
    return true;
}
