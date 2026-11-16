#pragma once
#include <cstdint>
#include <cstddef>
#include "flight_control_interface.h"
#include "mavlink_protocol.h"
#include "vision_packet.h"

class MavlinkPacketCodec {
public:
    // Encodes FlightCommand into MAVLink Msg #84 (SET_POSITION_TARGET_LOCAL_NED)
    // Positive target_z is converted to negative Z in NED frame.
    static size_t encodeSetPositionTarget(
        const FlightCommand& cmd,
        uint8_t seq,
        uint8_t* out_buffer,
        size_t out_max_size);

    // Encodes MavlinkVisionPositionEstimate into MAVLink Msg #102 (VISION_POSITION_ESTIMATE)
    static size_t encodeVisionPositionEstimate(
        const MavlinkVisionPositionEstimate& est,
        uint8_t seq,
        uint8_t* out_buffer,
        size_t out_max_size);

    // Validates packet framing, message ID, and 16-bit CRC with CRC_EXTRA.
    // Returns true if valid, setting out_msgid, out_payload, and out_payload_len.
    static bool verifyAndExtract(
        const uint8_t* buffer,
        size_t len,
        uint32_t& out_msgid,
        const uint8_t*& out_payload,
        uint8_t& out_payload_len);
};
