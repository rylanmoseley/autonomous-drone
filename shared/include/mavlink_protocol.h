#pragma once

#include <cstdint>
#include <cstddef>
#include <cstring>
#include <cmath>
#include "common_types.h"
#include "vision_packet.h"

#pragma pack(push, 1)

// MAVLink v2 Constants
constexpr uint8_t MAVLINK_STX_V2 = 0xFD;
constexpr uint8_t MAV_COMP_ID_VISUAL_INERTIAL_ODOMETRY = 197;
constexpr uint8_t MAV_COMP_ID_ONBOARD_COMPUTER = 191;
constexpr uint8_t MAV_COMP_ID_AUTOPILOT1 = 1;

// Message IDs
constexpr uint32_t MAVLINK_MSG_ID_HEARTBEAT = 0;
constexpr uint32_t MAVLINK_MSG_ID_ATTITUDE = 30;
constexpr uint32_t MAVLINK_MSG_ID_LOCAL_POSITION_NED = 32;
constexpr uint32_t MAVLINK_MSG_ID_SET_POSITION_TARGET_LOCAL_NED = 84;
constexpr uint32_t MAVLINK_MSG_ID_VISION_POSITION_ESTIMATE = 102;

// CRC Extra bytes for message verification
constexpr uint8_t MAVLINK_CRC_EXTRA_HEARTBEAT = 50;
constexpr uint8_t MAVLINK_CRC_EXTRA_ATTITUDE = 39;
constexpr uint8_t MAVLINK_CRC_EXTRA_LOCAL_POSITION_NED = 185;
constexpr uint8_t MAVLINK_CRC_EXTRA_SET_POSITION_TARGET_LOCAL_NED = 143;
constexpr uint8_t MAVLINK_CRC_EXTRA_VISION_POSITION_ESTIMATE = 158;

// MAVLink v2 Header (10 bytes)
struct MavlinkHeaderV2 {
    uint8_t magic;           // 0xFD
    uint8_t len;             // Payload length
    uint8_t incompat_flags;  // Incompatibility flags
    uint8_t compat_flags;    // Compatibility flags
    uint8_t seq;             // Packet sequence
    uint8_t sysid;           // System ID (typically 1 for FCU/Companion)
    uint8_t compid;          // Component ID
    uint8_t msgid_0;         // Msg ID low byte
    uint8_t msgid_1;         // Msg ID middle byte
    uint8_t msgid_2;         // Msg ID high byte
};

// Message #102: VISION_POSITION_ESTIMATE (116 bytes payload)
struct MavlinkVisionPositionEstimate {
    uint64_t usec;           // [us] Microsecond timestamp
    float x;                 // [m] Global X position (NED)
    float y;                 // [m] Global Y position (NED)
    float z;                 // [m] Global Z position (NED)
    float roll;              // [rad] Roll angle
    float pitch;             // [rad] Pitch angle
    float yaw;               // [rad] Yaw angle
    float covariance[21];    // Upper-right triangular covariance matrix
};

// Message #84: SET_POSITION_TARGET_LOCAL_NED (53 bytes payload)
struct MavlinkSetPositionTargetLocalNed {
    uint32_t time_boot_ms;
    float x;
    float y;
    float z;
    float vx;
    float vy;
    float vz;
    float afx;
    float afy;
    float afz;
    float yaw;
    float yaw_rate;
    uint16_t type_mask;
    uint8_t target_system;
    uint8_t target_component;
    uint8_t coordinate_frame;
};

// Message #32: LOCAL_POSITION_NED (28 bytes payload)
struct MavlinkLocalPositionNed {
    uint32_t time_boot_ms;
    float x;
    float y;
    float z;
    float vx;
    float vy;
    float vz;
};

// Message #30: ATTITUDE (28 bytes payload)
struct MavlinkAttitude {
    uint32_t time_boot_ms;
    float roll;
    float pitch;
    float yaw;
    float rollspeed;
    float pitchspeed;
    float yawspeed;
};

// Message #0: HEARTBEAT (9 bytes payload)
struct MavlinkHeartbeat {
    uint32_t custom_mode;
    uint8_t type;
    uint8_t autopilot;
    uint8_t base_mode;
    uint8_t system_status;
    uint8_t mavlink_version;
};

#pragma pack(pop)

// Helper CRC accumulation
inline void mavlink_crc_accumulate(uint8_t data, uint16_t *crcAccum) {
    uint8_t tmp = data ^ (uint8_t)(*crcAccum & 0xff);
    tmp ^= (tmp << 4);
    *crcAccum = (*crcAccum >> 8) ^ (tmp << 8) ^ (tmp << 3) ^ (tmp >> 4);
}

inline void mavlink_crc_accumulate_buffer(const uint8_t *buffer, size_t length, uint16_t *crcAccum) {
    for (size_t i = 0; i < length; ++i) {
        mavlink_crc_accumulate(buffer[i], crcAccum);
    }
}

// Encode a MAVLink v2 packet into a destination buffer
// Returns total encoded packet size (10 + payload_len + 2)
inline size_t mavlink_encode_packet(
    uint8_t* out_buffer,
    size_t out_max_size,
    uint8_t seq,
    uint8_t sysid,
    uint8_t compid,
    uint32_t msgid,
    uint8_t crc_extra,
    const void* payload,
    uint8_t payload_len)
{
    size_t total_len = 10 + payload_len + 2;
    if (out_max_size < total_len) return 0;

    MavlinkHeaderV2 header;
    header.magic = MAVLINK_STX_V2;
    header.len = payload_len;
    header.incompat_flags = 0x00;
    header.compat_flags = 0x00;
    header.seq = seq;
    header.sysid = sysid;
    header.compid = compid;
    header.msgid_0 = static_cast<uint8_t>(msgid & 0xFF);
    header.msgid_1 = static_cast<uint8_t>((msgid >> 8) & 0xFF);
    header.msgid_2 = static_cast<uint8_t>((msgid >> 16) & 0xFF);

    memcpy(out_buffer, &header, sizeof(header));
    if (payload_len > 0 && payload != nullptr) {
        memcpy(out_buffer + 10, payload, payload_len);
    }

    // CRC is computed over bytes 1 through 10 + payload_len - 1 (excluding magic byte 0)
    uint16_t crc = 0xFFFF;
    mavlink_crc_accumulate_buffer(out_buffer + 1, 9 + payload_len, &crc);
    mavlink_crc_accumulate(crc_extra, &crc);

    out_buffer[10 + payload_len] = static_cast<uint8_t>(crc & 0xFF);
    out_buffer[10 + payload_len + 1] = static_cast<uint8_t>((crc >> 8) & 0xFF);

    return total_len;
}

// Convert TagMeasurement to MavlinkVisionPositionEstimate
inline MavlinkVisionPositionEstimate tag_to_mavlink_vision_position(
    const TagMeasurement& tag,
    uint64_t timestamp_us)
{
    MavlinkVisionPositionEstimate est = {};
    est.usec = timestamp_us;
    est.x = tag.camera_pos_world.x;
    est.y = tag.camera_pos_world.y;
    est.z = tag.camera_pos_world.z;

    // Convert quaternion attitude to Euler angles (Roll, Pitch, Yaw in NED)
    const Quatf& q = tag.camera_att_world;
    // roll (x-axis rotation)
    double sinr_cosp = 2.0 * (q.w * q.x + q.y * q.z);
    double cosr_cosp = 1.0 - 2.0 * (q.x * q.x + q.y * q.y);
    est.roll = static_cast<float>(std::atan2(sinr_cosp, cosr_cosp));

    // pitch (y-axis rotation)
    double sinp = 2.0 * (q.w * q.y - q.z * q.x);
    if (std::abs(sinp) >= 1.0) {
        est.pitch = static_cast<float>(std::copysign(3.14159265358979323846 / 2.0, sinp));
    } else {
        est.pitch = static_cast<float>(std::asin(sinp));
    }

    // yaw (z-axis rotation)
    double siny_cosp = 2.0 * (q.w * q.z + q.x * q.y);
    double cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z);
    est.yaw = static_cast<float>(std::atan2(siny_cosp, cosy_cosp));

    // Calculate measurement covariance based on confidence (0.0 to 1.0)
    float conf = (tag.confidence > 0.05f) ? tag.confidence : 0.05f;
    float pos_std = 0.05f / conf;   // Standard deviation (meters)
    float att_std = 0.02f / conf;   // Standard deviation (radians)
    float pos_var = pos_std * pos_std;
    float att_var = att_std * att_std;

    // Upper triangular 6x6 matrix:
    // Indices:
    // (0,0)=0: x var
    // (1,1)=6: y var
    // (2,2)=11: z var
    // (3,3)=15: roll var
    // (4,4)=18: pitch var
    // (5,5)=20: yaw var
    est.covariance[0]  = pos_var;
    est.covariance[6]  = pos_var;
    est.covariance[11] = pos_var;
    est.covariance[15] = att_var;
    est.covariance[18] = att_var;
    est.covariance[20] = att_var;

    return est;
}
