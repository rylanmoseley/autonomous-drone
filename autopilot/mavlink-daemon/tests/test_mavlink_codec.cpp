#include <gtest/gtest.h>
#include "mavlink_packet_codec.h"
#include <cstring>

TEST(MavlinkCodecTest, EncodeMsg84SetPositionTarget) {
    FlightCommand cmd;
    cmd.target_x = 3.5;
    cmd.target_y = -2.0;
    cmd.target_z = 1.5; // Altitude 1.5m -> NED Z = -1.5m
    cmd.target_yaw = 0.785;
    cmd.enable = true;

    uint8_t buffer[128];
    size_t len = MavlinkPacketCodec::encodeSetPositionTarget(cmd, 12, buffer, sizeof(buffer));
    ASSERT_GT(len, 0u);
    EXPECT_EQ(buffer[0], MAVLINK_STX_V2);
    EXPECT_EQ(buffer[4], 12); // sequence number

    uint32_t msgid = 0;
    const uint8_t* payload = nullptr;
    uint8_t payload_len = 0;
    bool ok = MavlinkPacketCodec::verifyAndExtract(buffer, len, msgid, payload, payload_len);
    EXPECT_TRUE(ok);
    EXPECT_EQ(msgid, MAVLINK_MSG_ID_SET_POSITION_TARGET_LOCAL_NED);

    MavlinkSetPositionTargetLocalNed tgt{};
    std::memcpy(&tgt, payload, sizeof(tgt));
    EXPECT_FLOAT_EQ(tgt.x, 3.5f);
    EXPECT_FLOAT_EQ(tgt.y, -2.0f);
    EXPECT_FLOAT_EQ(tgt.z, -1.5f);
    EXPECT_FLOAT_EQ(tgt.yaw, 0.785f);
}

TEST(MavlinkCodecTest, EncodeMsg102VisionPositionEstimate) {
    MavlinkVisionPositionEstimate est{};
    est.usec = 500000;
    est.x = 1.0f;
    est.y = 2.0f;
    est.z = -0.5f;
    est.yaw = 1.2f;

    uint8_t buffer[256];
    size_t len = MavlinkPacketCodec::encodeVisionPositionEstimate(est, 4, buffer, sizeof(buffer));
    ASSERT_GT(len, 0u);

    uint32_t msgid = 0;
    const uint8_t* payload = nullptr;
    uint8_t payload_len = 0;
    bool ok = MavlinkPacketCodec::verifyAndExtract(buffer, len, msgid, payload, payload_len);
    EXPECT_TRUE(ok);
    EXPECT_EQ(msgid, MAVLINK_MSG_ID_VISION_POSITION_ESTIMATE);

    MavlinkVisionPositionEstimate parsed{};
    std::memcpy(&parsed, payload, sizeof(parsed));
    EXPECT_EQ(parsed.usec, 500000u);
    EXPECT_FLOAT_EQ(parsed.x, 1.0f);
    EXPECT_FLOAT_EQ(parsed.y, 2.0f);
    EXPECT_FLOAT_EQ(parsed.z, -0.5f);
    EXPECT_FLOAT_EQ(parsed.yaw, 1.2f);
}

TEST(MavlinkCodecTest, RejectCorruptedCrc) {
    FlightCommand cmd;
    cmd.target_x = 1.0;
    cmd.target_y = 1.0;
    cmd.target_z = 1.0;
    cmd.target_yaw = 0.0;

    uint8_t buffer[128];
    size_t len = MavlinkPacketCodec::encodeSetPositionTarget(cmd, 1, buffer, sizeof(buffer));
    ASSERT_GT(len, 0u);

    // Corrupt CRC
    buffer[len - 1] ^= 0xFF;

    uint32_t msgid = 0;
    const uint8_t* payload = nullptr;
    uint8_t payload_len = 0;
    bool ok = MavlinkPacketCodec::verifyAndExtract(buffer, len, msgid, payload, payload_len);
    EXPECT_FALSE(ok);
}

TEST(MavlinkCodecTest, RejectTruncatedBuffer) {
    uint8_t buffer[10] = {MAVLINK_STX_V2, 53, 0};
    uint32_t msgid = 0;
    const uint8_t* payload = nullptr;
    uint8_t payload_len = 0;
    EXPECT_FALSE(MavlinkPacketCodec::verifyAndExtract(buffer, 8, msgid, payload, payload_len));
}

TEST(MavlinkCodecTest, EncodeMsg76CommandLong) {
    MavlinkCommandLong cmd{};
    cmd.param1 = 0.0f;       // 0 = Disarm
    cmd.param2 = 21196.0f;   // 21196 = Force immediate cutoff
    cmd.command = MAV_CMD_COMPONENT_ARM_DISARM;
    cmd.target_system = 1;
    cmd.target_component = MAV_COMP_ID_AUTOPILOT1;
    cmd.confirmation = 0;

    uint8_t buffer[128];
    size_t len = MavlinkPacketCodec::encodeCommandLong(cmd, 7, buffer, sizeof(buffer));
    ASSERT_GT(len, 0u);
    EXPECT_EQ(buffer[0], MAVLINK_STX_V2);
    EXPECT_EQ(buffer[4], 7); // Sequence number

    uint32_t msgid = 0;
    const uint8_t* payload = nullptr;
    uint8_t payload_len = 0;
    bool ok = MavlinkPacketCodec::verifyAndExtract(buffer, len, msgid, payload, payload_len);
    EXPECT_TRUE(ok);
    EXPECT_EQ(msgid, MAVLINK_MSG_ID_COMMAND_LONG);
    EXPECT_EQ(payload_len, sizeof(MavlinkCommandLong));

    MavlinkCommandLong parsed{};
    std::memcpy(&parsed, payload, sizeof(parsed));
    EXPECT_FLOAT_EQ(parsed.param1, 0.0f);
    EXPECT_FLOAT_EQ(parsed.param2, 21196.0f);
    EXPECT_EQ(parsed.command, MAV_CMD_COMPONENT_ARM_DISARM);
    EXPECT_EQ(parsed.target_system, 1);
    EXPECT_EQ(parsed.target_component, MAV_COMP_ID_AUTOPILOT1);
}

TEST(MavlinkCodecTest, Msg0HeartbeatVerifyAndExtract) {
    MavlinkHeartbeat hb{};
    hb.custom_mode = 0;
    hb.type = 2; // Quadrotor
    hb.autopilot = 3; // ArduPilot
    hb.base_mode = 128; // Armed
    hb.system_status = 4; // MAV_STATE_ACTIVE
    hb.mavlink_version = 3;

    uint8_t buffer[128];
    size_t len = mavlink_encode_packet(
        buffer, sizeof(buffer),
        5, 1, 1,
        MAVLINK_MSG_ID_HEARTBEAT,
        MAVLINK_CRC_EXTRA_HEARTBEAT,
        &hb, sizeof(hb)
    );
    ASSERT_GT(len, 0u);

    uint32_t msgid = 0;
    const uint8_t* payload = nullptr;
    uint8_t payload_len = 0;
    bool ok = MavlinkPacketCodec::verifyAndExtract(buffer, len, msgid, payload, payload_len);
    EXPECT_TRUE(ok);
    EXPECT_EQ(msgid, MAVLINK_MSG_ID_HEARTBEAT);
    EXPECT_EQ(payload_len, sizeof(MavlinkHeartbeat));

    MavlinkHeartbeat parsed{};
    std::memcpy(&parsed, payload, sizeof(parsed));
    EXPECT_EQ(parsed.type, 2);
    EXPECT_EQ(parsed.autopilot, 3);
    EXPECT_EQ(parsed.base_mode, 128);
}

TEST(MavlinkCodecTest, UnalignedBufferExtractionNoFault) {
    // Deliberately allocate a 64-bit aligned memory region and offset by an odd byte count
    alignas(uint64_t) uint8_t raw_storage[256];
    std::memset(raw_storage, 0, sizeof(raw_storage));

    // Place packet at offset 3 (unaligned for 16-bit, 32-bit, and 64-bit integers)
    uint8_t* unaligned_ptr = raw_storage + 3;

    MavlinkVisionPositionEstimate est{};
    est.usec = 0x123456789ABCDEF0ULL;
    est.x = 1.25f;
    est.y = -3.75f;
    est.z = 2.50f;
    est.yaw = 0.5236f;

    size_t len = MavlinkPacketCodec::encodeVisionPositionEstimate(est, 42, unaligned_ptr, 200);
    ASSERT_GT(len, 0u);

    uint32_t msgid = 0;
    const uint8_t* payload = nullptr;
    uint8_t payload_len = 0;

    // verifyAndExtract must not crash or trigger SIGBUS on unaligned access
    bool ok = MavlinkPacketCodec::verifyAndExtract(unaligned_ptr, len, msgid, payload, payload_len);
    EXPECT_TRUE(ok);
    EXPECT_EQ(msgid, MAVLINK_MSG_ID_VISION_POSITION_ESTIMATE);

    MavlinkVisionPositionEstimate parsed{};
    std::memcpy(&parsed, payload, sizeof(parsed));
    EXPECT_EQ(parsed.usec, 0x123456789ABCDEF0ULL);
    EXPECT_FLOAT_EQ(parsed.x, 1.25f);
    EXPECT_FLOAT_EQ(parsed.y, -3.75f);
    EXPECT_FLOAT_EQ(parsed.z, 2.50f);
    EXPECT_FLOAT_EQ(parsed.yaw, 0.5236f);
}

TEST(MavlinkCodecTest, RejectInvalidStxMagicByte) {
    FlightCommand cmd;
    cmd.target_x = 1.0;
    cmd.target_y = 2.0;
    cmd.target_z = 3.0;
    cmd.target_yaw = 0.0;
    cmd.enable = true;

    uint8_t buffer[128];
    size_t len = MavlinkPacketCodec::encodeSetPositionTarget(cmd, 1, buffer, sizeof(buffer));
    ASSERT_GT(len, 0u);

    // Corrupt STX magic byte
    buffer[0] = 0xFE; // MAVLink v1 STX, not v2

    uint32_t msgid = 0;
    const uint8_t* payload = nullptr;
    uint8_t payload_len = 0;
    EXPECT_FALSE(MavlinkPacketCodec::verifyAndExtract(buffer, len, msgid, payload, payload_len));
}

TEST(MavlinkCodecTest, RejectPayloadLengthMismatch) {
    FlightCommand cmd;
    cmd.target_x = 1.0;
    cmd.target_y = 2.0;
    cmd.target_z = 3.0;
    cmd.target_yaw = 0.0;
    cmd.enable = true;

    uint8_t buffer[128];
    size_t len = MavlinkPacketCodec::encodeSetPositionTarget(cmd, 1, buffer, sizeof(buffer));
    ASSERT_GT(len, 0u);

    // Tamper payload length to claim 100 bytes when only buffer len is available
    buffer[1] = 100;

    uint32_t msgid = 0;
    const uint8_t* payload = nullptr;
    uint8_t payload_len = 0;
    EXPECT_FALSE(MavlinkPacketCodec::verifyAndExtract(buffer, len, msgid, payload, payload_len));
}

TEST(MavlinkCodecTest, BufferTooSmallReturnsZero) {
    FlightCommand cmd;
    cmd.target_x = 1.0;
    cmd.enable = true;

    uint8_t small_buf[10]; // Minimum MAVLink v2 header is 10 bytes, + payload + CRC requires > 60 bytes
    size_t len = MavlinkPacketCodec::encodeSetPositionTarget(cmd, 1, small_buf, sizeof(small_buf));
    EXPECT_EQ(len, 0u);

    MavlinkCommandLong cmd_long{};
    len = MavlinkPacketCodec::encodeCommandLong(cmd_long, 1, small_buf, sizeof(small_buf));
    EXPECT_EQ(len, 0u);

    MavlinkVisionPositionEstimate vpe{};
    len = MavlinkPacketCodec::encodeVisionPositionEstimate(vpe, 1, small_buf, sizeof(small_buf));
    EXPECT_EQ(len, 0u);
}

TEST(MavlinkCodecTest, NullAndZeroLengthHandling) {
    uint32_t msgid = 0;
    const uint8_t* payload = nullptr;
    uint8_t payload_len = 0;

    EXPECT_FALSE(MavlinkPacketCodec::verifyAndExtract(nullptr, 100, msgid, payload, payload_len));

    uint8_t dummy[10] = {0};
    EXPECT_FALSE(MavlinkPacketCodec::verifyAndExtract(dummy, 0, msgid, payload, payload_len));
}
