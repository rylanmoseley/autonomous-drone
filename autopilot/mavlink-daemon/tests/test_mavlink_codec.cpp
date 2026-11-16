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
