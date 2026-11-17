#include <gtest/gtest.h>
#include "mavlink_message_dispatcher.h"

TEST(MavlinkDispatcherTest, DirectTelemetryDispatch) {
    MavlinkMessageDispatcher dispatcher;
    Telemetry telem{};
    telem.battery_level = 88;
    telem.error_state = false;
    telem.current_pose.x = 2.0;
    telem.current_pose.y = 3.0;
    telem.current_pose.z = 1.0;

    EXPECT_FALSE(dispatcher.hasTelemetry());
    dispatcher.dispatchTelemetryStruct(telem);
    EXPECT_TRUE(dispatcher.hasTelemetry());

    Telemetry out = dispatcher.getTelemetry();
    EXPECT_EQ(out.battery_level, 88);
    EXPECT_NEAR(out.current_pose.x, 2.0, 1e-5);
    EXPECT_NEAR(out.current_pose.y, 3.0, 1e-5);
    EXPECT_NEAR(out.current_pose.z, 1.0, 1e-5);
}

TEST(MavlinkDispatcherTest, LocalPositionNedCoordinateConversion) {
    MavlinkMessageDispatcher dispatcher;

    // ArduPilot NED convention: -1.8m Z corresponds to +1.8m altitude (NEU)
    MavlinkLocalPositionNed pos{};
    pos.time_boot_ms = 1000;
    pos.x = 5.0f;
    pos.y = -2.5f;
    pos.z = -1.8f;

    dispatcher.dispatchLocalPositionNed(pos);
    Telemetry out = dispatcher.getTelemetry();
    EXPECT_FLOAT_EQ(out.current_pose.x, 5.0f);
    EXPECT_FLOAT_EQ(out.current_pose.y, -2.5f);
    EXPECT_FLOAT_EQ(out.current_pose.z, 1.8f);
}

TEST(MavlinkDispatcherTest, AttitudeDispatch) {
    MavlinkMessageDispatcher dispatcher;

    MavlinkAttitude att{};
    att.time_boot_ms = 1000;
    att.roll = 0.02f;
    att.pitch = -0.04f;
    att.yaw = 1.5707f;

    dispatcher.dispatchAttitude(att);
    Telemetry out = dispatcher.getTelemetry();
    EXPECT_FLOAT_EQ(out.current_pose.roll, 0.02f);
    EXPECT_FLOAT_EQ(out.current_pose.pitch, -0.04f);
    EXPECT_FLOAT_EQ(out.current_pose.yaw, 1.5707f);
}

TEST(MavlinkDispatcherTest, VisionEstimateTracking) {
    MavlinkMessageDispatcher dispatcher;
    EXPECT_FALSE(dispatcher.hasVisionPositionEstimate());
    EXPECT_EQ(dispatcher.getVisionEstimateCount(), 0u);

    MavlinkVisionPositionEstimate est{};
    est.usec = 12345;
    est.x = 1.0f;
    est.y = 2.0f;
    est.z = 0.5f;

    dispatcher.dispatchVisionPositionEstimate(est);
    EXPECT_TRUE(dispatcher.hasVisionPositionEstimate());
    EXPECT_EQ(dispatcher.getVisionEstimateCount(), 1u);

    auto out = dispatcher.getLastVisionPositionEstimate();
    EXPECT_EQ(out.usec, 12345u);
    EXPECT_FLOAT_EQ(out.x, 1.0f);
}

TEST(MavlinkDispatcherTest, TelemetryTimestampAndConfidencePopulated) {
    MavlinkMessageDispatcher dispatcher;

    MavlinkLocalPositionNed pos{};
    pos.time_boot_ms = 4500;
    pos.x = 1.0f;
    pos.y = 2.0f;
    pos.z = -1.5f;

    dispatcher.dispatchLocalPositionNed(pos);
    Telemetry out = dispatcher.getTelemetry();
    EXPECT_DOUBLE_EQ(out.current_pose.timestamp, 4.5);
    EXPECT_DOUBLE_EQ(out.current_pose.confidence, 1.0);

    MavlinkAttitude att{};
    att.time_boot_ms = 4550;
    att.yaw = 0.5f;

    dispatcher.dispatchAttitude(att);
    out = dispatcher.getTelemetry();
    EXPECT_DOUBLE_EQ(out.current_pose.confidence, 1.0);
}

TEST(MavlinkDispatcherTest, MavlinkPacketNotMisparsedAsRawTelemetryStruct) {
    MavlinkMessageDispatcher dispatcher;

    // Create a buffer that is exactly sizeof(Telemetry) bytes, but starts with MAVLINK_STX_V2
    uint8_t buffer[sizeof(Telemetry)];
    std::memset(buffer, 0, sizeof(buffer));
    buffer[0] = MAVLINK_STX_V2; // 0xFD

    // Set invalid payload/CRC
    buffer[1] = 10;

    // Because buffer[0] == MAVLINK_STX_V2, dispatchPacket MUST NOT treat it as a raw Telemetry struct!
    // It should route to MAVLink v2 parser, fail CRC/extraction, and return false.
    bool handled = dispatcher.dispatchPacket(buffer, sizeof(buffer));
    EXPECT_FALSE(handled);
    EXPECT_FALSE(dispatcher.hasTelemetry());
}

TEST(MavlinkDispatcherTest, HeartbeatRecordsLiveness) {
    MavlinkMessageDispatcher dispatcher;
    EXPECT_FALSE(dispatcher.hasTelemetry());

    MavlinkHeartbeat hb{};
    hb.type = 2;
    hb.autopilot = 3;
    hb.base_mode = 128;

    uint8_t buffer[128];
    size_t len = mavlink_encode_packet(
        buffer, sizeof(buffer),
        1, 1, 1,
        MAVLINK_MSG_ID_HEARTBEAT,
        MAVLINK_CRC_EXTRA_HEARTBEAT,
        &hb, sizeof(hb)
    );
    ASSERT_GT(len, 0u);

    bool handled = dispatcher.dispatchPacket(buffer, len);
    EXPECT_TRUE(handled);
    EXPECT_TRUE(dispatcher.hasTelemetry());
    EXPECT_LT(dispatcher.getTimeSinceLastTelemetry(), 0.1);
}

TEST(MavlinkDispatcherTest, NullAndEmptyPacketHandling) {
    MavlinkMessageDispatcher dispatcher;
    EXPECT_FALSE(dispatcher.dispatchPacket(nullptr, 64));
    uint8_t dummy[16] = {0};
    EXPECT_FALSE(dispatcher.dispatchPacket(dummy, 0));
}
