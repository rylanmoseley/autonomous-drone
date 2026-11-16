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
