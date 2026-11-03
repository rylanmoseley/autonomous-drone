#include <gtest/gtest.h>
#include "mavlink_daemon.h"
#include "mavlink_protocol.h"
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <thread>

TEST(MavlinkDaemonTest, SendAndReceive) {
    MavlinkDaemon daemon(24550, "127.0.0.1", 24551);
    daemon.start();

    // Setup mock FCU socket
    int fcu_sock = socket(AF_INET, SOCK_DGRAM, 0);
    struct sockaddr_in fcu_addr;
    fcu_addr.sin_family = AF_INET;
    fcu_addr.sin_port = htons(24551);
    fcu_addr.sin_addr.s_addr = INADDR_ANY;
    bind(fcu_sock, (struct sockaddr *)&fcu_addr, sizeof(fcu_addr));

    struct timeval tv;
    tv.tv_sec = 0;
    tv.tv_usec = 100000;
    setsockopt(fcu_sock, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));

    // Test sendFlightCommand
    FlightCommand cmd;
    cmd.target_x = 10.0;
    daemon.sendFlightCommand(cmd);

    char buffer[1024];
    struct sockaddr_in from_addr;
    socklen_t from_len = sizeof(from_addr);
    int n = recvfrom(fcu_sock, buffer, sizeof(buffer), 0, (struct sockaddr *)&from_addr, &from_len);
    
    EXPECT_EQ(n, static_cast<int>(sizeof(FlightCommand)));
    if (n == static_cast<int>(sizeof(FlightCommand))) {
        FlightCommand received_cmd;
        memcpy(&received_cmd, buffer, sizeof(FlightCommand));
        EXPECT_DOUBLE_EQ(received_cmd.target_x, 10.0);
    }

    // Test getTelemetry
    Telemetry telem;
    telem.battery_level = 42;
    telem.error_state = true;
    
    struct sockaddr_in daemon_addr;
    daemon_addr.sin_family = AF_INET;
    daemon_addr.sin_port = htons(24550);
    inet_pton(AF_INET, "127.0.0.1", &daemon_addr.sin_addr);
    
    sendto(fcu_sock, &telem, sizeof(Telemetry), 0, (struct sockaddr *)&daemon_addr, sizeof(daemon_addr));
    
    std::this_thread::sleep_for(std::chrono::milliseconds(50)); // let rx_thread process
    Telemetry current_telem = daemon.getTelemetry();
    EXPECT_EQ(current_telem.battery_level, 42);
    EXPECT_TRUE(current_telem.error_state);

    daemon.stop();
    close(fcu_sock);
}

TEST(MavlinkDaemonTest, SendVisionPositionEstimateMavlinkV2) {
    MavlinkDaemon daemon(24560, "127.0.0.1", 24561);
    daemon.start();

    int fcu_sock = socket(AF_INET, SOCK_DGRAM, 0);
    struct sockaddr_in fcu_addr;
    fcu_addr.sin_family = AF_INET;
    fcu_addr.sin_port = htons(24561);
    fcu_addr.sin_addr.s_addr = INADDR_ANY;
    bind(fcu_sock, (struct sockaddr *)&fcu_addr, sizeof(fcu_addr));

    struct timeval tv;
    tv.tv_sec = 0;
    tv.tv_usec = 150000;
    setsockopt(fcu_sock, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));

    TagMeasurement tag{};
    tag.tag_id = 42;
    tag.is_valid = 1;
    tag.camera_pos_world = {2.5f, -1.2f, 0.8f};
    tag.camera_att_world = {1.0f, 0.0f, 0.0f, 0.0f}; // Unit quaternion (yaw = 0)
    tag.confidence = 0.95f;

    daemon.sendVisionPositionEstimate(tag, 1000000ULL);

    uint8_t buffer[512];
    struct sockaddr_in from_addr;
    socklen_t from_len = sizeof(from_addr);
    int n = recvfrom(fcu_sock, buffer, sizeof(buffer), 0, (struct sockaddr *)&from_addr, &from_len);

    size_t expected_total_len = 10 + sizeof(MavlinkVisionPositionEstimate) + 2;
    EXPECT_EQ(n, static_cast<int>(expected_total_len));

    if (n == static_cast<int>(expected_total_len)) {
        EXPECT_EQ(buffer[0], MAVLINK_STX_V2);
        EXPECT_EQ(buffer[1], sizeof(MavlinkVisionPositionEstimate));
        EXPECT_EQ(buffer[5], 1);                                    // sysid: 1
        EXPECT_EQ(buffer[6], MAV_COMP_ID_VISUAL_INERTIAL_ODOMETRY); // compid: 197

        uint32_t msgid = static_cast<uint32_t>(buffer[7]) |
                         (static_cast<uint32_t>(buffer[8]) << 8) |
                         (static_cast<uint32_t>(buffer[9]) << 16);
        EXPECT_EQ(msgid, MAVLINK_MSG_ID_VISION_POSITION_ESTIMATE);

        MavlinkVisionPositionEstimate received_est{};
        memcpy(&received_est, buffer + 10, sizeof(MavlinkVisionPositionEstimate));
        EXPECT_EQ(received_est.usec, 1000000ULL);
        EXPECT_FLOAT_EQ(received_est.x, 2.5f);
        EXPECT_FLOAT_EQ(received_est.y, -1.2f);
        EXPECT_FLOAT_EQ(received_est.z, 0.8f);
        EXPECT_NEAR(received_est.yaw, 0.0f, 1e-4);
        EXPECT_GT(received_est.covariance[0], 0.0f); // Non-zero position variance
    }

    EXPECT_TRUE(daemon.hasVisionPositionEstimate());
    EXPECT_EQ(daemon.getVisionEstimateCount(), 1u);

    daemon.stop();
    close(fcu_sock);
}

TEST(MavlinkDaemonTest, ReceiveArduPilotMavlinkTelemetry) {
    MavlinkDaemon daemon(24570, "127.0.0.1", 24571);
    daemon.start();

    int fcu_sock = socket(AF_INET, SOCK_DGRAM, 0);
    struct sockaddr_in daemon_addr;
    daemon_addr.sin_family = AF_INET;
    daemon_addr.sin_port = htons(24570);
    inet_pton(AF_INET, "127.0.0.1", &daemon_addr.sin_addr);

    // Send MAVLink LOCAL_POSITION_NED (#32)
    MavlinkLocalPositionNed pos_pkt{};
    pos_pkt.time_boot_ms = 5000;
    pos_pkt.x = 4.5f;
    pos_pkt.y = -3.2f;
    pos_pkt.z = 1.1f;

    uint8_t tx_buf[128];
    size_t len = mavlink_encode_packet(
        tx_buf, sizeof(tx_buf),
        1, 1, 1,
        MAVLINK_MSG_ID_LOCAL_POSITION_NED,
        MAVLINK_CRC_EXTRA_LOCAL_POSITION_NED,
        &pos_pkt, sizeof(pos_pkt)
    );
    sendto(fcu_sock, tx_buf, len, 0, (struct sockaddr *)&daemon_addr, sizeof(daemon_addr));

    // Send MAVLink ATTITUDE (#30)
    MavlinkAttitude att_pkt{};
    att_pkt.time_boot_ms = 5000;
    att_pkt.roll = 0.05f;
    att_pkt.pitch = -0.02f;
    att_pkt.yaw = 1.57f;

    len = mavlink_encode_packet(
        tx_buf, sizeof(tx_buf),
        2, 1, 1,
        MAVLINK_MSG_ID_ATTITUDE,
        MAVLINK_CRC_EXTRA_ATTITUDE,
        &att_pkt, sizeof(att_pkt)
    );
    sendto(fcu_sock, tx_buf, len, 0, (struct sockaddr *)&daemon_addr, sizeof(daemon_addr));

    std::this_thread::sleep_for(std::chrono::milliseconds(60));

    Telemetry telem = daemon.getTelemetry();
    EXPECT_NEAR(telem.current_pose.x, 4.5, 1e-4);
    EXPECT_NEAR(telem.current_pose.y, -3.2, 1e-4);
    EXPECT_NEAR(telem.current_pose.z, 1.1, 1e-4);
    EXPECT_NEAR(telem.current_pose.roll, 0.05, 1e-4);
    EXPECT_NEAR(telem.current_pose.pitch, -0.02, 1e-4);
    EXPECT_NEAR(telem.current_pose.yaw, 1.57, 1e-4);

    daemon.stop();
    close(fcu_sock);
}
