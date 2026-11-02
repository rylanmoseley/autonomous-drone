#include <gtest/gtest.h>
#include <vector>
#include <cmath>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>

#include "gate_detector.h"
#include "vision_packet.h"

// 1. Verify 4-corner Quad Ordering: [Top-Left, Top-Right, Bottom-Right, Bottom-Left]
TEST(VisionProcessingTest, QuadPointOrdering) {
    std::vector<cv::Point> unordered = {
        {100, 100}, // Bottom-Right
        {0, 0},     // Top-Left
        {100, 0},   // Top-Right
        {0, 100}    // Bottom-Left
    };

    std::vector<cv::Point2f> ordered = vision::orderQuadPoints(unordered);
    ASSERT_EQ(ordered.size(), 4u);

    EXPECT_FLOAT_EQ(ordered[0].x, 0.0f);   EXPECT_FLOAT_EQ(ordered[0].y, 0.0f);   // Top-Left
    EXPECT_FLOAT_EQ(ordered[1].x, 100.0f); EXPECT_FLOAT_EQ(ordered[1].y, 0.0f);   // Top-Right
    EXPECT_FLOAT_EQ(ordered[2].x, 100.0f); EXPECT_FLOAT_EQ(ordered[2].y, 100.0f); // Bottom-Right
    EXPECT_FLOAT_EQ(ordered[3].x, 0.0f);   EXPECT_FLOAT_EQ(ordered[3].y, 100.0f); // Bottom-Left
}

// 2. Verify 5-corner Pentagon Ordering: Top peak first, followed by clockwise order
TEST(VisionProcessingTest, PentagonPointOrdering) {
    float cx = 100.0f, cy = 100.0f, r = 50.0f;
    // Construct regular pentagon points with top peak at -90 deg
    std::vector<cv::Point> pts = {
        {static_cast<int>(cx + r * std::cos(54.0f * CV_PI / 180.0f)),  static_cast<int>(cy + r * std::sin(54.0f * CV_PI / 180.0f))},  // Bottom-Right
        {static_cast<int>(cx + 0.0f),                                   static_cast<int>(cy - r)},                                      // Top Peak
        {static_cast<int>(cx - r * std::cos(18.0f * CV_PI / 180.0f)), static_cast<int>(cy - r * std::sin(18.0f * CV_PI / 180.0f))},  // Upper-Left
        {static_cast<int>(cx + r * std::cos(18.0f * CV_PI / 180.0f)), static_cast<int>(cy - r * std::sin(18.0f * CV_PI / 180.0f))},  // Upper-Right
        {static_cast<int>(cx - r * std::cos(54.0f * CV_PI / 180.0f)), static_cast<int>(cy + r * std::sin(54.0f * CV_PI / 180.0f))}   // Bottom-Left
    };

    std::vector<cv::Point2f> ordered = vision::orderPentagonPoints(pts);
    ASSERT_EQ(ordered.size(), 5u);

    // Verify first point is the top peak (highest y value in screen coordinates is minimum y)
    EXPECT_NEAR(ordered[0].x, cx, 1.0f);
    EXPECT_NEAR(ordered[0].y, cy - r, 1.0f);

    // Verify subsequent points proceed clockwise (Upper-Right next)
    EXPECT_GT(ordered[1].x, cx);
    EXPECT_LT(ordered[1].y, cy);
}

// 3. Verify Shape Classification for Square, Rectangle, and Circle
TEST(VisionProcessingTest, ShapeClassification) {
    // Synthetic Square contour
    std::vector<cv::Point> square = {{0, 0}, {100, 0}, {100, 100}, {0, 100}};
    EXPECT_EQ(vision::classifyShape(square), "Square");

    // Synthetic Rectangle contour
    std::vector<cv::Point> rect = {{0, 0}, {200, 0}, {200, 100}, {0, 100}};
    EXPECT_EQ(vision::classifyShape(rect), "Rectangle");

    // Synthetic Circle contour (36 points)
    std::vector<cv::Point> circle;
    float r = 50.0f;
    for (int deg = 0; deg < 360; deg += 10) {
        float rad = deg * (float)CV_PI / 180.0f;
        circle.emplace_back(static_cast<int>(100.0f + r * std::cos(rad)),
                            static_cast<int>(100.0f + r * std::sin(rad)));
    }
    EXPECT_EQ(vision::classifyShape(circle), "Circle");
}

// 4. Verify SI Unit Definitions and Pinhole Distance Calculations
TEST(VisionProcessingTest, SIUnitConversions) {
    EXPECT_DOUBLE_EQ(vision::KNOWN_SQUARE_WIDTH_IN, 11.25);
    EXPECT_DOUBLE_EQ(vision::KNOWN_PENTAGON_WIDTH_IN, 11.75);

    EXPECT_NEAR(vision::KNOWN_SQUARE_WIDTH_M, 0.28575, 1e-5);
    EXPECT_NEAR(vision::KNOWN_PENTAGON_WIDTH_M, 0.29845, 1e-5);

    double perceived_px = 100.0;
    double dist_m = vision::calculateDistance(vision::KNOWN_SQUARE_WIDTH_M, vision::FOCAL_LENGTH, perceived_px);
    double expected_m = (0.28575 * 566.8) / 100.0;
    EXPECT_NEAR(dist_m, expected_m, 1e-4);
    EXPECT_GT(dist_m, 1.6);
    EXPECT_LT(dist_m, 1.7);
}

// 5. Verify Wire Layout Alignment of VisionGoalEstimate
TEST(VisionProcessingTest, PacketMemoryLayout) {
    EXPECT_EQ(sizeof(VisionPose), 64u);
    EXPECT_EQ(sizeof(VisionGoalEstimate), 69u);
}

// 6. Verify UDP Serialization & Transmission to Autopilot Port Contract
TEST(VisionProcessingTest, UDPLoopbackTransmission) {
    // 1. Create UDP receiver (simulating AutopilotNode::vision_loop)
    int rx_sock = socket(AF_INET, SOCK_DGRAM, 0);
    ASSERT_GE(rx_sock, 0);

    struct sockaddr_in rx_addr{};
    rx_addr.sin_family = AF_INET;
    rx_addr.sin_port = htons(0); // Bind ephemeral port
    rx_addr.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
    ASSERT_EQ(bind(rx_sock, (struct sockaddr*)&rx_addr, sizeof(rx_addr)), 0);

    socklen_t addr_len = sizeof(rx_addr);
    ASSERT_EQ(getsockname(rx_sock, (struct sockaddr*)&rx_addr, &addr_len), 0);
    int assigned_port = ntohs(rx_addr.sin_port);

    // Set 1-second timeout on receiver
    struct timeval tv{1, 0};
    setsockopt(rx_sock, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));

    // 2. Create UDP sender (simulating Vision Node)
    int tx_sock = socket(AF_INET, SOCK_DGRAM, 0);
    ASSERT_GE(tx_sock, 0);

    struct sockaddr_in tx_target{};
    tx_target.sin_family = AF_INET;
    tx_target.sin_port = htons(assigned_port);
    tx_target.sin_addr.s_addr = htonl(INADDR_LOOPBACK);

    // Populate test estimate in SI METERS and RADIANS
    vision::GateDetectionResult det;
    det.detected = true;
    det.shape = "Square";
    det.posX_m = 0.25;
    det.posY_m = -0.15;
    det.posZ_m = 2.30;
    det.pitch_rad = -0.05;
    det.yaw_rad = 0.12;
    det.roll_rad = 0.01;
    det.confidence = 0.95;
    det.is_3d = true;
    det.goal_id = 1;

    double now = 100.5;
    VisionGoalEstimate sent_packet = vision::GateDetector::toVisionGoalEstimate(det, now);

    ssize_t bytes_sent = sendto(tx_sock, &sent_packet, sizeof(sent_packet), 0,
                                (struct sockaddr*)&tx_target, sizeof(tx_target));
    ASSERT_EQ(bytes_sent, static_cast<ssize_t>(sizeof(sent_packet)));

    // 3. Receive and verify packet
    char rx_buf[256];
    struct sockaddr_in from{};
    socklen_t from_len = sizeof(from);
    ssize_t bytes_received = recvfrom(rx_sock, rx_buf, sizeof(rx_buf), 0,
                                      (struct sockaddr*)&from, &from_len);

    ASSERT_EQ(bytes_received, static_cast<ssize_t>(sizeof(VisionGoalEstimate)));

    const VisionGoalEstimate* received = reinterpret_cast<const VisionGoalEstimate*>(rx_buf);

    EXPECT_DOUBLE_EQ(received->pose.x, 0.25);
    EXPECT_DOUBLE_EQ(received->pose.y, -0.15);
    EXPECT_DOUBLE_EQ(received->pose.z, 2.30);
    EXPECT_DOUBLE_EQ(received->pose.pitch, -0.05);
    EXPECT_DOUBLE_EQ(received->pose.yaw, 0.12);
    EXPECT_DOUBLE_EQ(received->pose.roll, 0.01);
    EXPECT_DOUBLE_EQ(received->pose.timestamp, 100.5);
    EXPECT_DOUBLE_EQ(received->pose.confidence, 0.95);
    EXPECT_EQ(received->goal_id, 1);
    EXPECT_TRUE(received->determinate);

    close(tx_sock);
    close(rx_sock);
}
