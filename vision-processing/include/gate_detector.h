#pragma once

#include <vector>
#include <string>
#include <cmath>
#include <algorithm>
#include <opencv2/opencv.hpp>
#include "vision_packet.h"

namespace vision {

// Physical Calibration Constants
constexpr double FOCAL_LENGTH = 566.8;

// Physical Gate Dimensions
constexpr double INCHES_TO_METERS = 0.0254;
constexpr double METERS_TO_INCHES = 1.0 / 0.0254;

constexpr double KNOWN_SQUARE_WIDTH_IN   = 11.25; // Physical width/height of square opening (inches)
constexpr double KNOWN_PENTAGON_WIDTH_IN = 11.75; // Physical max width of pentagon opening (inches)

constexpr double KNOWN_SQUARE_WIDTH_M   = KNOWN_SQUARE_WIDTH_IN * INCHES_TO_METERS;   // 0.28575 m
constexpr double KNOWN_PENTAGON_WIDTH_M = KNOWN_PENTAGON_WIDTH_IN * INCHES_TO_METERS; // 0.29845 m

// 1D pinhole distance fallback: Distance = (Known Width * Focal Length) / Perceived Pixel Width
inline double calculateDistance(double knownWidth, double focalLength, double perWidthPixels) {
    if (perWidthPixels <= 0.0) return 0.0;
    return (knownWidth * focalLength) / perWidthPixels;
}

// Helper: Convert Euler angles (roll, pitch, yaw in radians) to Quatf
inline Quatf eulerToQuaternion(double roll, double pitch, double yaw) {
    double cy = std::cos(yaw * 0.5);
    double sy = std::sin(yaw * 0.5);
    double cp = std::cos(pitch * 0.5);
    double sp = std::sin(pitch * 0.5);
    double cr = std::cos(roll * 0.5);
    double sr = std::sin(roll * 0.5);

    Quatf q;
    q.w = static_cast<float>(cr * cp * cy + sr * cp * sy);
    q.x = static_cast<float>(sr * cp * cy - cr * sp * sy);
    q.y = static_cast<float>(cr * sp * cy + sr * cp * sy);
    q.z = static_cast<float>(cr * cp * sy - sr * sp * cy);
    return q;
}

// Order 4 quadrilateral corners: [Top-Left, Top-Right, Bottom-Right, Bottom-Left]
std::vector<cv::Point2f> orderQuadPoints(const std::vector<cv::Point>& pts);

// Order 5 pentagon corners clockwise matching the 3D model (starting at Top Peak ~ -90 deg)
std::vector<cv::Point2f> orderPentagonPoints(const std::vector<cv::Point>& pts);

// Advanced classification using Circularity, Fill Ratio, and aspect ratio checks
std::string classifyShape(const std::vector<cv::Point>& contour);

// Gate detection and 6-DoF estimation result
struct GateDetectionResult {
    bool detected{false};
    std::string shape{"Unknown"};
    cv::Point center{0, 0};
    
    // Position relative to camera in SI METERS
    double posX_m{0.0};
    double posY_m{0.0};
    double posZ_m{0.0};

    // Orientation in RADIANS (for shared autopilot protocol)
    double pitch_rad{0.0};
    double yaw_rad{0.0};
    double roll_rad{0.0};

    // Orientation in DEGREES (for HUD and debug display)
    double pitch_deg{0.0};
    double yaw_deg{0.0};
    double roll_deg{0.0};

    double confidence{0.0};
    bool is_3d{false}; // true if solved via solvePnP, false if fallback 1D
    int goal_id{0};    // 1 for Square, 2 for Pentagon, 3 for Circle, etc.
};

class GateDetector {
public:
    explicit GateDetector(double focal_length = FOCAL_LENGTH);

    // Process a single frame and populate detection results
    GateDetectionResult processFrame(cv::Mat& frame, bool draw_overlay = true);

    // Helper to package result into canonical VisionPacket struct
    static VisionPacket toVisionPacket(const GateDetectionResult& result, uint32_t sequence_id = 0, uint64_t timestamp_ns = 0);

private:
    double focal_length_;
    std::vector<cv::Point3f> square_object_points_;
    std::vector<cv::Point3f> pentagon_object_points_;
};

} // namespace vision
