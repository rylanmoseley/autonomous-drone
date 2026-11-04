#include "gate_detector.h"

namespace vision {

std::vector<cv::Point2f> orderQuadPoints(const std::vector<cv::Point>& pts) {
    if (pts.size() != 4) return {};
    std::vector<cv::Point2f> ordered(4);
    std::vector<float> sum(4), diff(4);
    for (int i = 0; i < 4; i++) {
        sum[i] = static_cast<float>(pts[i].x + pts[i].y);
        diff[i] = static_cast<float>(pts[i].x - pts[i].y);
    }
    int tl = static_cast<int>(std::min_element(sum.begin(), sum.end()) - sum.begin());
    int br = static_cast<int>(std::max_element(sum.begin(), sum.end()) - sum.begin());
    int tr = static_cast<int>(std::max_element(diff.begin(), diff.end()) - diff.begin());
    int bl = static_cast<int>(std::min_element(diff.begin(), diff.end()) - diff.begin());

    ordered[0] = cv::Point2f(static_cast<float>(pts[tl].x), static_cast<float>(pts[tl].y)); // Top-Left
    ordered[1] = cv::Point2f(static_cast<float>(pts[tr].x), static_cast<float>(pts[tr].y)); // Top-Right
    ordered[2] = cv::Point2f(static_cast<float>(pts[br].x), static_cast<float>(pts[br].y)); // Bottom-Right
    ordered[3] = cv::Point2f(static_cast<float>(pts[bl].x), static_cast<float>(pts[bl].y)); // Bottom-Left
    return ordered;
}

std::vector<cv::Point2f> orderPentagonPoints(const std::vector<cv::Point>& pts) {
    if (pts.size() != 5) return {};
    float cx = 0.0f, cy = 0.0f;
    for (const auto& p : pts) {
        cx += static_cast<float>(p.x);
        cy += static_cast<float>(p.y);
    }
    cx /= static_cast<float>(pts.size());
    cy /= static_cast<float>(pts.size());

    struct AngledPt {
        cv::Point2f pt;
        float angle;
    };
    std::vector<AngledPt> angled(pts.size());
    for (size_t i = 0; i < pts.size(); i++) {
        angled[i].pt = cv::Point2f(static_cast<float>(pts[i].x), static_cast<float>(pts[i].y));
        angled[i].angle = std::atan2(static_cast<float>(pts[i].y) - cy, static_cast<float>(pts[i].x) - cx);
    }

    std::sort(angled.begin(), angled.end(), [](const AngledPt& a, const AngledPt& b) {
        return a.angle < b.angle;
    });

    float targetAngle = -static_cast<float>(CV_PI) / 2.0f; // Top peak (-90 degrees)
    int bestIdx = 0;
    float minDiff = 1e9f;
    for (int i = 0; i < static_cast<int>(pts.size()); i++) {
        float diff = std::abs(angled[i].angle - targetAngle);
        if (diff > static_cast<float>(CV_PI)) diff = 2.0f * static_cast<float>(CV_PI) - diff;
        if (diff < minDiff) {
            minDiff = diff;
            bestIdx = i;
        }
    }

    std::vector<cv::Point2f> ordered(pts.size());
    for (size_t i = 0; i < pts.size(); i++) {
        ordered[i] = angled[(bestIdx + i) % pts.size()].pt;
    }
    return ordered;
}

std::string classifyShape(const std::vector<cv::Point>& contour) {
    double area = cv::contourArea(contour);
    if (area < 300) return "Unknown";

    double perim = cv::arcLength(contour, true);
    if (perim == 0) return "Unknown";

    double circularity = 4.0 * CV_PI * area / (perim * perim);

    cv::Point2f center;
    float radius;
    cv::minEnclosingCircle(contour, center, radius);
    double circleArea = CV_PI * radius * radius;
    double fillRatio = (circleArea > 0) ? (area / circleArea) : 0;

    std::vector<cv::Point> approx;
    cv::approxPolyDP(contour, approx, 0.02 * perim, true);
    size_t n = approx.size();

    // 1. Circle test
    if (n >= 7 && circularity > 0.75 && fillRatio > 0.7) {
        return "Circle";
    }

    // 2. Polygon approximation
    if (n == 3) {
        return "Triangle";
    } else if (n == 4) {
        cv::Rect rect = cv::boundingRect(approx);
        float ar = (float)rect.width / rect.height;
        if (ar > 0.9 && ar < 1.1) return "Square";
        else return "Rectangle";
    } else if (n == 5) {
        cv::Rect rect = cv::boundingRect(approx);
        float ar = (float)rect.width / rect.height;
        if (ar > 0.7 && ar < 1.3 && circularity > 0.3) return "Pentagon";
    }

    return "Unknown";
}

GateDetector::GateDetector(double focal_length)
    : focal_length_(focal_length) {
    
    // 3D model points in SI METERS for Square gate
    float sqHalfW_m = static_cast<float>(KNOWN_SQUARE_WIDTH_M / 2.0);
    float sqHalfH_m = sqHalfW_m;
    square_object_points_ = {
        {-sqHalfW_m, -sqHalfH_m, 0.0f}, // Top-Left
        { sqHalfW_m, -sqHalfH_m, 0.0f}, // Top-Right
        { sqHalfW_m,  sqHalfH_m, 0.0f}, // Bottom-Right
        {-sqHalfW_m,  sqHalfH_m, 0.0f}  // Bottom-Left
    };

    // 3D model points in SI METERS for regular Pentagon gate
    float pentR_m = static_cast<float>(KNOWN_PENTAGON_WIDTH_M / (2.0f * std::cos(18.0f * (float)CV_PI / 180.0f)));
    pentagon_object_points_ = {
        { 0.0f,                                              -pentR_m,                                              0.0f }, // 0: Top Peak (-90 deg)
        { pentR_m * std::cos(18.0f * (float)CV_PI / 180.0f), -pentR_m * std::sin(18.0f * (float)CV_PI / 180.0f), 0.0f }, // 1: Upper-Right (-18 deg)
        { pentR_m * std::cos(54.0f * (float)CV_PI / 180.0f),  pentR_m * std::sin(54.0f * (float)CV_PI / 180.0f), 0.0f }, // 2: Bottom-Right (+54 deg)
        {-pentR_m * std::cos(54.0f * (float)CV_PI / 180.0f),  pentR_m * std::sin(54.0f * (float)CV_PI / 180.0f), 0.0f }, // 3: Bottom-Left (+126 deg)
        {-pentR_m * std::cos(18.0f * (float)CV_PI / 180.0f), -pentR_m * std::sin(18.0f * (float)CV_PI / 180.0f), 0.0f }  // 4: Upper-Left (-162 deg)
    };
}

GateDetectionResult GateDetector::processFrame(cv::Mat& frame, bool draw_overlay) {
    GateDetectionResult result;
    if (frame.empty()) return result;

    cv::Mat gray, blur, thresh;

    // Camera Intrinsic Matrix & Distortion Coefficients
    cv::Mat cameraMatrix = (cv::Mat_<double>(3, 3) <<
        focal_length_, 0.0, frame.cols / 2.0,
        0.0, focal_length_, frame.rows / 2.0,
        0.0, 0.0, 1.0
    );
    cv::Mat distCoeffs = cv::Mat::zeros(4, 1, CV_64F);

    // 1. Preprocessing (Grayscale, Blur, Canny Edge Detection)
    cv::cvtColor(frame, gray, cv::COLOR_BGR2GRAY);
    cv::GaussianBlur(gray, blur, cv::Size(5, 5), 0);
    cv::Canny(blur, thresh, 50, 150);

    // Dilate to connect micro-gaps
    cv::Mat kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3, 3));
    cv::dilate(thresh, thresh, kernel, cv::Point(-1, -1), 1);

    // 2. Find Contours with Hierarchy
    std::vector<std::vector<cv::Point>> contours;
    std::vector<cv::Vec4i> hierarchy;
    cv::findContours(thresh, contours, hierarchy, cv::RETR_TREE, cv::CHAIN_APPROX_SIMPLE);

    // 3. Gate Hole Detection (Parent/Child Hierarchy)
    if (hierarchy.empty()) return result;

    for (size_t i = 0; i < contours.size(); i++) {
        double parentArea = cv::contourArea(contours[i]);
        if (parentArea < 3000 || parentArea > (frame.cols * frame.rows * 0.9)) continue;

        int childIdx = hierarchy[i][2];
        while (childIdx != -1) {
            double childArea = cv::contourArea(contours[childIdx]);
            
            if (childArea >= 1000) {
                double holeRatio = childArea / parentArea;
                
                if (holeRatio >= 0.15 && holeRatio <= 0.95) {
                    std::string shapeName = classifyShape(contours[childIdx]);

                    if (shapeName != "Unknown") {
                        result.detected = true;
                        result.shape = shapeName;

                        if (draw_overlay) {
                            // Outer gate frame: Green, Inner hole boundary: Red
                            cv::drawContours(frame, contours, (int)i, cv::Scalar(0, 255, 0), 2);
                            cv::drawContours(frame, contours, childIdx, cv::Scalar(0, 0, 255), 2);
                        }

                        cv::Moments m = cv::moments(contours[childIdx]);
                        if (m.m00 != 0) {
                            cv::Point center(static_cast<int>(m.m10 / m.m00), static_cast<int>(m.m01 / m.m00));
                            result.center = center;

                            if (draw_overlay) {
                                cv::line(frame, cv::Point(center.x - 10, center.y), cv::Point(center.x + 10, center.y), cv::Scalar(0, 0, 255), 2);
                                cv::line(frame, cv::Point(center.x, center.y - 10), cv::Point(center.x, center.y + 10), cv::Scalar(0, 0, 255), 2);
                                cv::putText(frame, "Gate: " + shapeName, cv::Point(center.x - 30, center.y - 15), 
                                            cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255, 255, 255), 2);
                            }

                            // Polygon approximation to extract corners for solvePnP
                            double perim = cv::arcLength(contours[childIdx], true);
                            std::vector<cv::Point> approx;
                            cv::approxPolyDP(contours[childIdx], approx, 0.03 * perim, true);

                            std::vector<cv::Point3f> currentObjectPoints;
                            std::vector<cv::Point2f> currentImagePoints;
                            float currentHalfWidth_m = static_cast<float>(KNOWN_SQUARE_WIDTH_M / 2.0);
                            bool canSolvePnP = false;

                            if (approx.size() == 4) {
                                currentObjectPoints = square_object_points_;
                                currentImagePoints = orderQuadPoints(approx);
                                currentHalfWidth_m = static_cast<float>(KNOWN_SQUARE_WIDTH_M / 2.0);
                                canSolvePnP = true;
                                result.goal_id = 1; // Gate 1: Square
                            } else if (approx.size() == 5) {
                                currentObjectPoints = pentagon_object_points_;
                                currentImagePoints = orderPentagonPoints(approx);
                                currentHalfWidth_m = static_cast<float>(KNOWN_PENTAGON_WIDTH_M / 2.0);
                                canSolvePnP = true;
                                result.goal_id = 2; // Gate 2: Pentagon
                            }

                            if (canSolvePnP) {
                                cv::Mat rvec, tvec;
                                bool solved = cv::solvePnP(currentObjectPoints, currentImagePoints, cameraMatrix, distCoeffs, rvec, tvec);
                                if (solved) {
                                    result.is_3d = true;
                                    result.confidence = 0.95;

                                    // 3D Position in SI METERS
                                    result.posX_m = tvec.at<double>(0);
                                    result.posY_m = tvec.at<double>(1);
                                    result.posZ_m = tvec.at<double>(2);

                                    // Extract 3D Orientation (Pitch, Yaw, Roll)
                                    cv::Mat rotMat;
                                    cv::Rodrigues(rvec, rotMat);

                                    // Radians for shared protocol
                                    result.pitch_rad = std::atan2(-rotMat.at<double>(2, 0),
                                        std::sqrt(rotMat.at<double>(0, 0) * rotMat.at<double>(0, 0) + rotMat.at<double>(1, 0) * rotMat.at<double>(1, 0)));
                                    result.yaw_rad = std::atan2(rotMat.at<double>(1, 0), rotMat.at<double>(0, 0));
                                    result.roll_rad = std::atan2(rotMat.at<double>(2, 1), rotMat.at<double>(2, 2));

                                    // Degrees for display / HUD
                                    result.pitch_deg = result.pitch_rad * (180.0 / CV_PI);
                                    result.yaw_deg = result.yaw_rad * (180.0 / CV_PI);
                                    result.roll_deg = result.roll_rad * (180.0 / CV_PI);

                                    if (draw_overlay) {
                                        // Draw 3D coordinate frame axes directly projected onto gate (X: Red, Y: Green, Z: Blue)
                                        cv::drawFrameAxes(frame, cameraMatrix, distCoeffs, rvec, tvec, currentHalfWidth_m * 0.7f, 2);

                                        // Display 3D telemetry in SI METERS with inch reference
                                        std::string posText = "Pos(X,Y,Z): " + 
                                            std::to_string(result.posX_m).substr(0, 5) + ", " + 
                                            std::to_string(result.posY_m).substr(0, 5) + ", " + 
                                            std::to_string(result.posZ_m).substr(0, 5) + " m (" +
                                            std::to_string(result.posX_m * METERS_TO_INCHES).substr(0, 4) + ", " +
                                            std::to_string(result.posY_m * METERS_TO_INCHES).substr(0, 4) + ", " +
                                            std::to_string(result.posZ_m * METERS_TO_INCHES).substr(0, 4) + " in)";
                                        std::string rotText = "Rot(P,Y,R): " + 
                                            std::to_string((int)result.pitch_deg) + ", " + 
                                            std::to_string((int)result.yaw_deg) + ", " + 
                                            std::to_string((int)result.roll_deg) + " deg";

                                        cv::putText(frame, posText, cv::Point(center.x - 70, center.y + 20), 
                                                    cv::FONT_HERSHEY_SIMPLEX, 0.45, cv::Scalar(0, 255, 255), 2);
                                        cv::putText(frame, rotText, cv::Point(center.x - 70, center.y + 40), 
                                                    cv::FONT_HERSHEY_SIMPLEX, 0.45, cv::Scalar(255, 200, 0), 2);
                                    }

                                    return result;
                                }
                            } else {
                                // Fallback 1D pinhole distance for shapes without 4 or 5 corners (e.g. Circle)
                                cv::Rect box = cv::boundingRect(contours[childIdx]);
                                double distance_m = calculateDistance(KNOWN_SQUARE_WIDTH_M, focal_length_, box.width);
                                result.is_3d = false;
                                result.confidence = 0.70;
                                result.goal_id = 3; // Gate 3: Circle / Unknown
                                result.posX_m = (center.x - frame.cols / 2.0) * (distance_m / focal_length_);
                                result.posY_m = (center.y - frame.rows / 2.0) * (distance_m / focal_length_);
                                result.posZ_m = distance_m;
                                result.pitch_rad = 0.0;
                                result.yaw_rad = 0.0;
                                result.roll_rad = 0.0;
                                result.pitch_deg = 0.0;
                                result.yaw_deg = 0.0;
                                result.roll_deg = 0.0;

                                if (draw_overlay) {
                                    std::string distText = "Dist (1D): " + std::to_string(distance_m).substr(0, 5) + " m (" +
                                        std::to_string(distance_m * METERS_TO_INCHES).substr(0, 4) + " in)";
                                    cv::putText(frame, distText, cv::Point(center.x - 60, center.y + 20), 
                                                cv::FONT_HERSHEY_SIMPLEX, 0.45, cv::Scalar(255, 100, 0), 2);
                                }

                                return result;
                            }
                        }
                    }
                }
            }
            childIdx = hierarchy[childIdx][0];
        }
    }

    return result;
}

VisionPacket GateDetector::toVisionPacket(const GateDetectionResult& result, uint32_t sequence_id, uint64_t timestamp_ns) {
    VisionPacket packet{};
    packet.msg_type = MSG_TYPE_VISION;
    packet.sequence_id = sequence_id;
    packet.timestamp_ns = timestamp_ns;
    packet.error_state = result.detected ? VISION_ERR_NONE : VISION_ERR_FRAME_DROPPED;
    packet.num_tags_detected = 0;

    if (result.detected) {
        packet.num_goals_reported = 1;
        GoalMeasurement& gm = packet.goals[0];

        if (result.shape == "Circle") {
            gm.shape_type = SHAPE_CIRCLE;
        } else if (result.shape == "Triangle") {
            gm.shape_type = SHAPE_TRIANGLE;
        } else {
            gm.shape_type = SHAPE_SQUARE;
        }

        gm.is_detected = 1;
        gm.centroid.u = static_cast<float>(result.center.x);
        gm.centroid.v = static_cast<float>(result.center.y);
        gm.contour_area = 0.0f;
        gm.num_hypotheses = 1;

        PoseHypothesis& hyp = gm.hypotheses[0];
        hyp.position_cam.x = static_cast<float>(result.posX_m);
        hyp.position_cam.y = static_cast<float>(result.posY_m);
        hyp.position_cam.z = static_cast<float>(result.posZ_m);
        hyp.orientation_cam = eulerToQuaternion(result.roll_rad, result.pitch_rad, result.yaw_rad);
        hyp.confidence = static_cast<float>(result.confidence);
    } else {
        packet.num_goals_reported = 0;
    }

    return packet;
}

} // namespace vision
