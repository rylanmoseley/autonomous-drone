// =========================================================================================
// Drone Vision System: 6-DoF Gate Pose Estimation via cv::solvePnP
// =========================================================================================
// Computes full 3D position (X, Y, Z) and 3D orientation (Pitch, Yaw, Roll) of gates:
//   - Uses calibrated camera matrix K (FOCAL_LENGTH = 566.8)
//   - Matches detected 2D corners from approxPolyDP to known 3D gate dimensions
//   - Projects 3D coordinate frame axes directly onto the gate opening
// =========================================================================================

#include <iostream>
#include <vector>
#include <string>
#include <cmath>
#include <algorithm>
#include <opencv2/opencv.hpp>

// --- Calibration & Gate Dimensions ---
const double FOCAL_LENGTH = 566.8;
const double KNOWN_SQUARE_WIDTH   = 11.25; // Physical width/height of the square opening
const double KNOWN_PENTAGON_WIDTH = 11.75; // Physical max width (across upper corners) of pentagon opening

// 1D distance calculation fallback
double calculateDistance(double knownWidth, double focalLength, double perWidthPixels) {
    return (knownWidth * focalLength) / perWidthPixels;
}

// Order 4 quadrilateral corners: [Top-Left, Top-Right, Bottom-Right, Bottom-Left]
std::vector<cv::Point2f> orderQuadPoints(const std::vector<cv::Point>& pts) {
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

// Order 5 pentagon corners clockwise matching the 3D model (starting at Top Peak ~ -90 deg)
std::vector<cv::Point2f> orderPentagonPoints(const std::vector<cv::Point>& pts) {
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
    for (int i = 0; i < 5; i++) {
        float diff = std::abs(angled[i].angle - targetAngle);
        if (diff > static_cast<float>(CV_PI)) diff = 2.0f * static_cast<float>(CV_PI) - diff;
        if (diff < minDiff) {
            minDiff = diff;
            bestIdx = i;
        }
    }

    std::vector<cv::Point2f> ordered(5);
    for (int i = 0; i < 5; i++) {
        ordered[i] = angled[(bestIdx + i) % 5].pt;
    }
    return ordered;
}

// Advanced classification using Circularity, Fill Ratio, and aspect ratio checks
std::string classifyShape(const std::vector<cv::Point>& contour) {
    double area = cv::contourArea(contour);
    if (area < 300) return "Unknown";

    double perim = cv::arcLength(contour, true);
    if (perim == 0) return "Unknown";

    // Circularity: 1.0 is a perfect circle
    double circularity = 4.0 * CV_PI * area / (perim * perim);

    // Fill ratio for circle
    cv::Point2f center;
    float radius;
    cv::minEnclosingCircle(contour, center, radius);
    double circleArea = CV_PI * radius * radius;
    double fillRatio = (circleArea > 0) ? (area / circleArea) : 0;

    std::vector<cv::Point> approx;
    cv::approxPolyDP(contour, approx, 0.02 * perim, true);
    size_t n = approx.size();

    // 1. Circle test first
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

int main() {
    cv::VideoCapture cap(0);
    if (!cap.isOpened()) {
        std::cerr << "Failed to open camera." << std::endl;
        return -1;
    }

    cv::Mat frame, gray, blur, thresh;

    // 3D model points for a Square gate (Width = Height = KNOWN_SQUARE_WIDTH)
    float sqHalfW = static_cast<float>(KNOWN_SQUARE_WIDTH / 2.0);
    float sqHalfH = sqHalfW;
    std::vector<cv::Point3f> squareObjectPoints = {
        {-sqHalfW, -sqHalfH, 0.0f}, // Top-Left
        { sqHalfW, -sqHalfH, 0.0f}, // Top-Right
        { sqHalfW,  sqHalfH, 0.0f}, // Bottom-Right
        {-sqHalfW,  sqHalfH, 0.0f}  // Bottom-Left
    };

    // 3D model points for a regular Pentagon gate (Width = KNOWN_PENTAGON_WIDTH across widest upper corners)
    // Modeled with Top Peak at -90 deg, flat horizontal bottom baseline
    float pentR = static_cast<float>(KNOWN_PENTAGON_WIDTH / (2.0f * std::cos(18.0f * CV_PI / 180.0f)));
    std::vector<cv::Point3f> pentagonObjectPoints = {
        { 0.0f,                                            -pentR,                                            0.0f }, // 0: Top Peak (-90 deg)
        { pentR * std::cos(18.0f * (float)CV_PI / 180.0f), -pentR * std::sin(18.0f * (float)CV_PI / 180.0f), 0.0f }, // 1: Upper-Right (-18 deg)
        { pentR * std::cos(54.0f * (float)CV_PI / 180.0f),  pentR * std::sin(54.0f * (float)CV_PI / 180.0f), 0.0f }, // 2: Bottom-Right (+54 deg)
        {-pentR * std::cos(54.0f * (float)CV_PI / 180.0f),  pentR * std::sin(54.0f * (float)CV_PI / 180.0f), 0.0f }, // 3: Bottom-Left (+126 deg)
        {-pentR * std::cos(18.0f * (float)CV_PI / 180.0f), -pentR * std::sin(18.0f * (float)CV_PI / 180.0f), 0.0f }  // 4: Upper-Left (-162 deg)
    };

    std::cout << "Advanced Drone Vision with solvePnP started. Press 'q' to exit." << std::endl;

    while (true) {
        cap >> frame;
        if (frame.empty()) break;

        // Camera Intrinsic Matrix & Distortion Coefficients
        cv::Mat cameraMatrix = (cv::Mat_<double>(3, 3) <<
            FOCAL_LENGTH, 0.0, frame.cols / 2.0,
            0.0, FOCAL_LENGTH, frame.rows / 2.0,
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
        if (!hierarchy.empty()) {
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
                                // Draw outer gate frame in Green
                                cv::drawContours(frame, contours, (int)i, cv::Scalar(0, 255, 0), 2);
                                // Draw inner hole boundary in Red
                                cv::drawContours(frame, contours, childIdx, cv::Scalar(0, 0, 255), 2);

                                cv::Moments m = cv::moments(contours[childIdx]);
                                if (m.m00 != 0) {
                                    cv::Point center(m.m10 / m.m00, m.m01 / m.m00);

                                    // Draw bullseye crosshair on the hole center
                                    cv::line(frame, cv::Point(center.x - 10, center.y), cv::Point(center.x + 10, center.y), cv::Scalar(0, 0, 255), 2);
                                    cv::line(frame, cv::Point(center.x, center.y - 10), cv::Point(center.x, center.y + 10), cv::Scalar(0, 0, 255), 2);

                                    cv::putText(frame, "Gate: " + shapeName, cv::Point(center.x - 30, center.y - 15), 
                                                cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255, 255, 255), 2);

                                    // Polygon approximation to extract corners for solvePnP
                                    double perim = cv::arcLength(contours[childIdx], true);
                                    std::vector<cv::Point> approx;
                                    cv::approxPolyDP(contours[childIdx], approx, 0.03 * perim, true);

                                    // Check whether we have a Square (4 corners) or Pentagon (5 corners)
                                    std::vector<cv::Point3f> currentObjectPoints;
                                    std::vector<cv::Point2f> currentImagePoints;
                                    float currentWidth = sqHalfW;
                                    bool canSolvePnP = false;

                                    if (approx.size() == 4) {
                                        currentObjectPoints = squareObjectPoints;
                                        currentImagePoints = orderQuadPoints(approx);
                                        currentWidth = sqHalfW;
                                        canSolvePnP = true;
                                    } else if (approx.size() == 5) {
                                        currentObjectPoints = pentagonObjectPoints;
                                        currentImagePoints = orderPentagonPoints(approx);
                                        currentWidth = static_cast<float>(KNOWN_PENTAGON_WIDTH / 2.0);
                                        canSolvePnP = true;
                                    }

                                    if (canSolvePnP) {
                                        cv::Mat rvec, tvec;
                                        bool solved = cv::solvePnP(currentObjectPoints, currentImagePoints, cameraMatrix, distCoeffs, rvec, tvec);
                                        if (solved) {
                                            // Draw 3D coordinate frame axes directly projected onto gate (X: Red, Y: Green, Z: Blue)
                                            cv::drawFrameAxes(frame, cameraMatrix, distCoeffs, rvec, tvec, currentWidth * 0.7f, 2);

                                            // Extract 3D Position
                                            double posX = tvec.at<double>(0);
                                            double posY = tvec.at<double>(1);
                                            double posZ = tvec.at<double>(2);

                                            // Extract 3D Orientation (Pitch, Yaw, Roll)
                                            cv::Mat rotMat;
                                            cv::Rodrigues(rvec, rotMat);
                                            double pitch = std::atan2(-rotMat.at<double>(2, 0),
                                                std::sqrt(rotMat.at<double>(0, 0) * rotMat.at<double>(0, 0) + rotMat.at<double>(1, 0) * rotMat.at<double>(1, 0))) * (180.0 / CV_PI);
                                            double yaw = std::atan2(rotMat.at<double>(1, 0), rotMat.at<double>(0, 0)) * (180.0 / CV_PI);
                                            double roll = std::atan2(rotMat.at<double>(2, 1), rotMat.at<double>(2, 2)) * (180.0 / CV_PI);

                                            // Display 3D telemetry
                                            std::string posText = "Pos(X,Y,Z): " + 
                                                std::to_string(posX).substr(0, 5) + ", " + 
                                                std::to_string(posY).substr(0, 5) + ", " + 
                                                std::to_string(posZ).substr(0, 5);
                                            std::string rotText = "Rot(P,Y,R): " + 
                                                std::to_string((int)pitch) + ", " + 
                                                std::to_string((int)yaw) + ", " + 
                                                std::to_string((int)roll) + " deg";

                                            cv::putText(frame, posText, cv::Point(center.x - 60, center.y + 20), 
                                                        cv::FONT_HERSHEY_SIMPLEX, 0.45, cv::Scalar(0, 255, 255), 2);
                                            cv::putText(frame, rotText, cv::Point(center.x - 60, center.y + 40), 
                                                        cv::FONT_HERSHEY_SIMPLEX, 0.45, cv::Scalar(255, 200, 0), 2);
                                        }
                                    } else {
                                        // Fallback 1D pinhole distance for shapes without 4 or 5 corners (e.g. Circle)
                                        cv::Rect box = cv::boundingRect(contours[childIdx]);
                                        double distance = calculateDistance(KNOWN_SQUARE_WIDTH, FOCAL_LENGTH, box.width);
                                        std::string distText = "Dist (1D): " + std::to_string(distance).substr(0, 5) + " units";
                                        cv::putText(frame, distText, cv::Point(center.x - 40, center.y + 20), 
                                                    cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255, 100, 0), 2);
                                    }
                                }
                            }
                        }
                    }
                    childIdx = hierarchy[childIdx][0];
                }
            }
        }

        // Display results
        cv::imshow("Advanced Drone Vision (solvePnP)", frame);
        cv::imshow("Threshold Mask", thresh);

        if (cv::waitKey(1) == 'q') break;
    }

    cap.release();
    cv::destroyAllWindows();
    return 0;
}
