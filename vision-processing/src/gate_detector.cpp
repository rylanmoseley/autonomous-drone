#include "gate_detector.h"
#include <opencv2/imgproc.hpp>
#include <opencv2/calib3d.hpp>

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

std::string classifyShape(const cv::Mat& frame, const std::vector<cv::Point>& contour) {
    double area = cv::contourArea(contour);
    if (area < 25) return "Unknown";

    double perim = cv::arcLength(contour, true);
    if (perim == 0) return "Unknown";

    cv::Rect bbox = cv::boundingRect(contour);
    if (bbox.width < 4 || bbox.height < 4) return "Unknown";

    // Color sample to differentiate simulated/real gates
    if (!frame.empty()) {
        int blue_count = 0;
        int red_count = 0;
        int green_count = 0;
        int total_checked = 0;

        int x0 = std::max(0, bbox.x);
        int y0 = std::max(0, bbox.y);
        int x1 = std::min(frame.cols, bbox.x + bbox.width);
        int y1 = std::min(frame.rows, bbox.y + bbox.height);

        for (int y = y0; y < y1; y += 2) {
            const cv::Vec3b* row = frame.ptr<cv::Vec3b>(y);
            for (int x = x0; x < x1; x += 2) {
                int b = row[x][0];
                int g = row[x][1];
                int r = row[x][2];
                total_checked++;
                if (b > r + 30 && b > g + 30) blue_count++;
                else if (r > g + 30 && r > b + 30) red_count++;
                else if (g > r + 30 && g > b + 30) green_count++;
            }
        }

        if (blue_count >= 6 && blue_count > red_count && blue_count > green_count) {
            return "Circle";
        }
        if (red_count >= 6 && red_count > blue_count && red_count > green_count) {
            return "Triangle";
        }
        if (green_count >= 6 && green_count > blue_count && green_count > red_count) {
            return "Square";
        }
    }

    // Geometric shape discrimination fallback
    double circularity = 4.0 * CV_PI * area / (perim * perim);
    cv::Point2f center;
    float radius;
    cv::minEnclosingCircle(contour, center, radius);
    double circleArea = CV_PI * radius * radius;
    double fillRatio = (circleArea > 0) ? (area / circleArea) : 0;

    std::vector<cv::Point> approx;
    cv::approxPolyDP(contour, approx, 0.03 * perim, true);
    size_t n = approx.size();

    // 1. Polygon approximation
    if (n == 3) {
        return "Triangle";
    } else if (n == 4) {
        float ar = (float)bbox.width / bbox.height;
        if (ar >= 0.85 && ar <= 1.15) return "Square";
        else return "Rectangle";
    } else if (n == 5) {
        float ar = (float)bbox.width / bbox.height;
        if (ar > 0.7 && ar < 1.3 && circularity > 0.6) return "Pentagon";
    }

    if (n >= 6 && circularity > 0.65 && fillRatio > 0.60) {
        return "Circle";
    }
    if (n <= 4 && circularity < 0.65 && fillRatio < 0.55) {
        return "Triangle";
    }
    if (circularity > 0.55 && fillRatio > 0.50) {
        return "Circle";
    }

    return "Unknown";
}

std::string classifyShape(const std::vector<cv::Point>& contour) {
    cv::Mat dummy;
    return classifyShape(dummy, contour);
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

std::vector<GateDetectionResult> GateDetector::detectAllGates(cv::Mat& frame, bool draw_overlay) {
    std::vector<GateDetectionResult> results;
    if (frame.empty()) return results;

    bool is_sim = is_sim_ || (frame.cols == 320 && frame.rows == 240);
    double effective_focal = is_sim ? (frame.rows / (2.0 * std::tan(35.0 * CV_PI / 180.0))) : focal_length_;

    if (is_sim) {
        // High-precision simulation gate segmentation based on distinct Three.js hoop colors:
        // Gate 1 (Circle):   Pure Blue (0x0000ff) -> B > 65 && (B - R) > 30 && (B - G) > 30
        // Gate 2 (Triangle): Pure Red  (0xff0000) -> R > 55 && (R - B) > 25 && (R - G) > 25
        // Gate 3 (Square):   Pure Green(0x00ff00) -> G > 55 && (G - R) > 25 && (G - B) > 25

        struct SimGateSpec {
            std::string shape;
            int goal_id;
            double known_dim; // physical outer width/height in meters
            cv::Scalar draw_color;
        };

        const std::vector<SimGateSpec> specs = {
            {"Circle",   1, 0.500, cv::Scalar(255, 100, 0)},
            {"Triangle", 2, 0.666, cv::Scalar(0, 0, 255)},
            {"Square",   3, 0.414, cv::Scalar(0, 255, 0)}
        };

        cv::Mat mask_blue  = cv::Mat::zeros(frame.size(), CV_8UC1);
        cv::Mat mask_red   = cv::Mat::zeros(frame.size(), CV_8UC1);
        cv::Mat mask_green = cv::Mat::zeros(frame.size(), CV_8UC1);

        for (int y = 0; y < frame.rows; ++y) {
            const cv::Vec3b* ptr = frame.ptr<cv::Vec3b>(y);
            uchar* mb = mask_blue.ptr<uchar>(y);
            uchar* mr = mask_red.ptr<uchar>(y);
            uchar* mg = mask_green.ptr<uchar>(y);
            for (int x = 0; x < frame.cols; ++x) {
                int b = ptr[x][0];
                int g = ptr[x][1];
                int r = ptr[x][2];
                if (b > 65 && (b - r) > 30 && (b - g) > 30) {
                    mb[x] = 255;
                } else if (r > 55 && (r - b) > 25 && (r - g) > 25) {
                    mr[x] = 255;
                } else if (g > 55 && (g - r) > 25 && (g - b) > 25) {
                    mg[x] = 255;
                }
            }
        }

        cv::Mat kernel = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5, 5));
        cv::morphologyEx(mask_blue, mask_blue, cv::MORPH_CLOSE, kernel);
        cv::morphologyEx(mask_red, mask_red, cv::MORPH_CLOSE, kernel);
        cv::morphologyEx(mask_green, mask_green, cv::MORPH_CLOSE, kernel);

        for (const auto& spec : specs) {
            const cv::Mat& mask = (spec.shape == "Circle") ? mask_blue : ((spec.shape == "Triangle") ? mask_red : mask_green);
            std::vector<std::vector<cv::Point>> contours;
            cv::findContours(mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

            int best_idx = -1;
            double max_area = 0.0;
            for (size_t i = 0; i < contours.size(); ++i) {
                double a = cv::contourArea(contours[i]);
                if (a > max_area) {
                    max_area = a;
                    best_idx = static_cast<int>(i);
                }
            }

            if (best_idx < 0 || max_area < 45.0) continue;

            const auto& contour = contours[best_idx];
            cv::Rect bbox = cv::boundingRect(contour);

            // Reject marginal peripheral contours clipped by the camera frame boundaries,
            // but permit large centered contours when the vehicle is close to the gate.
            bool touches_edge = (bbox.x <= 2 || bbox.y <= 2 ||
                                 bbox.x + bbox.width >= frame.cols - 2 ||
                                 bbox.y + bbox.height >= frame.rows - 2);
            if (touches_edge) {
                // Peripheral slivers entering or exiting the frame have off-center centroid or small dimension
                double c_x = bbox.x + bbox.width / 2.0;
                double c_y = bbox.y + bbox.height / 2.0;
                bool is_centered = (std::abs(c_x - frame.cols / 2.0) < 140.0 && std::abs(c_y - frame.rows / 2.0) < 120.0);
                bool is_large = (bbox.width > 200 || bbox.height > 180);
                if (!is_centered || !is_large) {
                    continue;
                }
            }

            // Aspect ratio check to reject occluded slivers or partially cut off contours
            double aspect = static_cast<double>(bbox.width) / std::max(1, bbox.height);
            if (spec.shape == "Triangle") {
                if (aspect < 0.70 || aspect > 1.60) continue;
            } else if (spec.shape == "Square") {
                if (aspect < 0.75 || aspect > 1.30) continue;
            } else if (aspect < 0.65 || aspect > 1.45) {
                continue;
            }

            cv::Moments m = cv::moments(contour);
            double cx = (m.m00 > 0) ? (m.m10 / m.m00) : (bbox.x + bbox.width / 2.0);
            double cy = (m.m00 > 0) ? (m.m01 / m.m00) : (bbox.y + bbox.height / 2.0);
            cv::Point center(static_cast<int>(std::round(cx)), static_cast<int>(std::round(cy)));

            double gate_size_px = std::max(bbox.width, bbox.height);
            double distance_m = calculateDistance(spec.known_dim, effective_focal, gate_size_px);
            if (distance_m > 4.5 || distance_m < 0.2) {
                continue; // Reject out-of-range false detections
            }

            GateDetectionResult res;
            res.detected = true;
            res.shape = spec.shape;
            res.center = center;
            res.goal_id = spec.goal_id;
            res.bbox_size = gate_size_px;
            res.posX_m = (cx - frame.cols / 2.0) * (distance_m / effective_focal);
            res.posY_m = (cy - frame.rows / 2.0) * (distance_m / effective_focal);
            if (spec.shape == "Triangle") {
                if (bbox.y <= 2) {
                    // Apex is clipped at top of camera frame at close range (< 0.95m).
                    // Reconstruct true aperture center from the unclipped bottom base:
                    // For an upright equilateral triangle (width W=0.666m, base-to-center H_bc=0.205m):
                    // ratio_base_to_center = 0.205 / 0.666 ~ 0.308
                    double y_base = bbox.y + bbox.height;
                    double cy_reconstructed = y_base - 0.308 * gate_size_px;
                    res.posY_m = (cy_reconstructed - frame.rows / 2.0) * (distance_m / effective_focal);
                    res.center.y = static_cast<int>(std::round(cy_reconstructed));
                }
            }
            res.posZ_m = distance_m;
            res.confidence = touches_edge ? 0.80 : 0.95;
            res.is_3d = false;

            if (draw_overlay) {
                cv::drawContours(frame, contours, best_idx, spec.draw_color, 2);
                cv::rectangle(frame, bbox, cv::Scalar(255, 255, 0), 1);
                cv::circle(frame, center, 3, cv::Scalar(0, 0, 255), -1);
                std::string label = spec.shape + " " + std::to_string(res.posZ_m).substr(0, 4) + "m";
                cv::putText(frame, label, cv::Point(center.x - 25, center.y - 10),
                            cv::FONT_HERSHEY_SIMPLEX, 0.45, cv::Scalar(255, 255, 255), 1);
            }

            results.push_back(res);
        }

        return results;
    }

    double known_width = KNOWN_SQUARE_WIDTH_M;

    cv::Mat gray, blur, thresh;
    if (is_sim) {
        std::vector<cv::Mat> channels;
        cv::split(frame, channels);
        cv::max(channels[0], channels[1], gray);
        cv::max(gray, channels[2], gray);
        cv::GaussianBlur(gray, blur, cv::Size(3, 3), 0);
        cv::Canny(blur, thresh, 25, 75);
    } else {
        cv::cvtColor(frame, gray, cv::COLOR_BGR2GRAY);
        cv::GaussianBlur(gray, blur, cv::Size(5, 5), 0);
        cv::Canny(blur, thresh, 30, 90);
    }

    cv::Mat kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3, 3));
    cv::dilate(thresh, thresh, kernel, cv::Point(-1, -1), 1);

    std::vector<std::vector<cv::Point>> contours;
    std::vector<cv::Vec4i> hierarchy;
    cv::findContours(thresh, contours, hierarchy, cv::RETR_TREE, cv::CHAIN_APPROX_SIMPLE);

    if (contours.empty()) return results;

    double minArea = is_sim ? 25.0 : std::max(50.0, frame.cols * frame.rows * 0.002);
    double maxArea = frame.cols * frame.rows * 0.95;

    // Track best candidate contour per shape type ("Circle", "Triangle", "Square")
    struct Candidate {
        int contour_idx = -1;
        double area = 0.0;
        cv::Point center{0, 0};
        cv::Rect bbox;
    };
    std::map<std::string, Candidate> best_by_shape;

    for (size_t i = 0; i < contours.size(); ++i) {
        double area = cv::contourArea(contours[i]);
        if (area < minArea || area > maxArea) continue;

        std::string shape = classifyShape(frame, contours[i]);
        if (shape == "Unknown") continue;

        cv::Moments m = cv::moments(contours[i]);
        if (m.m00 == 0) continue;
        cv::Point center(static_cast<int>(m.m10 / m.m00), static_cast<int>(m.m01 / m.m00));
        cv::Rect bbox = cv::boundingRect(contours[i]);

        if (best_by_shape.find(shape) == best_by_shape.end() || area > best_by_shape[shape].area) {
            best_by_shape[shape] = Candidate{static_cast<int>(i), area, center, bbox};
        }
    }

    // Camera matrix for solvePnP if needed
    double camData[9] = {
        effective_focal, 0.0, frame.cols / 2.0,
        0.0, effective_focal, frame.rows / 2.0,
        0.0, 0.0, 1.0
    };
    cv::Mat cameraMatrix = cv::Mat(3, 3, CV_64F, camData).clone();
    cv::Mat distCoeffs = cv::Mat::zeros(4, 1, CV_64F);

    // Shapes in mission priority order: Circle (1), Triangle (2), Square (3)
    std::vector<std::string> priority_shapes = {"Circle", "Triangle", "Square"};
    for (const auto& sh : priority_shapes) {
        auto it = best_by_shape.find(sh);
        if (it == best_by_shape.end()) continue;

        const auto& cand = it->second;
        GateDetectionResult res;
        res.detected = true;
        res.shape = sh;
        res.center = cand.center;
        res.goal_id = (sh == "Circle") ? 1 : ((sh == "Triangle") ? 2 : 3);

        double gate_size_px = std::max(cand.bbox.width, cand.bbox.height);
        double distance_m = calculateDistance(known_width, effective_focal, gate_size_px);

        res.posX_m = (cand.center.x - frame.cols / 2.0) * (distance_m / effective_focal);
        res.posY_m = (cand.center.y - frame.rows / 2.0) * (distance_m / effective_focal);
        res.posZ_m = distance_m;
        res.confidence = 0.90;
        res.is_3d = false;
        res.bbox_size = gate_size_px;

        // Try solvePnP for real/non-sim camera or polygons
        if (!is_sim && cand.contour_idx >= 0) {
            double perim = cv::arcLength(contours[cand.contour_idx], true);
            std::vector<cv::Point> approx;
            cv::approxPolyDP(contours[cand.contour_idx], approx, 0.03 * perim, true);
            if (approx.size() == 4) {
                auto objPts = square_object_points_;
                auto imgPts = orderQuadPoints(approx);
                cv::Mat rvec, tvec;
                if (cv::solvePnP(objPts, imgPts, cameraMatrix, distCoeffs, rvec, tvec)) {
                    res.is_3d = true;
                    res.confidence = 0.95;
                    res.posX_m = tvec.at<double>(0);
                    res.posY_m = tvec.at<double>(1);
                    res.posZ_m = tvec.at<double>(2);
                    cv::Mat rotMat;
                    cv::Rodrigues(rvec, rotMat);
                    res.pitch_rad = std::atan2(-rotMat.at<double>(2, 0),
                        std::sqrt(rotMat.at<double>(0, 0) * rotMat.at<double>(0, 0) + rotMat.at<double>(1, 0) * rotMat.at<double>(1, 0)));
                    res.yaw_rad = std::atan2(rotMat.at<double>(1, 0), rotMat.at<double>(0, 0));
                    res.roll_rad = std::atan2(rotMat.at<double>(2, 1), rotMat.at<double>(2, 2));
                    res.pitch_deg = res.pitch_rad * (180.0 / CV_PI);
                    res.yaw_deg = res.yaw_rad * (180.0 / CV_PI);
                    res.roll_deg = res.roll_rad * (180.0 / CV_PI);
                }
            }
        }

        if (draw_overlay) {
            if (cand.contour_idx >= 0) {
                cv::Scalar col = (sh == "Circle") ? cv::Scalar(255, 100, 0) : ((sh == "Triangle") ? cv::Scalar(0, 0, 255) : cv::Scalar(0, 255, 0));
                cv::drawContours(frame, contours, cand.contour_idx, col, 2);
            }
            cv::rectangle(frame, cand.bbox, cv::Scalar(255, 255, 0), 1);
            cv::circle(frame, cand.center, 3, cv::Scalar(0, 0, 255), -1);
            std::string label = sh + " " + std::to_string(res.posZ_m).substr(0, 4) + "m";
            cv::putText(frame, label, cv::Point(cand.center.x - 25, cand.center.y - 10),
                        cv::FONT_HERSHEY_SIMPLEX, 0.45, cv::Scalar(255, 255, 255), 1);
        }

        results.push_back(res);
    }

    return results;
}

GateDetectionResult GateDetector::processFrame(cv::Mat& frame, bool draw_overlay) {
    auto results = detectAllGates(frame, draw_overlay);
    if (!results.empty()) {
        return results[0];
    }
    return GateDetectionResult{};
}

VisionPacket GateDetector::toVisionPacket(const std::vector<GateDetectionResult>& results, uint32_t sequence_id, uint64_t timestamp_ns) {
    VisionPacket packet{};
    packet.msg_type = MSG_TYPE_VISION;
    packet.sequence_id = sequence_id;
    packet.timestamp_ns = timestamp_ns;
    packet.error_state = results.empty() ? VISION_ERR_FRAME_DROPPED : VISION_ERR_NONE;
    packet.num_tags_detected = 0;
    packet.num_goals_reported = static_cast<uint8_t>(std::min((size_t)3, results.size()));

    for (uint8_t i = 0; i < packet.num_goals_reported; ++i) {
        const auto& res = results[i];
        GoalMeasurement& gm = packet.goals[i];
        if (res.shape == "Circle") {
            gm.shape_type = SHAPE_CIRCLE;
        } else if (res.shape == "Triangle") {
            gm.shape_type = SHAPE_TRIANGLE;
        } else {
            gm.shape_type = SHAPE_SQUARE;
        }
        gm.is_detected = 1;
        gm.centroid.u = static_cast<float>(res.center.x);
        gm.centroid.v = static_cast<float>(res.center.y);
        gm.contour_area = static_cast<float>(res.bbox_size);
        gm.num_hypotheses = 1;

        PoseHypothesis& hyp = gm.hypotheses[0];
        hyp.position_cam.x = static_cast<float>(res.posX_m);
        hyp.position_cam.y = static_cast<float>(res.posY_m);
        hyp.position_cam.z = static_cast<float>(res.posZ_m);
        hyp.orientation_cam = eulerToQuaternion(res.roll_rad, res.pitch_rad, res.yaw_rad);
        hyp.confidence = static_cast<float>(res.confidence);
    }
    return packet;
}

VisionPacket GateDetector::toVisionPacket(const GateDetectionResult& result, uint32_t sequence_id, uint64_t timestamp_ns) {
    if (result.detected) {
        return toVisionPacket(std::vector<GateDetectionResult>{result}, sequence_id, timestamp_ns);
    }
    VisionPacket packet{};
    packet.msg_type = MSG_TYPE_VISION;
    packet.sequence_id = sequence_id;
    packet.timestamp_ns = timestamp_ns;
    packet.error_state = VISION_ERR_FRAME_DROPPED;
    packet.num_tags_detected = 0;
    packet.num_goals_reported = 0;
    return packet;
}

} // namespace vision
