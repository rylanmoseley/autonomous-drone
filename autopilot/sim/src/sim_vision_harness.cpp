#include "sim_vision_harness.h"
#include <cmath>
#include <algorithm>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <fcntl.h>

namespace sim {

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

#ifdef SIM_HAS_OPENCV
SimVisionHarness::SimVisionHarness(const SimPerceptionConfig& config)
    : config_(config), detector_(vision::FOCAL_LENGTH) {
    detector_.setSimMode(true);
    initSockets();
}
#else
SimVisionHarness::SimVisionHarness(const SimPerceptionConfig& config)
    : config_(config) {
    initSockets();
}
#endif

SimVisionHarness::~SimVisionHarness() {
    closeSockets();
}

void SimVisionHarness::initSockets() {
    if (!config_.hook_running_simulation) return;

    // UDP transmitter socket to send camera frames to running vision_node (UDP 14555)
    udp_tx_sock_ = socket(AF_INET, SOCK_DGRAM, 0);
    if (udp_tx_sock_ >= 0) {
        int opt = 1;
        setsockopt(udp_tx_sock_, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));
        fcntl(udp_tx_sock_, F_SETFL, O_NONBLOCK);
    }

    // UDP receiver socket to receive feedback telemetry from running vision_node (UDP 14556)
    udp_rx_sock_ = socket(AF_INET, SOCK_DGRAM, 0);
    if (udp_rx_sock_ >= 0) {
        int opt = 1;
        setsockopt(udp_rx_sock_, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));
#ifdef SO_REUSEPORT
        setsockopt(udp_rx_sock_, SOL_SOCKET, SO_REUSEPORT, &opt, sizeof(opt));
#endif
        fcntl(udp_rx_sock_, F_SETFL, O_NONBLOCK);

        struct sockaddr_in rx_addr{};
        rx_addr.sin_family = AF_INET;
        rx_addr.sin_port = htons(config_.running_sim_feedback_port);
        rx_addr.sin_addr.s_addr = INADDR_ANY;
        if (bind(udp_rx_sock_, (struct sockaddr*)&rx_addr, sizeof(rx_addr)) < 0) {
            // If binding fails (e.g. port exclusively held), close RX socket and fall back to in-process detector
            close(udp_rx_sock_);
            udp_rx_sock_ = -1;
        }
    }
}

void SimVisionHarness::closeSockets() {
    if (udp_tx_sock_ >= 0) {
        close(udp_tx_sock_);
        udp_tx_sock_ = -1;
    }
    if (udp_rx_sock_ >= 0) {
        close(udp_rx_sock_);
        udp_rx_sock_ = -1;
    }
}

#ifdef SIM_HAS_OPENCV
void SimVisionHarness::sendFrameToRunningSimulation(const cv::Mat& frame) {
    if (udp_tx_sock_ < 0 || frame.empty()) return;

    std::vector<uchar> buf;
    std::vector<int> params = {cv::IMWRITE_JPEG_QUALITY, 80};
    if (cv::imencode(".jpg", frame, buf, params)) {
        struct sockaddr_in tx_addr{};
        tx_addr.sin_family = AF_INET;
        tx_addr.sin_port = htons(config_.running_sim_rx_port);
        inet_pton(AF_INET, config_.running_sim_ip.c_str(), &tx_addr.sin_addr);
        sendto(udp_tx_sock_, buf.data(), buf.size(), MSG_DONTWAIT,
               (struct sockaddr*)&tx_addr, sizeof(tx_addr));
    }
}
#endif

bool SimVisionHarness::tryReceiveRunningSimulationPacket(VisionPacket& packet) {
    if (udp_rx_sock_ < 0) return false;

    VisionPacket tmp{};
    ssize_t n = recvfrom(udp_rx_sock_, &tmp, sizeof(tmp), MSG_DONTWAIT, nullptr, nullptr);
    if (n == static_cast<ssize_t>(sizeof(tmp)) && tmp.msg_type == MSG_TYPE_VISION) {
        packet = tmp;
        return true;
    }
    return false;
}

#ifdef SIM_HAS_OPENCV
cv::Mat SimVisionHarness::renderFrame(
    const Pose& true_p,
    const std::vector<SimulatedGoal>& goals) const
{
    if (render_buffer_.empty() || render_buffer_.cols != 320 || render_buffer_.rows != 240) {
        render_buffer_ = cv::Mat::zeros(240, 320, CV_8UC3);
    } else {
        render_buffer_.setTo(cv::Scalar(0, 0, 0));
    }

    if (goals.empty()) {
        return render_buffer_;
    }

    double cy = std::cos(true_p.yaw), sy = std::sin(true_p.yaw);
    double cp = std::cos(true_p.pitch), sp = std::sin(true_p.pitch);
    double cr = std::cos(true_p.roll), sr = std::sin(true_p.roll);

    // Drone rotation matrix: Three.js order YZX: R = Ry(yaw) * Rz(-pitch) * Rx(roll)
    double r00 = cy * cp;
    double r01 = cy * sp * cr + sy * sr;
    double r02 = -cy * sp * sr + sy * cr;

    double r10 = -sp;
    double r11 = cp * cr;
    double r12 = -cp * sr;

    double r20 = -sy * cp;
    double r21 = -sy * sp * cr + cy * sr;
    double r22 = sy * sp * sr + cy * cr;

    // Camera origin in world coordinates (mount offset: 0.18m forward, 0.04m up):
    double p_cam_tx = r00 * 0.18 + r01 * 0.04;
    double p_cam_ty = r10 * 0.18 + r11 * 0.04;
    double p_cam_tz = r20 * 0.18 + r21 * 0.04;

    double cam_x = true_p.x + p_cam_tx;
    double cam_y = true_p.y - p_cam_tz;
    double cam_z = true_p.z + p_cam_ty;

    // Camera basis vectors in world coordinates:
    // forward = +Z_cam, right = +X_cam, down = +Y_cam
    double fwd_x = r00,   fwd_y = -r20,  fwd_z = r10;
    double right_x = r02, right_y = -r22, right_z = r12;
    double down_x = -r01, down_y = r21,  down_z = -r11;

    // Effective focal length for 320x240 frame matching GateDetector
    const double effective_focal = 240.0 / (2.0 * std::tan(35.0 * M_PI / 180.0));
    const double cx_img = 160.0;
    const double cy_img = 120.0;

    // Depth sort goals: furthest goals rendered first, nearest goals rendered on top
    struct GoalRenderEntry {
        size_t index;
        double dist;
        double fwd;
        double lat;
        double vert_down;
    };
    std::vector<GoalRenderEntry> entries;
    entries.reserve(goals.size());

    for (size_t i = 0; i < goals.size(); ++i) {
        const auto& goal = goals[i];
        double dx = goal.x - cam_x;
        double dy = goal.y - cam_y;
        double dz = goal.z - cam_z;

        double fwd = dx * fwd_x + dy * fwd_y + dz * fwd_z;
        double lat = dx * right_x + dy * right_y + dz * right_z;
        double vert_down = dx * down_x + dy * down_y + dz * down_z;
        double dist = std::sqrt(dx * dx + dy * dy + dz * dz);

        if (config_.fov_filtering_enabled) {
            if (fwd <= config_.min_range_m || dist > config_.max_range_m) {
                continue;
            }
            double h_angle = std::atan2(std::abs(lat), fwd);
            double v_angle = std::atan2(std::abs(vert_down), fwd);
            if (h_angle > (config_.h_fov_deg * 0.5 * M_PI / 180.0) ||
                v_angle > (config_.v_fov_deg * 0.5 * M_PI / 180.0)) {
                continue;
            }
            if (config_.proximity_dropout_enabled && dist < config_.proximity_dropout_dist) {
                continue;
            }
            if (config_.occlusion_check_enabled) {
                bool occluded = false;
                double ray_x = dx / dist, ray_y = dy / dist, ray_z = dz / dist;
                for (size_t j = 0; j < goals.size(); ++j) {
                    if (j == i) continue;
                    double ox = goals[j].x - cam_x;
                    double oy = goals[j].y - cam_y;
                    double oz = goals[j].z - cam_z;
                    double o_dist = std::sqrt(ox * ox + oy * oy + oz * oz);
                    if (o_dist < dist - 0.25) {
                        double proj = ox * ray_x + oy * ray_y + oz * ray_z;
                        if (proj > 0.0 && proj < dist) {
                            double perp_sq = (ox - proj * ray_x) * (ox - proj * ray_x) +
                                             (oy - proj * ray_y) * (oy - proj * ray_y) +
                                             (oz - proj * ray_z) * (oz - proj * ray_z);
                            if (perp_sq < (0.28 * 0.28)) {
                                occluded = true;
                                break;
                            }
                        }
                    }
                }
                if (occluded) continue;
            }
        }

        if (fwd > 0.05) {
            entries.push_back({i, dist, fwd, lat, vert_down});
        }
    }

    std::sort(entries.begin(), entries.end(), [](const GoalRenderEntry& a, const GoalRenderEntry& b) {
        return a.dist > b.dist; // Furthest first
    });

    for (const auto& entry : entries) {
        const auto& goal = goals[entry.index];
        double yaw_g = goal.yaw;

        // Projection helper for gate local coordinates (u_loc: right, v_loc: up)
        auto projectLocalPoint = [&](double u_loc, double v_loc, cv::Point& pt_out) -> bool {
            double wx = goal.x - u_loc * std::sin(yaw_g);
            double wy = goal.y + u_loc * std::cos(yaw_g);
            double wz = goal.z + v_loc;

            double dx = wx - cam_x;
            double dy = wy - cam_y;
            double dz = wz - cam_z;

            double pfwd = dx * fwd_x + dy * fwd_y + dz * fwd_z;
            if (pfwd <= 0.05) return false;

            double plat = dx * right_x + dy * right_y + dz * right_z;
            double pdown = dx * down_x + dy * down_y + dz * down_z;

            pt_out.x = static_cast<int>(std::round(cx_img + effective_focal * (plat / pfwd)));
            pt_out.y = static_cast<int>(std::round(cy_img + effective_focal * (pdown / pfwd)));
            return true;
        };

        if (goal.shape_type == SHAPE_CIRCLE) {
            // Render Gate 1 (Circle): Pure Blue (B=255, G=0, R=0), outer diameter 0.500m (radius 0.250m)
            const int NUM_PTS = 32;
            std::vector<cv::Point> poly;
            poly.reserve(NUM_PTS);
            bool all_valid = true;
            for (int k = 0; k < NUM_PTS; ++k) {
                double theta = 2.0 * M_PI * k / NUM_PTS;
                double u_loc = 0.250 * std::cos(theta);
                double v_loc = 0.250 * std::sin(theta);
                cv::Point pt;
                if (!projectLocalPoint(u_loc, v_loc, pt)) {
                    all_valid = false;
                    break;
                }
                poly.push_back(pt);
            }
            if (all_valid && poly.size() >= 3) {
                cv::fillPoly(render_buffer_, std::vector<std::vector<cv::Point>>{poly}, cv::Scalar(255, 0, 0));
            }
        } else if (goal.shape_type == SHAPE_TRIANGLE) {
            // Render Gate 2 (Triangle): Pure Red (B=0, G=0, R=255), outer width 0.6662m, apex UP
            std::vector<cv::Point> poly(3);
            if (projectLocalPoint(0.0, 0.38, poly[0]) &&
                projectLocalPoint(0.3331, -0.205, poly[1]) &&
                projectLocalPoint(-0.3331, -0.205, poly[2])) {
                cv::fillPoly(render_buffer_, std::vector<std::vector<cv::Point>>{poly}, cv::Scalar(0, 0, 255));
            }
        } else if (goal.shape_type == SHAPE_SQUARE) {
            // Render Gate 3 (Square): Pure Green (B=0, G=255, R=0), outer half-width = 0.20678m (matching Three.js & CollisionDetector)
            const double hw = 0.20678;
            std::vector<cv::Point> poly(4);
            if (projectLocalPoint(-hw, hw, poly[0]) &&
                projectLocalPoint(hw, hw, poly[1]) &&
                projectLocalPoint(hw, -hw, poly[2]) &&
                projectLocalPoint(-hw, -hw, poly[3])) {
                cv::fillPoly(render_buffer_, std::vector<std::vector<cv::Point>>{poly}, cv::Scalar(0, 255, 0));
            }
        }
    }

    return render_buffer_;
}
#endif

VisionPacket SimVisionHarness::generateVisionPacket(
    const Pose& true_p,
    const std::vector<SimulatedGoal>& goals,
    const std::vector<SimulatedTag>& tags,
    uint32_t sequence_id,
    double current_time)
{
    VisionPacket v_pkt{};
    v_pkt.msg_type = MSG_TYPE_VISION;
    v_pkt.sequence_id = sequence_id;
    v_pkt.timestamp_ns = static_cast<uint64_t>(current_time * 1e9);

    if (goals.empty() && tags.empty()) {
        return v_pkt;
    }

#ifdef SIM_HAS_OPENCV
    // 1. Render camera view frame from drone perspective
    cv::Mat frame = renderFrame(true_p, goals);

    // 2. Hook into running simulation daemon over UDP (if active)
    sendFrameToRunningSimulation(frame);

    VisionPacket sim_daemon_pkt{};
    bool received_from_sim = tryReceiveRunningSimulationPacket(sim_daemon_pkt);

    // 3. Process frame via actual OpenCV vision pipeline (findContours, moments, solvePnP)
    std::vector<vision::GateDetectionResult> results;
    if (received_from_sim && sim_daemon_pkt.num_goals_reported > 0) {
        // If external vision_node daemon provided detections, adopt them
        v_pkt.num_goals_reported = sim_daemon_pkt.num_goals_reported;
        for (uint8_t i = 0; i < v_pkt.num_goals_reported; ++i) {
            v_pkt.goals[i] = sim_daemon_pkt.goals[i];
        }
    } else {
        // Run in-process vision detector
        results = detector_.detectAllGates(frame, false);

        v_pkt.num_goals_reported = static_cast<uint8_t>(std::min(goals.size(), static_cast<size_t>(3)));
        for (size_t i = 0; i < v_pkt.num_goals_reported; ++i) {
            const auto& goal = goals[i];
            GoalMeasurement& gm = v_pkt.goals[i];
            gm.shape_type = goal.shape_type;

            std::string expected_shape = "Square";
            if (goal.shape_type == SHAPE_CIRCLE) expected_shape = "Circle";
            else if (goal.shape_type == SHAPE_TRIANGLE) expected_shape = "Triangle";

            // Find matching detection result from vision processing
            const vision::GateDetectionResult* matched = nullptr;
            for (const auto& res : results) {
                if (res.detected && res.shape == expected_shape) {
                    matched = &res;
                    break;
                }
            }

            if (matched) {
                gm.is_detected = 1;
                gm.centroid.u = static_cast<float>(matched->center.x);
                gm.centroid.v = static_cast<float>(matched->center.y);
                gm.contour_area = static_cast<float>(matched->bbox_size);
                gm.num_hypotheses = 1;
                gm.hypotheses[0].position_cam = {
                    static_cast<float>(matched->posX_m),
                    static_cast<float>(matched->posY_m),
                    static_cast<float>(matched->posZ_m)
                };
                if (matched->is_3d) {
                    gm.hypotheses[0].orientation_cam = vision::eulerToQuaternion(
                        matched->roll_rad, matched->pitch_rad, matched->yaw_rad);
                } else {
                    double rel_yaw = goal.yaw - true_p.yaw;
                    while (rel_yaw > M_PI) rel_yaw -= 2.0 * M_PI;
                    while (rel_yaw < -M_PI) rel_yaw += 2.0 * M_PI;
                    gm.hypotheses[0].orientation_cam = {
                        static_cast<float>(std::cos(rel_yaw * 0.5)),
                        0.0f,
                        0.0f,
                        static_cast<float>(std::sin(rel_yaw * 0.5))
                    };
                }
                gm.hypotheses[0].confidence = static_cast<float>(matched->confidence);
            } else {
                gm.is_detected = 0;
                gm.num_hypotheses = 0;
            }
        }

        if (goals.empty() && !results.empty()) {
            VisionPacket auto_pkt = vision::GateDetector::toVisionPacket(results, sequence_id, v_pkt.timestamp_ns);
            v_pkt.num_goals_reported = auto_pkt.num_goals_reported;
            for (uint8_t i = 0; i < v_pkt.num_goals_reported; ++i) {
                v_pkt.goals[i] = auto_pkt.goals[i];
            }
        }
    }
#else
    // When compiling without OpenCV (e.g. cross-compilation target sysroots without libopencv-dev),
    // compute ground-truth 3D camera frame projection geometrically.
    VisionPacket sim_daemon_pkt{};
    if (tryReceiveRunningSimulationPacket(sim_daemon_pkt) && sim_daemon_pkt.num_goals_reported > 0) {
        v_pkt.num_goals_reported = sim_daemon_pkt.num_goals_reported;
        for (uint8_t i = 0; i < v_pkt.num_goals_reported; ++i) {
            v_pkt.goals[i] = sim_daemon_pkt.goals[i];
        }
    } else {
        double cy = std::cos(true_p.yaw), sy = std::sin(true_p.yaw);
        double cp = std::cos(true_p.pitch), sp = std::sin(true_p.pitch);
        double cr = std::cos(true_p.roll), sr = std::sin(true_p.roll);

        double r00 = cy * cp;
        double r01 = cy * sp * cr + sy * sr;
        double r02 = -cy * sp * sr + sy * cr;
        double r10 = -sp;
        double r11 = cp * cr;
        double r12 = -cp * sr;
        double r20 = -sy * cp;
        double r21 = -sy * sp * cr + cy * sr;
        double r22 = sy * sp * sr + cy * cr;

        double p_cam_tx = r00 * 0.18 + r01 * 0.04;
        double p_cam_ty = r10 * 0.18 + r11 * 0.04;
        double p_cam_tz = r20 * 0.18 + r21 * 0.04;

        double cam_x = true_p.x + p_cam_tx;
        double cam_y = true_p.y - p_cam_tz;
        double cam_z = true_p.z + p_cam_ty;

        double fwd_x = r00,   fwd_y = -r20,  fwd_z = r10;
        double right_x = r02, right_y = -r22, right_z = r12;
        double down_x = -r01, down_y = r21,  down_z = -r11;

        v_pkt.num_goals_reported = static_cast<uint8_t>(std::min(goals.size(), static_cast<size_t>(3)));
        for (size_t i = 0; i < v_pkt.num_goals_reported; ++i) {
            const auto& goal = goals[i];
            GoalMeasurement& gm = v_pkt.goals[i];
            gm.shape_type = goal.shape_type;

            double dx = goal.x - cam_x;
            double dy = goal.y - cam_y;
            double dz = goal.z - cam_z;

            double fwd = dx * fwd_x + dy * fwd_y + dz * fwd_z;
            double lat = dx * right_x + dy * right_y + dz * right_z;
            double vert_down = dx * down_x + dy * down_y + dz * down_z;
            double dist = std::sqrt(dx * dx + dy * dy + dz * dz);

            if (config_.fov_filtering_enabled) {
                if (fwd <= config_.min_range_m || dist > config_.max_range_m) {
                    gm.is_detected = 0;
                    gm.num_hypotheses = 0;
                    continue;
                }
                double h_angle = std::atan2(std::abs(lat), fwd);
                double v_angle = std::atan2(std::abs(vert_down), fwd);
                if (h_angle > (config_.h_fov_deg * 0.5 * M_PI / 180.0) ||
                    v_angle > (config_.v_fov_deg * 0.5 * M_PI / 180.0)) {
                    gm.is_detected = 0;
                    gm.num_hypotheses = 0;
                    continue;
                }
                if (config_.proximity_dropout_enabled && dist < config_.proximity_dropout_dist) {
                    gm.is_detected = 0;
                    gm.num_hypotheses = 0;
                    continue;
                }
                if (config_.occlusion_check_enabled) {
                    bool occluded = false;
                    double ray_x = dx / dist, ray_y = dy / dist, ray_z = dz / dist;
                    for (size_t j = 0; j < goals.size(); ++j) {
                        if (j == i) continue;
                        double ox = goals[j].x - cam_x;
                        double oy = goals[j].y - cam_y;
                        double oz = goals[j].z - cam_z;
                        double o_dist = std::sqrt(ox * ox + oy * oy + oz * oz);
                        if (o_dist < dist - 0.25) {
                            double proj = ox * ray_x + oy * ray_y + oz * ray_z;
                            if (proj > 0.0 && proj < dist) {
                                double perp_sq = (ox - proj * ray_x) * (ox - proj * ray_x) +
                                                 (oy - proj * ray_y) * (oy - proj * ray_y) +
                                                 (oz - proj * ray_z) * (oz - proj * ray_z);
                                if (perp_sq < (0.28 * 0.28)) {
                                    occluded = true;
                                    break;
                                }
                            }
                        }
                    }
                    if (occluded) {
                        gm.is_detected = 0;
                        gm.num_hypotheses = 0;
                        continue;
                    }
                }
            }

            gm.is_detected = 1;
            gm.num_hypotheses = 1;
            gm.hypotheses[0].position_cam = {
                static_cast<float>(lat),
                static_cast<float>(vert_down),
                static_cast<float>(fwd)
            };
            double rel_yaw = goal.yaw - true_p.yaw;
            while (rel_yaw > M_PI) rel_yaw -= 2.0 * M_PI;
            while (rel_yaw < -M_PI) rel_yaw += 2.0 * M_PI;
            gm.hypotheses[0].orientation_cam = {
                static_cast<float>(std::cos(rel_yaw * 0.5)),
                0.0f,
                0.0f,
                static_cast<float>(std::sin(rel_yaw * 0.5))
            };
            gm.hypotheses[0].confidence = 1.0f;
        }
    }
#endif

    // 4. Populate surveyed fiducial tags
    v_pkt.num_tags_detected = 0;
    for (const auto& tag : tags) {
        if (v_pkt.num_tags_detected >= 4) break;
        double dx = true_p.x - tag.x;
        double dy = true_p.y - tag.y;
        double dz = true_p.z - tag.z;
        double dist = std::sqrt(dx * dx + dy * dy + dz * dz);

        // Visible when within 3.5m radius of the surveyed tag
        if (dist <= 3.5) {
            TagMeasurement& tm = v_pkt.tags[v_pkt.num_tags_detected++];
            tm.tag_id = tag.tag_id;
            tm.is_valid = 1;
            tm.camera_pos_world = {
                static_cast<float>(true_p.x),
                static_cast<float>(true_p.y),
                static_cast<float>(true_p.z)
            };

            double cy_tag = std::cos(true_p.yaw * 0.5);
            double sy_tag = std::sin(true_p.yaw * 0.5);
            double cp_tag = std::cos(true_p.pitch * 0.5);
            double sp_tag = std::sin(true_p.pitch * 0.5);
            double cr_tag = std::cos(true_p.roll * 0.5);
            double sr_tag = std::sin(true_p.roll * 0.5);
            tm.camera_att_world.w = static_cast<float>(cr_tag * cp_tag * cy_tag + sr_tag * cp_tag * sy_tag);
            tm.camera_att_world.x = static_cast<float>(sr_tag * cp_tag * cy_tag - cr_tag * sp_tag * sy_tag);
            tm.camera_att_world.y = static_cast<float>(cr_tag * sp_tag * cy_tag + sr_tag * cp_tag * sy_tag);
            tm.camera_att_world.z = static_cast<float>(cr_tag * cp_tag * sy_tag - sr_tag * sp_tag * cy_tag);
            tm.confidence = tag.confidence;
        }
    }

    return v_pkt;
}

} // namespace sim
