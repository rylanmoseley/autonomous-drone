#include "sim_camera_perception.h"
#include <cmath>
#include <algorithm>

namespace sim {

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

SimCameraPerception::SimCameraPerception(const SimPerceptionConfig& config)
    : config_(config) {}

VisionPacket SimCameraPerception::generateVisionPacket(
    const Pose& true_p,
    const std::vector<SimulatedGoal>& goals,
    const std::vector<SimulatedTag>& tags,
    uint32_t sequence_id,
    double current_time) const
{
    VisionPacket v_pkt{};
    v_pkt.msg_type = MSG_TYPE_VISION;
    v_pkt.sequence_id = sequence_id;
    v_pkt.timestamp_ns = static_cast<uint64_t>(current_time * 1e9);

    if (goals.empty() && tags.empty()) {
        return v_pkt;
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

    // Camera basis vectors in world coordinates (forward = +Z_cam, right = +X_cam, down = +Y_cam):
    double fwd_x = r00,   fwd_y = -r20,  fwd_z = r10;
    double right_x = r02, right_y = -r22, right_z = r12;
    double down_x = -r01, down_y = r21,  down_z = -r11;

    // 1. Populate goals
    v_pkt.num_goals_reported = static_cast<uint8_t>(std::min(goals.size(), static_cast<size_t>(3)));
    for (size_t i = 0; i < v_pkt.num_goals_reported; ++i) {
        const auto& goal = goals[i];
        GoalMeasurement& gm = v_pkt.goals[i];
        gm.shape_type = goal.shape_type;

        double dx = goal.x - cam_x;
        double dy = goal.y - cam_y;
        double dz = goal.z - cam_z;

        // Exact 3D camera projection accounting for drone yaw, pitch, and roll:
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

    // 2. Populate fiducial tags
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
