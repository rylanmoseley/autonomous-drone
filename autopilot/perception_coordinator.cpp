#include "perception_coordinator.h"
#include <cmath>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

PerceptionCoordinator::PerceptionCoordinator() {}

void PerceptionCoordinator::registerSurveyedTag(uint16_t tag_id, const Vec3f& world_pos, const Quatf& world_att, float size_m) {
    std::lock_guard<std::mutex> lock(tag_map_mutex_);
    SurveyedTagPose tag{};
    tag.tag_id = tag_id;
    tag.world_pos = world_pos;
    tag.world_att = world_att;
    tag.size_m = size_m;
    tag.is_known = true;
    surveyed_tags_[tag_id] = tag;
}

bool PerceptionCoordinator::getSurveyedTag(uint16_t tag_id, SurveyedTagPose& out_tag) const {
    std::lock_guard<std::mutex> lock(tag_map_mutex_);
    auto it = surveyed_tags_.find(tag_id);
    if (it != surveyed_tags_.end()) {
        out_tag = it->second;
        return true;
    }
    return false;
}

bool PerceptionCoordinator::hasSurveyedTag(uint16_t tag_id) const {
    std::lock_guard<std::mutex> lock(tag_map_mutex_);
    return surveyed_tags_.find(tag_id) != surveyed_tags_.end();
}

void PerceptionCoordinator::clearSurveyedTags() {
    std::lock_guard<std::mutex> lock(tag_map_mutex_);
    surveyed_tags_.clear();
}

void PerceptionCoordinator::resetCounts() {
    tags_received_count = 0;
    tags_forwarded_count = 0;
    surveyed_tags_verified_count_ = 0;
    std::lock_guard<std::mutex> lock(last_tag_mutex_);
    last_tag_measurement = {};
}

VisionPacket PerceptionCoordinator::transformVisionGoals(const VisionPacket& pkt, const Pose& cur) {
    VisionPacket transformed_pkt = pkt;

    double cy = std::cos(cur.yaw), sy = std::sin(cur.yaw);
    double cp = std::cos(cur.pitch), sp = std::sin(cur.pitch);
    double cr = std::cos(cur.roll), sr = std::sin(cur.roll);

    // Three.js drone rotation matrix (order YZX): R = Ry(yaw) * Rz(-pitch) * Rx(roll)
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

    double cam_x = cur.x + p_cam_tx;
    double cam_y = cur.y - p_cam_tz;
    double cam_z = cur.z + p_cam_ty;

    // Camera basis vectors in world coordinates (forward = +Z_cam, right = +X_cam, down = +Y_cam):
    double fwd_x = r00,   fwd_y = -r20,  fwd_z = r10;
    double right_x = r02, right_y = -r22, right_z = r12;
    double down_x = -r01, down_y = r21,  down_z = -r11;

    for (uint8_t i = 0; i < transformed_pkt.num_goals_reported && i < 3; ++i) {
        GoalMeasurement& gm = transformed_pkt.goals[i];
        if (gm.is_detected && gm.num_hypotheses > 0) {
            PoseHypothesis& hyp = gm.hypotheses[0];
            double z_cam = hyp.position_cam.z;
            double x_cam = hyp.position_cam.x;
            double y_cam = hyp.position_cam.y;

            // Full 3D attitude-compensated projection:
            double world_x = cam_x + z_cam * fwd_x + x_cam * right_x + y_cam * down_x;
            double world_y = cam_y + z_cam * fwd_y + x_cam * right_y + y_cam * down_y;
            double world_z = cam_z + z_cam * fwd_z + x_cam * right_z + y_cam * down_z;

            // Plausibility check: gates in the arena safety cage are strictly bounded
            if (world_z > 2.30 || world_z < 0.20 || z_cam > 5.5 || z_cam < 0.20 || world_x > 8.0 || world_x < -2.0 || std::abs(world_y) > 4.0) {
                gm.is_detected = 0;
                continue;
            }

            hyp.position_cam.x = static_cast<float>(world_x);
            hyp.position_cam.y = static_cast<float>(world_y);
            hyp.position_cam.z = static_cast<float>(world_z);

            Quatf q_cam = hyp.orientation_cam;

            // Drone orientation quaternion:
            double cy_d = std::cos(cur.yaw * 0.5), sy_d = std::sin(cur.yaw * 0.5);
            double cp_d = std::cos(cur.pitch * 0.5), sp_d = std::sin(cur.pitch * 0.5);
            double cr_d = std::cos(cur.roll * 0.5), sr_d = std::sin(cur.roll * 0.5);

            Quatf q_drone;
            q_drone.w = static_cast<float>(cr_d * cp_d * cy_d + sr_d * sp_d * sy_d);
            q_drone.x = static_cast<float>(sr_d * cp_d * cy_d - cr_d * sp_d * sy_d);
            q_drone.y = static_cast<float>(cr_d * sp_d * cy_d + sr_d * cp_d * sy_d);
            q_drone.z = static_cast<float>(cr_d * cp_d * sy_d - sr_d * sp_d * cy_d);

            // Compound rotation: q_world = q_drone * q_cam
            Quatf q_world;
            q_world.w = q_drone.w * q_cam.w - q_drone.x * q_cam.x - q_drone.y * q_cam.y - q_drone.z * q_cam.z;
            q_world.x = q_drone.w * q_cam.x + q_drone.x * q_cam.w + q_drone.y * q_cam.z - q_drone.z * q_cam.y;
            q_world.y = q_drone.w * q_cam.y - q_drone.x * q_cam.z + q_drone.y * q_cam.w + q_drone.z * q_cam.x;
            q_world.z = q_drone.w * q_cam.z + q_drone.x * q_cam.y - q_drone.y * q_cam.x + q_drone.z * q_cam.w;

            float norm = std::sqrt(q_world.w * q_world.w + q_world.x * q_world.x +
                                   q_world.y * q_world.y + q_world.z * q_world.z);
            if (norm > 1e-6f) {
                q_world.w /= norm;
                q_world.x /= norm;
                q_world.y /= norm;
                q_world.z /= norm;
            }
            hyp.orientation_cam = q_world;
        }
    }

    return transformed_pkt;
}

void PerceptionCoordinator::processTagForwarding(const VisionPacket& pkt, double current_time, MavlinkDaemon& fcu) {
    for (uint8_t i = 0; i < pkt.num_tags_detected && i < 4; ++i) {
        const TagMeasurement& tag = pkt.tags[i];
        if (tag.is_valid) {
            tags_received_count++;
            if (hasSurveyedTag(tag.tag_id)) {
                surveyed_tags_verified_count_++;
                uint64_t usec = (pkt.timestamp_ns > 0)
                    ? (pkt.timestamp_ns / 1000)
                    : static_cast<uint64_t>(current_time * 1e6);
                fcu.sendVisionPositionEstimate(tag, usec);
                tags_forwarded_count++;
                std::lock_guard<std::mutex> lock(last_tag_mutex_);
                last_tag_measurement = tag;
            }
        }
    }
}
