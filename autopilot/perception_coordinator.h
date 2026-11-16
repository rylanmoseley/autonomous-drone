#pragma once
#include <map>
#include <mutex>
#include <cstdint>
#include "vision_packet.h"
#include "flight_control_interface.h"
#include "mavlink_daemon.h"

struct SurveyedTagPose {
    uint16_t tag_id{0};
    Vec3f world_pos{0.0f, 0.0f, 0.0f};  // World NED coordinates (meters)
    Quatf world_att{1.0f, 0.0f, 0.0f, 0.0f}; // World NED attitude
    float size_m{0.16f};                 // Physical side length in meters
    bool is_known{false};
};

class PerceptionCoordinator {
public:
    PerceptionCoordinator();

    void registerSurveyedTag(uint16_t tag_id, const Vec3f& world_pos, const Quatf& world_att, float size_m = 0.16f);
    bool getSurveyedTag(uint16_t tag_id, SurveyedTagPose& out_tag) const;
    bool hasSurveyedTag(uint16_t tag_id) const;
    void clearSurveyedTags();

    // Performs full 3D attitude-compensated camera-to-world projection and cage bounds check
    static VisionPacket transformVisionGoals(const VisionPacket& pkt, const Pose& drone_pose);

    // Processes tag measurements: verifies against surveyed map and forwards to FCU via MAVLink
    void processTagForwarding(const VisionPacket& pkt, double current_time, MavlinkDaemon& fcu);

    uint32_t getTagsReceivedCount() const { return tags_received_count; }
    uint32_t getTagsForwardedCount() const { return tags_forwarded_count; }
    uint32_t getSurveyedTagsVerifiedCount() const { return surveyed_tags_verified_count_; }
    TagMeasurement getLastTagMeasurement() const { return last_tag_measurement; }

    void resetCounts();

    uint32_t tags_received_count{0};
    uint32_t tags_forwarded_count{0};
    uint32_t surveyed_tags_verified_count_{0};
    TagMeasurement last_tag_measurement{};

private:
    std::map<uint16_t, SurveyedTagPose> surveyed_tags_;
    mutable std::mutex tag_map_mutex_;
};
