#include "mission_sequencer.h"
#include <algorithm>

MissionSequencer::MissionSequencer() {
    reset();
}

void MissionSequencer::reset() {
    active_sequence_len = 0;
    std::memset(goal_sequence, 0, sizeof(goal_sequence));
    current_seq_index = 0;
    configured_end_action = MISSION_END_HOVER;
    hover_timeout_s = 0.0f;
    use_tube_constraint = 0;
    tube_rail_y = 0.0f;
    tube_rail_z = 0.0f;

    new_goal = false;
    goal_phase = 0;
    align_ticks = 0;
    standoff_ticks = 0;
    seek_ticks = 0;
    traverse_ticks = 0;

    approach_dx = 0.0;
    approach_dy = 0.0;
    approach_dz = 0.0;
    approach_sign_ = 1.0;
    last_approach_dx_ = 1.0;
    last_approach_dy_ = 0.0;

    active_target_pose = {};
    has_active_target = false;
    active_orthogonal_path_ = {};
    has_orthogonal_path_ = false;

    has_hover_pose_ = false;
}

void MissionSequencer::startMission(const UICommandPacket& pkt, const Pose& current_pose) {
    current_seq_index = 0;
    new_goal = true;
    last_approach_dx_ = 1.0;
    last_approach_dy_ = 0.0;
    approach_sign_ = 1.0;
    has_active_target = false;
    goal_phase = 0;
    align_ticks = 0;
    standoff_ticks = 0;
    seek_ticks = 0;
    traverse_ticks = 0;
    active_target_pose = {};
    has_hover_pose_ = false;
    has_orthogonal_path_ = false;
    active_orthogonal_path_ = {};
    last_completed_goal_pose_ = current_pose;

    active_sequence_len = pkt.sequence_length;
    std::memcpy(goal_sequence, pkt.goal_sequence, sizeof(goal_sequence));
    configured_end_action = pkt.end_action;
    hover_timeout_s = pkt.hover_timeout_s;
    use_tube_constraint = pkt.use_tube_constraint;
    tube_rail_y = pkt.tube_rail_y;
    tube_rail_z = pkt.tube_rail_z;
}

void MissionSequencer::updateSequence(const UICommandPacket& pkt, const Pose& current_pose) {
    active_sequence_len = pkt.sequence_length;
    std::memcpy(goal_sequence, pkt.goal_sequence, sizeof(goal_sequence));
    configured_end_action = pkt.end_action;
    hover_timeout_s = pkt.hover_timeout_s;
    current_seq_index = 0;

    new_goal = true;
    last_approach_dx_ = 1.0;
    last_approach_dy_ = 0.0;
    approach_sign_ = 1.0;
    has_active_target = false;
    goal_phase = 0;
    align_ticks = 0;
    standoff_ticks = 0;
    seek_ticks = 0;
    traverse_ticks = 0;
    active_target_pose = {};
    has_hover_pose_ = false;
    has_orthogonal_path_ = false;
    active_orthogonal_path_ = {};
    last_completed_goal_pose_ = current_pose;
}

void MissionSequencer::setConstraints(uint8_t use_tube, float rail_y, float rail_z) {
    use_tube_constraint = use_tube;
    tube_rail_y = rail_y;
    tube_rail_z = rail_z;
}

uint8_t MissionSequencer::getCurrentTargetShape() const {
    if (current_seq_index < active_sequence_len) {
        return goal_sequence[current_seq_index];
    }
    return static_cast<uint8_t>(SHAPE_NONE);
}

void MissionSequencer::advanceSequence(const Pose& completed_goal_pose) {
    last_completed_goal_pose_ = completed_goal_pose;
    last_approach_dx_ = approach_dx;
    last_approach_dy_ = approach_dy;
    current_seq_index++;
    new_goal = true;
    has_active_target = false;
    has_orthogonal_path_ = false;
    goal_phase = 0;
    align_ticks = 0;
    standoff_ticks = 0;
    traverse_ticks = 0;
}
