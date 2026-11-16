#pragma once
#include <cstdint>
#include <vector>
#include <cstring>
#include "flight_control_interface.h"
#include "ui_protocol.h"
#include "path_planner.h"

class MissionSequencer {
public:
    MissionSequencer();

    void reset();
    void startMission(const UICommandPacket& pkt, const Pose& current_pose);
    void updateSequence(const UICommandPacket& pkt, const Pose& current_pose);
    void setConstraints(uint8_t use_tube, float rail_y, float rail_z);

    bool isSequenceComplete() const { return current_seq_index >= active_sequence_len; }
    uint8_t getCurrentTargetShape() const;
    void advanceSequence(const Pose& completed_goal_pose);

    // Sequence properties
    uint8_t active_sequence_len{0};
    uint8_t goal_sequence[MAX_GOAL_SEQUENCE_LEN]{};
    uint8_t current_seq_index{0};
    uint8_t configured_end_action{MISSION_END_HOVER};
    float hover_timeout_s{0.0f};
    uint8_t use_tube_constraint{0};
    float tube_rail_y{0.0f};
    float tube_rail_z{0.0f};

    // Tracking progress
    bool new_goal{false};
    int goal_phase{0}; // 0 = standoff, 1 = align, 2 = traverse
    int align_ticks{0};
    int standoff_ticks{0};
    int seek_ticks{0};
    int traverse_ticks{0};

    double approach_dx{0.0};
    double approach_dy{0.0};
    double approach_dz{0.0};
    double approach_sign_{1.0};
    double last_approach_dx_{1.0};
    double last_approach_dy_{0.0};

    Pose active_target_pose{};
    bool has_active_target{false};
    PathPlanner::OrthogonalPath active_orthogonal_path_{};
    bool has_orthogonal_path_{false};

    Pose last_completed_goal_pose_{0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0};
    Pose hover_pose_{0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0};
    bool has_hover_pose_{false};
};
