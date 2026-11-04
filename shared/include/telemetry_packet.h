#pragma once
#include "common_types.h"

#pragma pack(push, 1)

// Goal tracking state summary for display
typedef struct {
    uint8_t  shape_type;         // ShapeType
    uint8_t  state;              // TrackState (SEARCHING, CONVERGING, LOCKED, OCCLUDED)
    Vec3f    world_position;     // Filtered 3D position in World NED
    float    spatial_variance;   // Running variance (meters^2)
} GoalTrackTelemetry;

// High-rate state table for 2D/3D arena dashboard
typedef struct {
    uint8_t  msg_type;           // Always MSG_TYPE_TELEMETRY (4)
    uint32_t sequence_id;
    uint64_t timestamp_ns;

    // Fused Vehicle Odometry (from ArduPilot EKF3)
    Vec3f    drone_pos_ned;      // Meters
    Vec3f    drone_vel_ned;      // m/s
    Quatf    drone_att_ned;      // Vehicle attitude quaternion

    // Active Path Planner Setpoint
    Vec3f    target_pos_ned;     // Current waypoint position
    Vec3f    target_vel_ned;     // Current velocity setpoint

    // Goal Estimates
    GoalTrackTelemetry goals[3]; // Circle, Triangle, Square tracks

    // Fiducial localization telemetry
    uint8_t  num_tags_in_view;
    uint16_t tag_ids_in_view[4];

    // Raw sensor diagnostics
    float    optical_flow_quality; // 0 to 255 (from ArduPilot)
    float    ekf_pos_variance;     // EKF3 innovation test metric
    float    cpu_temperature_c;    // SBC temperature monitoring
} TelemetryPacket;

#pragma pack(pop)
