#pragma once
#include "common_types.h"

#pragma pack(push, 1)

// Error bitflags for the vision pipeline
typedef enum {
    VISION_ERR_NONE             = 0,
    VISION_ERR_CAMERA_TIMEOUT   = (1 << 0),
    VISION_ERR_FRAME_DROPPED    = (1 << 1),
    VISION_ERR_UNDEREXPOSED     = (1 << 2),
    VISION_ERR_OVEREXPOSED      = (1 << 3),
    VISION_ERR_CALIBRATION_FAIL = (1 << 4)
} VisionErrorFlags;

// Tag-derived world-space camera position
typedef struct {
    uint16_t tag_id;
    uint8_t  is_valid;
    Vec3f    camera_pos_world;   // Camera position in World NED
    Quatf    camera_att_world;   // Camera attitude in World NED
    float    confidence;         // 0.0 to 1.0 (inverse reprojection error / tag margin)
} TagMeasurement;

// Single 3D pose hypothesis for an observed shape
typedef struct {
    Vec3f position_cam;          // 3D position relative to camera lens (meters)
    Quatf orientation_cam;       // Orientation relative to camera
    float confidence;            // Geometric fit confidence / inverse residual
} PoseHypothesis;

// Goal detection payload supporting both Case A (3D PnP) and Case B (Tube constraint)
typedef struct {
    uint8_t  shape_type;         // ShapeType (Circle, Triangle, Square)
    uint8_t  is_detected;        // 1 if observed in this frame, 0 if occluded
    Point2f  centroid;           // 2D image centroid (used for Case B ray-casting)
    float    contour_area;       // Pixel area (scale heuristic)

    // Case A: 2 for Circle/Square, up to 4 for Triangle (Set to 0 if Case B is active)
    uint8_t  num_hypotheses;
    PoseHypothesis hypotheses[4];
} GoalMeasurement;

// Top-level datagram emitted by the Vision System
typedef struct {
    uint8_t  msg_type;           // Always MSG_TYPE_VISION (1)
    uint32_t sequence_id;        // Monotonically increasing frame counter
    uint64_t timestamp_ns;       // Hardware mid-exposure timestamp (CLOCK_MONOTONIC)
    uint32_t error_state;        // Bitmask of VisionErrorFlags

    uint8_t  num_tags_detected;
    TagMeasurement tags[4];      // Up to 4 simultaneous tags

    uint8_t  num_goals_reported; // Typically 3 (Circle, Triangle, Square)
    GoalMeasurement goals[3];
} VisionPacket;

#pragma pack(pop)
