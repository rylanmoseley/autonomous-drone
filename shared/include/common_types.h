#pragma once
#include <stdint.h>

#pragma pack(push, 1)

// Global Message Type Registry (Always Byte Offset 0)
typedef enum {
    MSG_TYPE_NONE          = 0,
    MSG_TYPE_VISION        = 1,
    MSG_TYPE_UI_CMD        = 2,
    MSG_TYPE_UI_STATUS     = 3,
    MSG_TYPE_TELEMETRY     = 4,
    MSG_TYPE_LOGGER_STATUS = 5
} MsgType;

// Geometric shape classifications
typedef enum {
    SHAPE_NONE     = 0,
    SHAPE_CIRCLE   = 1,
    SHAPE_TRIANGLE = 2,
    SHAPE_SQUARE   = 3
} ShapeType;

// Terminal behavior once all sequence goals are visited
typedef enum {
    MISSION_END_HOVER = 0, // Hold position/standoff at the final goal
    MISSION_END_LAND  = 1  // Execute designated landing sequence
} MissionEndAction;

// 3D Cartesian Vector (meters or m/s)
typedef struct {
    float x;
    float y;
    float z;
} Vec3f;

// Normalized Quaternion [w, x, y, z]
typedef struct {
    float w;
    float x;
    float y;
    float z;
} Quatf;

// 2D Image Coordinates (pixels or normalized [0.0 - 1.0])
typedef struct {
    float u;
    float v;
} Point2f;

// Geometric shape classifications
typedef enum {
    SHAPE_NONE     = 0,
    SHAPE_CIRCLE   = 1,
    SHAPE_TRIANGLE = 2,
    SHAPE_SQUARE   = 3
} ShapeType;

// Target tracking filter status in Autopilot
typedef enum {
    TRACK_STATE_SEARCHING   = 0, // Not yet in view
    TRACK_STATE_CONVERGING  = 1, // Observed, disambiguating/filtering
    TRACK_STATE_LOCKED      = 2, // Spatial variance below threshold
    TRACK_STATE_OCCLUDED    = 3  // Previously locked, currently lost
} TrackState;

// High-level flight mission states
typedef enum {
    FLIGHT_STATE_BOOT           = 0,
    FLIGHT_STATE_STANDBY_DISARM = 1,
    FLIGHT_STATE_ARMED          = 2,
    FLIGHT_STATE_TAKEOFF        = 3,
    FLIGHT_STATE_GOAL_SEEK      = 4,
    FLIGHT_STATE_PATH_FOLLOWING = 5,
    FLIGHT_STATE_HOVER_WAIT	= 6,
    FLIGHT_STATE_LANDING        = 7,
    FLIGHT_STATE_ROTOR_STOPPED  = 8,
    FLIGHT_STATE_ESTOP_TRIPPED  = 9
} FlightState;

#pragma pack(pop)
