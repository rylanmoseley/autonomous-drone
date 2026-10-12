#pragma once
#include "common_types.h"

// Maximum number of goal waypoints in a single mission sequence
#define MAX_GOAL_SEQUENCE_LEN 16

#pragma pack(push, 1)

// Discrete command triggers from the operator
typedef enum {
    UI_CMD_NONE              = 0,
    UI_CMD_ARM               = 1,
    UI_CMD_DISARM            = 2,
    UI_CMD_START_MISSION     = 3,
    UI_CMD_PAUSE_MISSION     = 4,
    UI_CMD_RESUME_MISSION    = 5,
    UI_CMD_LAND_NOW          = 6, // Force immediate landing sequence
    UI_CMD_ESTOP             = 7, // Immediate motor cutoff (highest priority)
    UI_CMD_SET_SEQUENCE      = 8, // Upload goal sequence & end behavior
    UI_CMD_SET_CONSTRAINTS   = 9  // Update tube Y/Z rail bounds
} UICommandType;

// Command packet: Operator UI -> Autopilot (/tmp/ui_to_autopilot.sock)
typedef struct {
    uint8_t  msg_type;           // Always MSG_TYPE_UI_CMD (2)
    uint32_t command_id;         // Monotonic command counter for ACKs
    uint64_t timestamp_ns;       // Host dispatch timestamp
    uint8_t  command;            // UICommandType

    // -------------------------------------------------------------
    // Mission Sequence Configuration
    // -------------------------------------------------------------
    uint8_t  sequence_length;    // 0 to MAX_GOAL_SEQUENCE_LEN
    uint8_t  goal_sequence[MAX_GOAL_SEQUENCE_LEN]; // Ordered array of ShapeType

    // -------------------------------------------------------------
    // Terminal Mission Behavior
    // -------------------------------------------------------------
    uint8_t  end_action;         // MissionEndAction (0 = HOVER, 1 = LAND)
    float    hover_timeout_s;    // If HOVER: timeout before auto-landing (0.0 = infinite hover)

    // -------------------------------------------------------------
    // Arena Physical Constraints
    // -------------------------------------------------------------
    uint8_t  use_tube_constraint;// 1 = Case B (Tube), 0 = Case A (Unconstrained)
    float    tube_rail_y;        // Tube Y coordinate in World NED (meters)
    float    tube_rail_z;        // Tube Z coordinate in World NED (meters, negative altitude)
} UICommandPacket;

// Status packet: Autopilot -> Operator UI (/tmp/autopilot_to_ui.sock at 2-5 Hz)
typedef struct {
    uint8_t  msg_type;           // Always MSG_TYPE_UI_STATUS (3)
    uint32_t sequence_id;        // Monotonic heartbeat counter
    uint64_t timestamp_ns;       // SBC CLOCK_MONOTONIC time

    // Core System Status
    uint8_t  flight_state;       // FlightState enum (STANDBY, SEEK, PATH, HOVER_WAIT, etc.)
    uint8_t  is_armed;           // 1 if motors armed, 0 if safe
    uint8_t  estop_active;       // 1 if emergency stop was triggered

    // Sequence Execution Telemetry
    uint8_t  active_sequence_len;// Total goals in the active mission sequence
    uint8_t  current_seq_index;  // Current step (0-indexed progress through sequence)
    uint8_t  current_target_shape;// ShapeType currently being targeted
    uint8_t  configured_end_action;// MissionEndAction currently active (HOVER vs LAND)

    // Battery & Hardware Health
    float    battery_voltage;    // From FCU SYS_STATUS (volts)
    float    battery_remaining;  // 0.0 to 100.0%
    uint32_t active_error_flags; // Aggregated system error bitmask

    // Command Handshake / Acknowledgment
    uint32_t last_acked_cmd_id;  // Echo of last command_id processed
    uint8_t  cmd_ack_result;     // 0 = Success, 1 = Rejected / Invalid State
} UIStatusPacket;

#pragma pack(pop)
