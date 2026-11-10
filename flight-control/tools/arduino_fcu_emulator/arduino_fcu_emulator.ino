/*
 * ==============================================================================
 *  HEXAGON Flight Drone - Arduino Uno R3 MAVLink FCU Emulator
 * ==============================================================================
 *  Description:
 *    Emulates an ArduPilot (ArduCopter) Flight Controller over MAVLink v2
 *    on an Arduino Uno R3 (ATmega328P). Allows end-to-end testing of:
 *      1. Companion computer serial bridge (mavlink_proxy.py / mavlink-routerd)
 *      2. Wi-Fi AP UDP telemetry broadcast (192.168.4.255:14550)
 *      3. Mission Planner / QGroundControl connectivity, parameters, and HUD
 *      4. Autopilot daemon telemetry ingestion (LOCAL_POSITION_NED, ATTITUDE)
 *
 *  Hardware Connections:
 *    Arduino Uno R3                 Raspberry Pi (40-Pin Header)
 *    ----------------             -----------------------------
 *    GND          --------------> Pin 6  (GND)
 *    Pin 0 (RX)   <-------------- Pin 8  (GPIO 14 - Pi TX, 3.3V safe)
 *    Pin 1 (TX)   ---[Divider]--> Pin 10 (GPIO 15 - Pi RX, 3.3V input!)
 *
 *  CRITICAL VOLTAGE LEVEL WARNING:
 *    The Arduino Uno R3 operates at 5.0V logic. The Raspberry Pi GPIO is 3.3V
 *    and NOT 5V tolerant!
 *    - Pi TX (3.3V) -> Uno RX: Safe directly (Uno reads 3.3V as HIGH).
 *    - Uno TX (5.0V) -> Pi RX: MUST use a logic level shifter or a simple
 *      two-resistor voltage divider (e.g. 1k ohm in series, 2k ohm to GND)
 *      to step 5V down to ~3.3V before connecting to Pi Pin 10!
 *
 *  Baud Rate Note:
 *    The default baud rate is set to 115200. On a 16MHz ATmega328P, 921600 baud
 *    has an 8.5% clock divisor error which causes framing corruptions. 115200
 *    baud has only 2.1% error and runs with zero packet drops.
 *    Start the companion proxy with:
 *      python3 mavlink_proxy.py --serial /dev/ttyAMA0 --baud 115200
 * ==============================================================================
 */

#include <Arduino.h>
#include <avr/pgmspace.h>

// ------------------------------------------------------------------------------
// Configuration
// ------------------------------------------------------------------------------
#define MAVLINK_BAUD          115200
#define SYSTEM_ID             1       // MAVLink System ID (1 = Primary Vehicle)
#define COMPONENT_ID          1       // MAVLink Component ID (1 = Autopilot)

#define MAVLINK_STX_V2        0xFD
#define MAVLINK_STX_V1        0xFE

// MAVLink Message IDs
#define MSG_ID_HEARTBEAT            0
#define MSG_ID_SYS_STATUS           1
#define MSG_ID_PARAM_REQUEST_READ   20
#define MSG_ID_PARAM_REQUEST_LIST   21
#define MSG_ID_PARAM_VALUE          22
#define MSG_ID_ATTITUDE             30
#define MSG_ID_LOCAL_POSITION_NED   32
#define MSG_ID_GLOBAL_POSITION_INT  33
#define MSG_ID_MISSION_REQUEST_LIST 43
#define MSG_ID_MISSION_COUNT        44
#define MSG_ID_VFR_HUD              74
#define MSG_ID_COMMAND_LONG         76
#define MSG_ID_COMMAND_ACK          77
#define MSG_ID_STATUSTEXT           253

// CRC Extra seed bytes
#define CRC_EXTRA_HEARTBEAT           50
#define CRC_EXTRA_SYS_STATUS          124
#define CRC_EXTRA_PARAM_VALUE         220
#define CRC_EXTRA_ATTITUDE            39
#define CRC_EXTRA_LOCAL_POSITION_NED  185
#define CRC_EXTRA_GLOBAL_POSITION_INT 104
#define CRC_EXTRA_MISSION_COUNT       221
#define CRC_EXTRA_VFR_HUD             20
#define CRC_EXTRA_COMMAND_ACK         143
#define CRC_EXTRA_STATUSTEXT          83

// ArduPilot specific modes & flags
#define MAV_TYPE_QUADROTOR            2
#define MAV_AUTOPILOT_ARDUPILOTMEGA   3
#define MAV_MODE_FLAG_CUSTOM_MODE     1
#define MAV_MODE_FLAG_SAFETY_ARMED    128
#define MAV_STATE_ACTIVE              4
#define MAV_STATE_STANDBY             3
#define COPTER_MODE_STABILIZE         0
#define COPTER_MODE_GUIDED            4
#define MAV_CMD_COMPONENT_ARM_DISARM  400
#define MAV_RESULT_ACCEPTED           0

// ------------------------------------------------------------------------------
// Packed MAVLink Message Payloads
// ------------------------------------------------------------------------------
#pragma pack(push, 1)

struct MsgHeartbeat {
    uint32_t custom_mode;
    uint8_t  type;
    uint8_t  autopilot;
    uint8_t  base_mode;
    uint8_t  system_status;
    uint8_t  mavlink_version;
};

struct MsgSysStatus {
    uint32_t onboard_control_sensors_present;
    uint32_t onboard_control_sensors_enabled;
    uint32_t onboard_control_sensors_health;
    uint16_t load;
    uint16_t voltage_battery; // [mV]
    int16_t  current_battery; // [10*mA = cA]
    uint16_t drop_rate_comm;
    uint16_t errors_comm;
    uint16_t errors_count1;
    uint16_t errors_count2;
    uint16_t errors_count3;
    uint16_t errors_count4;
    int8_t   battery_remaining; // [%]
};

struct MsgAttitude {
    uint32_t time_boot_ms;
    float    roll;       // [rad]
    float    pitch;      // [rad]
    float    yaw;        // [rad]
    float    rollspeed;  // [rad/s]
    float    pitchspeed; // [rad/s]
    float    yawspeed;   // [rad/s]
};

struct MsgLocalPositionNed {
    uint32_t time_boot_ms;
    float    x;  // [m] North
    float    y;  // [m] East
    float    z;  // [m] Down (negative is up)
    float    vx; // [m/s]
    float    vy; // [m/s]
    float    vz; // [m/s]
};

struct MsgGlobalPositionInt {
    uint32_t time_boot_ms;
    int32_t  lat;          // [degE7]
    int32_t  lon;          // [degE7]
    int32_t  alt;          // [mm MSL]
    int32_t  relative_alt; // [mm AGL]
    int16_t  vx;           // [cm/s]
    int16_t  vy;           // [cm/s]
    int16_t  vz;           // [cm/s]
    uint16_t hdg;          // [cdeg 0-35999]
};

struct MsgVfrHud {
    float    airspeed;    // [m/s]
    float    groundspeed; // [m/s]
    float    alt;         // [m]
    float    climb;       // [m/s]
    int16_t  heading;     // [deg]
    uint16_t throttle;    // [% 0-100]
};

struct MsgParamValue {
    float    param_value;
    uint16_t param_count;
    uint16_t param_index;
    char     param_id[16];
    uint8_t  param_type; // 9 = MAV_PARAM_TYPE_REAL32
};

struct MsgCommandAck {
    uint16_t command;
    uint8_t  result;
};

struct MsgMissionCount {
    uint16_t count;
    uint8_t  target_system;
    uint8_t  target_component;
};

struct MsgStatusText {
    uint8_t severity; // 6 = MAV_SEVERITY_INFO
    char    text[50];
};

#pragma pack(pop)

// ------------------------------------------------------------------------------
// Stored Parameters Table (Stored in PROGMEM Flash to save 2KB Uno SRAM)
// ------------------------------------------------------------------------------
struct StoredParam {
    char  id[16];
    float value;
};

const StoredParam PROGMEM kParameters[] = {
    {"SYSID_THISMAV",   1.0f},
    {"PILOT_SPEED_UP",  250.0f},
    {"PILOT_SPEED_DN",  150.0f},
    {"WP_RADIUS",       50.0f},
    {"BATT_MONITOR",    4.0f},
    {"FRAME_CLASS",     1.0f},  // Quadcopter
    {"FRAME_TYPE",      1.0f},  // X configuration
    {"ARMING_CHECK",    0.0f}   // 0 = Disabled for bench testing
};
const uint16_t kNumParameters = sizeof(kParameters) / sizeof(StoredParam);

// ------------------------------------------------------------------------------
// State Variables
// ------------------------------------------------------------------------------
static uint8_t  g_tx_seq = 0;
static bool     g_is_armed = false;
static uint32_t g_last_1hz_ms = 0;
static uint32_t g_last_2hz_ms = 0;
static uint32_t g_last_4hz_ms = 0;

// ------------------------------------------------------------------------------
// CRC-16 (X.25 / MCRF4XX) Implementation
// ------------------------------------------------------------------------------
static inline void crc_accumulate(uint8_t data, uint16_t *crcAccum) {
    uint8_t tmp = data ^ (uint8_t)(*crcAccum & 0xFF);
    tmp ^= (tmp << 4);
    *crcAccum = (*crcAccum >> 8) ^ (tmp << 8) ^ (tmp << 3) ^ (tmp >> 4);
}

// ------------------------------------------------------------------------------
// MAVLink v2 Packet Serializer
// ------------------------------------------------------------------------------
void send_mavlink_v2(uint32_t msgid, uint8_t crc_extra, const void *payload, uint8_t len) {
    uint8_t header[10];
    header[0] = MAVLINK_STX_V2;
    header[1] = len;
    header[2] = 0x00; // incompat_flags
    header[3] = 0x00; // compat_flags
    header[4] = g_tx_seq++;
    header[5] = SYSTEM_ID;
    header[6] = COMPONENT_ID;
    header[7] = (uint8_t)(msgid & 0xFF);
    header[8] = (uint8_t)((msgid >> 8) & 0xFF);
    header[9] = (uint8_t)((msgid >> 16) & 0xFF);

    uint16_t crc = 0xFFFF;
    for (uint8_t i = 1; i < 10; ++i) {
        crc_accumulate(header[i], &crc);
    }

    const uint8_t *p = (const uint8_t *)payload;
    for (uint8_t i = 0; i < len; ++i) {
        crc_accumulate(p[i], &crc);
    }
    crc_accumulate(crc_extra, &crc);

    uint8_t crc_bytes[2];
    crc_bytes[0] = (uint8_t)(crc & 0xFF);
    crc_bytes[1] = (uint8_t)((crc >> 8) & 0xFF);

    Serial.write(header, 10);
    if (len > 0 && payload != nullptr) {
        Serial.write(p, len);
    }
    Serial.write(crc_bytes, 2);
}

// ------------------------------------------------------------------------------
// Telemetry Transmit Functions
// ------------------------------------------------------------------------------
void send_heartbeat() {
    MsgHeartbeat hb;
    hb.custom_mode = g_is_armed ? COPTER_MODE_GUIDED : COPTER_MODE_STABILIZE;
    hb.type = MAV_TYPE_QUADROTOR;
    hb.autopilot = MAV_AUTOPILOT_ARDUPILOTMEGA;
    hb.base_mode = MAV_MODE_FLAG_CUSTOM_MODE | (g_is_armed ? MAV_MODE_FLAG_SAFETY_ARMED : 0);
    hb.system_status = g_is_armed ? MAV_STATE_ACTIVE : MAV_STATE_STANDBY;
    hb.mavlink_version = 3;

    send_mavlink_v2(MSG_ID_HEARTBEAT, CRC_EXTRA_HEARTBEAT, &hb, sizeof(hb));
}

void send_sys_status() {
    MsgSysStatus ss;
    // Simulated sensors present/enabled/healthy: 3D Gyro, Accel, Mag, Optical Flow, Rangefinder
    ss.onboard_control_sensors_present = 0x0001E03F;
    ss.onboard_control_sensors_enabled = 0x0001E03F;
    ss.onboard_control_sensors_health  = 0x0001E03F;
    ss.load = 320;              // 32.0% MCU load
    ss.voltage_battery = 12580; // 12.58 V (3S LiPo)
    ss.current_battery = 145;   // 1.45 A
    ss.drop_rate_comm = 0;
    ss.errors_comm = 0;
    ss.errors_count1 = 0;
    ss.errors_count2 = 0;
    ss.errors_count3 = 0;
    ss.errors_count4 = 0;
    ss.battery_remaining = 91;  // 91%

    send_mavlink_v2(MSG_ID_SYS_STATUS, CRC_EXTRA_SYS_STATUS, &ss, sizeof(ss));
}

void send_attitude() {
    uint32_t now = millis();
    float t = now * 0.001f;

    MsgAttitude att;
    att.time_boot_ms = now;
    // Gentle oscillation to demonstrate live HUD attitude response
    att.roll  = 0.04f * sin(t * 1.5f);
    att.pitch = 0.02f * cos(t * 1.2f);
    att.yaw   = fmod(t * 0.1f, 6.283185f);
    att.rollspeed  = 0.06f * cos(t * 1.5f);
    att.pitchspeed = -0.024f * sin(t * 1.2f);
    att.yawspeed   = 0.10f;

    send_mavlink_v2(MSG_ID_ATTITUDE, CRC_EXTRA_ATTITUDE, &att, sizeof(att));
}

void send_vfr_hud() {
    MsgVfrHud hud;
    hud.airspeed    = 1.2f;  // [m/s]
    hud.groundspeed = 1.2f;  // [m/s]
    hud.alt         = 1.25f; // [m] Hovering at target gate height
    hud.climb       = 0.0f;  // [m/s]
    hud.heading     = (int16_t)(millis() / 500) % 360;
    hud.throttle    = g_is_armed ? 52 : 0; // [%]

    send_mavlink_v2(MSG_ID_VFR_HUD, CRC_EXTRA_VFR_HUD, &hud, sizeof(hud));
}

void send_local_position_ned() {
    uint32_t now = millis();
    MsgLocalPositionNed pos;
    pos.time_boot_ms = now;
    pos.x  = 1.50f;  // [m] North
    pos.y  = 0.00f;  // [m] East
    pos.z  = -1.25f; // [m] Down (negative represents 1.25m altitude)
    pos.vx = 0.00f;
    pos.vy = 0.00f;
    pos.vz = 0.00f;

    send_mavlink_v2(MSG_ID_LOCAL_POSITION_NED, CRC_EXTRA_LOCAL_POSITION_NED, &pos, sizeof(pos));
}

void send_global_position_int() {
    uint32_t now = millis();
    MsgGlobalPositionInt gpos;
    gpos.time_boot_ms = now;
    // Fixed reference coordinate (San Francisco arena / test location)
    gpos.lat = 377749000;  // 37.7749000 N
    gpos.lon = -1224194000; // -122.4194000 W
    gpos.alt = 15000;       // 15.0m MSL
    gpos.relative_alt = 1250; // 1.25m AGL
    gpos.vx = 0;
    gpos.vy = 0;
    gpos.vz = 0;
    gpos.hdg = 9000;        // 90.00 deg heading

    send_mavlink_v2(MSG_ID_GLOBAL_POSITION_INT, CRC_EXTRA_GLOBAL_POSITION_INT, &gpos, sizeof(gpos));
}

void send_statustext(const char *msg) {
    MsgStatusText st;
    st.severity = 6; // MAV_SEVERITY_INFO
    memset(st.text, 0, sizeof(st.text));
    strncpy(st.text, msg, sizeof(st.text) - 1);

    send_mavlink_v2(MSG_ID_STATUSTEXT, CRC_EXTRA_STATUSTEXT, &st, sizeof(st));
}

void send_parameter_index(uint16_t idx) {
    if (idx >= kNumParameters) return;

    MsgParamValue pv;
    pv.param_count = kNumParameters;
    pv.param_index = idx;
    pv.param_type  = 9; // MAV_PARAM_TYPE_REAL32
    memset(pv.param_id, 0, sizeof(pv.param_id));

    // Read parameter definition from PROGMEM
    StoredParam p;
    memcpy_P(&p, &kParameters[idx], sizeof(StoredParam));

    strncpy(pv.param_id, p.id, sizeof(pv.param_id));
    pv.param_value = p.value;

    send_mavlink_v2(MSG_ID_PARAM_VALUE, CRC_EXTRA_PARAM_VALUE, &pv, sizeof(pv));
}

void send_all_parameters() {
    for (uint16_t i = 0; i < kNumParameters; ++i) {
        send_parameter_index(i);
        delay(12); // Small throttle delay to prevent Uno serial TX buffer overflow
    }
}

void send_command_ack(uint16_t command, uint8_t result) {
    MsgCommandAck ack;
    ack.command = command;
    ack.result  = result;

    send_mavlink_v2(MSG_ID_COMMAND_ACK, CRC_EXTRA_COMMAND_ACK, &ack, sizeof(ack));
}

void send_mission_count() {
    MsgMissionCount mc;
    mc.count = 0; // Empty mission waypoint list
    mc.target_system = 255;
    mc.target_component = 0;

    send_mavlink_v2(MSG_ID_MISSION_COUNT, CRC_EXTRA_MISSION_COUNT, &mc, sizeof(mc));
}

// ------------------------------------------------------------------------------
// Incoming MAVLink Stream Parser
// ------------------------------------------------------------------------------
enum RxState {
    STATE_IDLE,
    STATE_V1_LEN,
    STATE_V1_SEQ,
    STATE_V1_SYSID,
    STATE_V1_COMPID,
    STATE_V1_MSGID,
    STATE_V1_PAYLOAD,
    STATE_V1_CRC,
    STATE_V2_LEN,
    STATE_V2_INCOMPAT,
    STATE_V2_COMPAT,
    STATE_V2_SEQ,
    STATE_V2_SYSID,
    STATE_V2_COMPID,
    STATE_V2_MSGID_0,
    STATE_V2_MSGID_1,
    STATE_V2_MSGID_2,
    STATE_V2_PAYLOAD,
    STATE_V2_CRC
};

static RxState  g_rx_state = STATE_IDLE;
static uint8_t  g_rx_len = 0;
static uint32_t g_rx_msgid = 0;
static uint8_t  g_rx_idx = 0;
static uint8_t  g_rx_payload[64];

void process_incoming_packet(uint32_t msgid, const uint8_t *payload, uint8_t len) {
    if (msgid == MSG_ID_PARAM_REQUEST_LIST) {
        // Mission Planner initial parameter sync
        send_all_parameters();
    }
    else if (msgid == MSG_ID_PARAM_REQUEST_READ && len >= 2) {
        int16_t req_idx = (int16_t)(payload[0] | (payload[1] << 8));
        if (req_idx >= 0 && req_idx < (int16_t)kNumParameters) {
            send_parameter_index(req_idx);
        } else {
            send_parameter_index(0);
        }
    }
    else if (msgid == MSG_ID_MISSION_REQUEST_LIST) {
        // Waypoint request
        send_mission_count();
    }
    else if (msgid == MSG_ID_COMMAND_LONG && len >= 30) {
        uint16_t cmd = (uint16_t)(payload[28] | (payload[29] << 8));
        if (cmd == MAV_CMD_COMPONENT_ARM_DISARM) {
            float arm_param;
            memcpy(&arm_param, payload, 4);
            g_is_armed = (arm_param >= 1.0f);
            send_command_ack(cmd, MAV_RESULT_ACCEPTED);
            send_statustext(g_is_armed ? "FCU: Motors Armed" : "FCU: Motors Disarmed");
        } else {
            // Acknowledge other commands
            send_command_ack(cmd, MAV_RESULT_ACCEPTED);
        }
    }
}

void parse_rx_byte(uint8_t b) {
    switch (g_rx_state) {
        case STATE_IDLE:
            if (b == MAVLINK_STX_V2) {
                g_rx_state = STATE_V2_LEN;
            } else if (b == MAVLINK_STX_V1) {
                g_rx_state = STATE_V1_LEN;
            }
            break;

        // --- MAVLink v2 Branch ---
        case STATE_V2_LEN:
            g_rx_len = b;
            g_rx_state = STATE_V2_INCOMPAT;
            break;
        case STATE_V2_INCOMPAT:
            g_rx_state = STATE_V2_COMPAT;
            break;
        case STATE_V2_COMPAT:
            g_rx_state = STATE_V2_SEQ;
            break;
        case STATE_V2_SEQ:
            g_rx_state = STATE_V2_SYSID;
            break;
        case STATE_V2_SYSID:
            g_rx_state = STATE_V2_COMPID;
            break;
        case STATE_V2_COMPID:
            g_rx_state = STATE_V2_MSGID_0;
            break;
        case STATE_V2_MSGID_0:
            g_rx_msgid = b;
            g_rx_state = STATE_V2_MSGID_1;
            break;
        case STATE_V2_MSGID_1:
            g_rx_msgid |= ((uint32_t)b << 8);
            g_rx_state = STATE_V2_MSGID_2;
            break;
        case STATE_V2_MSGID_2:
            g_rx_msgid |= ((uint32_t)b << 16);
            g_rx_idx = 0;
            g_rx_state = (g_rx_len > 0) ? STATE_V2_PAYLOAD : STATE_V2_CRC;
            break;
        case STATE_V2_PAYLOAD:
            if (g_rx_idx < sizeof(g_rx_payload)) {
                g_rx_payload[g_rx_idx] = b;
            }
            g_rx_idx++;
            if (g_rx_idx >= g_rx_len) {
                g_rx_idx = 0;
                g_rx_state = STATE_V2_CRC;
            }
            break;
        case STATE_V2_CRC:
            g_rx_idx++;
            if (g_rx_idx >= 2) {
                process_incoming_packet(g_rx_msgid, g_rx_payload, g_rx_len);
                g_rx_state = STATE_IDLE;
            }
            break;

        // --- MAVLink v1 Branch ---
        case STATE_V1_LEN:
            g_rx_len = b;
            g_rx_state = STATE_V1_SEQ;
            break;
        case STATE_V1_SEQ:
            g_rx_state = STATE_V1_SYSID;
            break;
        case STATE_V1_SYSID:
            g_rx_state = STATE_V1_COMPID;
            break;
        case STATE_V1_COMPID:
            g_rx_state = STATE_V1_MSGID;
            break;
        case STATE_V1_MSGID:
            g_rx_msgid = b;
            g_rx_idx = 0;
            g_rx_state = (g_rx_len > 0) ? STATE_V1_PAYLOAD : STATE_V1_CRC;
            break;
        case STATE_V1_PAYLOAD:
            if (g_rx_idx < sizeof(g_rx_payload)) {
                g_rx_payload[g_rx_idx] = b;
            }
            g_rx_idx++;
            if (g_rx_idx >= g_rx_len) {
                g_rx_idx = 0;
                g_rx_state = STATE_V1_CRC;
            }
            break;
        case STATE_V1_CRC:
            g_rx_idx++;
            if (g_rx_idx >= 2) {
                process_incoming_packet(g_rx_msgid, g_rx_payload, g_rx_len);
                g_rx_state = STATE_IDLE;
            }
            break;

        default:
            g_rx_state = STATE_IDLE;
            break;
    }
}

// ------------------------------------------------------------------------------
// Setup & Main Loop
// ------------------------------------------------------------------------------
void setup() {
    pinMode(LED_BUILTIN, OUTPUT);
    digitalWrite(LED_BUILTIN, LOW);

    // Initialize UART at 115200 baud
    Serial.begin(MAVLINK_BAUD);

    // Wait briefly for serial pipeline to stabilize
    delay(500);

    // Send startup announcement
    send_heartbeat();
    send_statustext("ArduPilot Uno FCU Emulator Ready");
}

void loop() {
    uint32_t now = millis();

    // 1. Process incoming commands from Pi / Proxy / Mission Planner
    while (Serial.available() > 0) {
        uint8_t c = (uint8_t)Serial.read();
        parse_rx_byte(c);

        // Simple manual key controls via Serial Monitor
        if (c == 'p' || c == 'P') {
            send_all_parameters();
        } else if (c == 'a' || c == 'A') {
            g_is_armed = !g_is_armed;
            send_statustext(g_is_armed ? "Manual: Armed" : "Manual: Disarmed");
        }
    }

    // 2. High-rate loop (4 Hz / 250 ms): Attitude telemetry
    if (now - g_last_4hz_ms >= 250) {
        g_last_4hz_ms = now;
        send_attitude();
    }

    // 3. Medium-rate loop (2 Hz / 500 ms): Local position and HUD data
    if (now - g_last_2hz_ms >= 500) {
        g_last_2hz_ms = now;
        send_local_position_ned();
        send_vfr_hud();
    }

    // 4. Low-rate loop (1 Hz / 1000 ms): Heartbeat, Status, Global GPS
    if (now - g_last_1hz_ms >= 1000) {
        g_last_1hz_ms = now;
        send_heartbeat();
        send_sys_status();
        send_global_position_int();

        // Heartbeat visual indicator on Pin 13
        if (g_is_armed) {
            digitalWrite(LED_BUILTIN, HIGH); // Solid ON when armed
        } else {
            digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN)); // Blink when standby
        }
    }
}
