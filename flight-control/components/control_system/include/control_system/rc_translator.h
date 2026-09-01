#pragma once
#include "control_system/control_system.h"

struct RCState {
    float throttle = 0.0f; // 0.0 to 1.0
    float yaw = 0.0f;      // -1.0 to 1.0
    float pitch = 0.0f;    // -1.0 to 1.0
    float roll = 0.0f;     // -1.0 to 1.0
};

class RCTranslator {
public:
    // Translates consumer RC inputs (Mode 2) into the 6D spatial command vector
    // expected by the lower-level ControlSystem mixer.
    static CommandVector translate(const RCState& rc) {
        CommandVector cmd;
        cmd.lz = rc.throttle;
        cmd.az = rc.yaw;
        cmd.ax = rc.pitch;
        cmd.ay = rc.roll;
        
        // lx and ly are left at 0, as standard RC commands attitude (pitch/roll) 
        // rather than lateral linear translations directly.
        cmd.lx = 0.0f;
        cmd.ly = 0.0f;
        
        return cmd;
    }
};
