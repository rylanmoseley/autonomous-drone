#pragma once
#include <string>
#include <sys/socket.h>
#include <netinet/in.h>
#include "ui_protocol.h"
#include "telemetry_packet.h"

class TelemetryBridge {
public:
    TelemetryBridge();
    ~TelemetryBridge();

    bool openSockets(int ui_port = 14552, int vision_port = 14553, const std::string& telem_ip = "127.0.0.1", int telem_port = 14554);
    void closeSockets();

    bool sendUIStatus(const struct sockaddr_in& dest, socklen_t dest_len, const UIStatusPacket& status);
    bool sendTelemetry(const TelemetryPacket& pkt);

    int getUiSocket() const { return ui_sock_; }
    int getVisionSocket() const { return vision_sock_; }
    int getTelemSocket() const { return telem_sock_; }
    const struct sockaddr_in& getTelemDest() const { return telem_dest_; }

private:
    int ui_sock_{-1};
    int vision_sock_{-1};
    int telem_sock_{-1};
    struct sockaddr_in telem_dest_{};
};
