#include "telemetry_bridge.h"
#include <arpa/inet.h>
#include <unistd.h>
#include <cstring>
#include <iostream>

TelemetryBridge::TelemetryBridge() {}

TelemetryBridge::~TelemetryBridge() {
    closeSockets();
}

bool TelemetryBridge::openSockets(int ui_port, int vision_port, const std::string& telem_ip, int telem_port) {
    closeSockets();

    struct timeval tv;
    tv.tv_sec = 0;
    tv.tv_usec = 100000;

    // UI Socket
    ui_sock_ = socket(AF_INET, SOCK_DGRAM, 0);
    if (ui_sock_ >= 0) {
        struct sockaddr_in ui_addr;
        std::memset(&ui_addr, 0, sizeof(ui_addr));
        ui_addr.sin_family = AF_INET;
        ui_addr.sin_port = htons(ui_port);
        ui_addr.sin_addr.s_addr = INADDR_ANY;
        bind(ui_sock_, (struct sockaddr *)&ui_addr, sizeof(ui_addr));
        setsockopt(ui_sock_, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));
    }

    // Vision Socket
    vision_sock_ = socket(AF_INET, SOCK_DGRAM, 0);
    if (vision_sock_ >= 0) {
        struct sockaddr_in vision_addr;
        std::memset(&vision_addr, 0, sizeof(vision_addr));
        vision_addr.sin_family = AF_INET;
        vision_addr.sin_port = htons(vision_port);
        vision_addr.sin_addr.s_addr = INADDR_ANY;
        bind(vision_sock_, (struct sockaddr *)&vision_addr, sizeof(vision_addr));
        setsockopt(vision_sock_, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));
    }

    // Telemetry Broadcast Socket
    telem_sock_ = socket(AF_INET, SOCK_DGRAM, 0);
    if (telem_sock_ >= 0) {
        std::memset(&telem_dest_, 0, sizeof(telem_dest_));
        telem_dest_.sin_family = AF_INET;
        telem_dest_.sin_port = htons(telem_port);
        inet_pton(AF_INET, telem_ip.c_str(), &telem_dest_.sin_addr);
    }

    return (ui_sock_ >= 0 && vision_sock_ >= 0 && telem_sock_ >= 0);
}

void TelemetryBridge::closeSockets() {
    if (ui_sock_ >= 0) { close(ui_sock_); ui_sock_ = -1; }
    if (vision_sock_ >= 0) { close(vision_sock_); vision_sock_ = -1; }
    if (telem_sock_ >= 0) { close(telem_sock_); telem_sock_ = -1; }
}

bool TelemetryBridge::sendUIStatus(const struct sockaddr_in& dest, socklen_t dest_len, const UIStatusPacket& status) {
    if (ui_sock_ < 0 || dest_len == 0) return false;
    ssize_t sent = sendto(ui_sock_, &status, sizeof(status), 0, (struct sockaddr *)&dest, dest_len);
    return (sent == static_cast<ssize_t>(sizeof(status)));
}

bool TelemetryBridge::sendTelemetry(const TelemetryPacket& pkt) {
    if (telem_sock_ < 0) return false;
    ssize_t sent = sendto(telem_sock_, &pkt, sizeof(pkt), 0, (struct sockaddr *)&telem_dest_, sizeof(telem_dest_));
    return (sent == static_cast<ssize_t>(sizeof(pkt)));
}
