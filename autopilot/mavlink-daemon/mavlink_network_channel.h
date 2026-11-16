#pragma once
#include <string>
#include <thread>
#include <atomic>
#include <functional>
#include <sys/socket.h>
#include <netinet/in.h>

class MavlinkNetworkChannel {
public:
    using PacketCallback = std::function<void(const uint8_t*, size_t)>;

    MavlinkNetworkChannel(int local_port, const std::string& remote_ip, int remote_port);
    ~MavlinkNetworkChannel();

    bool openChannel();
    void closeChannel();

    bool startReceiver(PacketCallback callback);
    void stopReceiver();

    bool sendData(const void* data, size_t len);

    bool isOpen() const { return socket_fd_ >= 0; }
    bool isRunning() const { return running_; }
    int getLocalPort() const { return local_port_; }
    const std::string& getRemoteIp() const { return remote_ip_; }
    int getRemotePort() const { return remote_port_; }

private:
    void receiveLoop();

    int local_port_;
    std::string remote_ip_;
    int remote_port_;
    int socket_fd_;

    std::atomic<bool> running_;
    std::thread rx_thread_;
    PacketCallback callback_;
};
