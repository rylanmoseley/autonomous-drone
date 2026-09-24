#include <gtest/gtest.h>
#include "mavlink_daemon.h"
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <thread>

TEST(MavlinkDaemonTest, SendAndReceive) {
    MavlinkDaemon daemon(24550, "127.0.0.1", 24551);
    daemon.start();

    // Setup mock FCU socket
    int fcu_sock = socket(AF_INET, SOCK_DGRAM, 0);
    struct sockaddr_in fcu_addr;
    fcu_addr.sin_family = AF_INET;
    fcu_addr.sin_port = htons(24551);
    fcu_addr.sin_addr.s_addr = INADDR_ANY;
    bind(fcu_sock, (struct sockaddr *)&fcu_addr, sizeof(fcu_addr));

    struct timeval tv;
    tv.tv_sec = 0;
    tv.tv_usec = 100000;
    setsockopt(fcu_sock, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));

    // Test sendFlightCommand
    FlightCommand cmd;
    cmd.target_x = 10.0;
    daemon.sendFlightCommand(cmd);

    char buffer[1024];
    struct sockaddr_in from_addr;
    socklen_t from_len = sizeof(from_addr);
    int n = recvfrom(fcu_sock, buffer, sizeof(buffer), 0, (struct sockaddr *)&from_addr, &from_len);
    
    EXPECT_EQ(n, sizeof(FlightCommand));
    if (n == sizeof(FlightCommand)) {
        FlightCommand received_cmd;
        memcpy(&received_cmd, buffer, sizeof(FlightCommand));
        EXPECT_DOUBLE_EQ(received_cmd.target_x, 10.0);
    }

    // Test getTelemetry
    Telemetry telem;
    telem.battery_level = 42;
    telem.error_state = true;
    
    struct sockaddr_in daemon_addr;
    daemon_addr.sin_family = AF_INET;
    daemon_addr.sin_port = htons(24550);
    inet_pton(AF_INET, "127.0.0.1", &daemon_addr.sin_addr);
    
    sendto(fcu_sock, &telem, sizeof(Telemetry), 0, (struct sockaddr *)&daemon_addr, sizeof(daemon_addr));
    
    std::this_thread::sleep_for(std::chrono::milliseconds(50)); // let rx_thread process
    Telemetry current_telem = daemon.getTelemetry();
    EXPECT_EQ(current_telem.battery_level, 42);
    EXPECT_TRUE(current_telem.error_state);

    daemon.stop();
    close(fcu_sock);
}
