// =========================================================================================
// Drone Vision System: 6-DoF Gate Pose Estimation via cv::solvePnP & UDP Autopilot Telemetry
// =========================================================================================
// Computes full 3D position (X, Y, Z in meters) and orientation (Pitch, Yaw, Roll):
//   - Uses calibrated camera matrix K (FOCAL_LENGTH = 566.8)
//   - Matches detected 2D corners from approxPolyDP to known 3D gate dimensions (in meters)
//   - Projects 3D coordinate frame axes directly onto the gate opening
//   - Transmits VisionGoalEstimate UDP packets to autopilot node (port 14553)
//   - Supports headless execution for embedded SBCs (Radxa Zero 3W / Pi Zero 2W)
// =========================================================================================

#include <iostream>
#include <vector>
#include <string>
#include <cmath>
#include <chrono>
#include <thread>
#include <csignal>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>

#include "gate_detector.h"
#include "vision_packet.h"

static volatile sig_atomic_t g_running = 1;

void signalHandler(int) {
    g_running = 0;
}

int main(int argc, char** argv) {
    std::signal(SIGINT, signalHandler);
    std::signal(SIGTERM, signalHandler);

    // CLI configuration options
    bool headless = false;
    std::string camera_source = "0";
    std::string target_ip = "127.0.0.1";
    int target_port = 14553;
    int udp_rx_port = 0;
    int feedback_port = 14556;

    for (int i = 1; i < argc; ++i) {
        std::string arg = argv[i];
        if (arg == "--headless" || arg == "-h") {
            headless = true;
        } else if (arg == "--camera" && i + 1 < argc) {
            camera_source = argv[++i];
        } else if (arg == "--ip" && i + 1 < argc) {
            target_ip = argv[++i];
        } else if (arg == "--port" && i + 1 < argc) {
            target_port = std::stoi(argv[++i]);
        } else if (arg == "--udp" && i + 1 < argc) {
            udp_rx_port = std::stoi(argv[++i]);
        } else if (arg == "--sim") {
            udp_rx_port = 14555;
        } else if (arg == "--feedback-port" && i + 1 < argc) {
            feedback_port = std::stoi(argv[++i]);
        } else if (arg == "--help") {
            std::cout << "Usage: " << argv[0] << " [options]\n"
                      << "  --headless, -h      Run in headless mode without GUI display windows\n"
                      << "  --camera <src>      Camera device index (default: 0) or video file path\n"
                      << "  --udp <port>        Receive JPEG camera frames from simulator via UDP (e.g. 14555)\n"
                      << "  --sim               Alias for --udp 14555 (simulation harness stream)\n"
                      << "  --ip <address>      Autopilot target IP address (default: 127.0.0.1)\n"
                      << "  --port <port>       Autopilot UDP port (default: 14553)\n"
                      << "  --feedback-port <p> Feedback telemetry UDP port for sim visualizer (default: 14556)\n"
                      << "  --help              Show this help message\n";
            return 0;
        }
    }

    if (camera_source == "udp" || camera_source == "sim") {
        udp_rx_port = 14555;
    }

    // Auto-detect display environment
    const char* display_env = std::getenv("DISPLAY");
    if (!headless && (!display_env || std::string(display_env).empty())) {
        std::cout << "[INFO] No DISPLAY detected. Auto-switching to headless mode." << std::endl;
        headless = true;
    }

    // Initialize UDP transmission socket for autopilot telemetry
    int udp_sock = socket(AF_INET, SOCK_DGRAM, 0);
    if (udp_sock < 0) {
        std::cerr << "[ERROR] Failed to create UDP socket." << std::endl;
        return -1;
    }

    struct sockaddr_in target_addr{};
    target_addr.sin_family = AF_INET;
    target_addr.sin_port = htons(target_port);
    if (inet_pton(AF_INET, target_ip.c_str(), &target_addr.sin_addr) <= 0) {
        std::cerr << "[ERROR] Invalid target IP address: " << target_ip << std::endl;
        close(udp_sock);
        return -1;
    }

    struct sockaddr_in feedback_addr{};
    feedback_addr.sin_family = AF_INET;
    feedback_addr.sin_port = htons(feedback_port);
    inet_pton(AF_INET, target_ip.c_str(), &feedback_addr.sin_addr);

    // Initialize UDP receiver if streaming from simulator
    int img_rx_sock = -1;
    if (udp_rx_port > 0) {
        img_rx_sock = socket(AF_INET, SOCK_DGRAM, 0);
        struct sockaddr_in rx_addr{};
        rx_addr.sin_family = AF_INET;
        rx_addr.sin_port = htons(udp_rx_port);
        rx_addr.sin_addr.s_addr = INADDR_ANY;
        int opt = 1;
        setsockopt(img_rx_sock, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));
        struct timeval tv{0, 100000}; // 100ms timeout
        setsockopt(img_rx_sock, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));
        if (bind(img_rx_sock, (struct sockaddr*)&rx_addr, sizeof(rx_addr)) < 0) {
            std::cerr << "[WARN] Failed to bind image RX UDP port " << udp_rx_port << ", falling back to camera index." << std::endl;
            close(img_rx_sock);
            img_rx_sock = -1;
        } else {
            std::cout << "[INFO] Piped camera view active: Listening for UDP JPEG frames on port " << udp_rx_port << std::endl;
        }
    }

    cv::VideoCapture cap;
    if (img_rx_sock < 0) {
        if (camera_source.find_first_not_of("0123456789") == std::string::npos) {
            cap.open(std::stoi(camera_source));
        } else {
            cap.open(camera_source);
        }

        if (!cap.isOpened()) {
            std::cout << "[INFO] Camera index " << camera_source << " not found, auto-fallback to UDP sim stream (port 14555)." << std::endl;
            img_rx_sock = socket(AF_INET, SOCK_DGRAM, 0);
            struct sockaddr_in rx_addr{};
            rx_addr.sin_family = AF_INET;
            rx_addr.sin_port = htons(14555);
            rx_addr.sin_addr.s_addr = INADDR_ANY;
            int opt = 1;
            setsockopt(img_rx_sock, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));
            struct timeval tv{0, 100000};
            setsockopt(img_rx_sock, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));
            if (bind(img_rx_sock, (struct sockaddr*)&rx_addr, sizeof(rx_addr)) < 0) {
                std::cerr << "[ERROR] Failed to bind image RX UDP port 14555." << std::endl;
                close(img_rx_sock);
                img_rx_sock = -1;
                close(udp_sock);
                return -1;
            }
            udp_rx_port = 14555;
        }
    }

    std::cout << "========================================================\n"
              << "Drone Vision Node Initialized\n"
              << "  Camera Source : " << (img_rx_sock >= 0 ? ("UDP Port " + std::to_string(udp_rx_port) + " (Simulator PiP Pipe)") : camera_source) << "\n"
              << "  Autopilot UDP : " << target_ip << ":" << target_port << "\n"
              << "  Feedback UDP  : " << target_ip << ":" << feedback_port << "\n"
              << "  Mode          : " << (headless ? "Headless (Onboard SBC)" : "Interactive GUI") << "\n"
              << "  Square Width  : " << vision::KNOWN_SQUARE_WIDTH_M << " m (" << vision::KNOWN_SQUARE_WIDTH_IN << " in)\n"
              << "  Pentagon Width: " << vision::KNOWN_PENTAGON_WIDTH_M << " m (" << vision::KNOWN_PENTAGON_WIDTH_IN << " in)\n"
              << "========================================================" << std::endl;

    vision::GateDetector detector(vision::FOCAL_LENGTH);
    if (udp_rx_port > 0 || camera_source == "udp" || camera_source == "sim") {
        detector.setSimMode(true);
    }
    cv::Mat frame;
    uint32_t sequence_id = 0;
    std::vector<uchar> img_buf(65536);
    auto last_heartbeat = std::chrono::steady_clock::now();

    while (g_running) {
        if (img_rx_sock >= 0) {
            struct sockaddr_in src_addr{};
            socklen_t src_len = sizeof(src_addr);
            ssize_t n = recvfrom(img_rx_sock, img_buf.data(), img_buf.size(), 0, (struct sockaddr*)&src_addr, &src_len);
            if (n <= 0) {
                // Heartbeat to keep autopilot watchdog alive
                auto now = std::chrono::steady_clock::now();
                if (std::chrono::duration<double>(now - last_heartbeat).count() >= 0.1) {
                    VisionPacket empty_pkt{};
                    empty_pkt.msg_type = MSG_TYPE_VISION;
                    empty_pkt.sequence_id = sequence_id++;
                    empty_pkt.timestamp_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
                        now.time_since_epoch()).count();
                    empty_pkt.num_goals_reported = 0;
                    empty_pkt.num_tags_detected = 0;
                    sendto(udp_sock, &empty_pkt, sizeof(empty_pkt), 0, (struct sockaddr*)&target_addr, sizeof(target_addr));
                    last_heartbeat = now;
                }
                continue;
            }
            cv::Mat rawData(1, static_cast<int>(n), CV_8UC1, img_buf.data());
            frame = cv::imdecode(rawData, cv::IMREAD_COLOR);
            if (frame.empty()) {
                continue;
            }
        } else {
            cap >> frame;
            if (frame.empty()) {
                if (camera_source.find_first_not_of("0123456789") != std::string::npos) {
                    cap.set(cv::CAP_PROP_POS_FRAMES, 0);
                    continue;
                }
                break;
            }
        }

        sequence_id++;
        uint64_t timestamp_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
            std::chrono::steady_clock::now().time_since_epoch()).count();
        last_heartbeat = std::chrono::steady_clock::now();

        auto results = detector.detectAllGates(frame, !headless);
        VisionPacket packet = vision::GateDetector::toVisionPacket(results, sequence_id, timestamp_ns);

        // Send to Autopilot
        sendto(udp_sock, &packet, sizeof(packet), 0, (struct sockaddr*)&target_addr, sizeof(target_addr));

        // Send copy to Simulator feedback port
        sendto(udp_sock, &packet, sizeof(packet), 0, (struct sockaddr*)&feedback_addr, sizeof(feedback_addr));

        if (!results.empty() && headless) {
            for (const auto& res : results) {
                std::cout << "[VISION] Gate detected (" << res.shape 
                          << "): Pos(X,Y,Z)=[" << res.posX_m << ", " << res.posY_m << ", " << res.posZ_m << "] m "
                          << "UDP -> " << target_ip << ":" << target_port << std::endl;
            }
        }

        if (!headless) {
            cv::imshow("Advanced Drone Vision (solvePnP)", frame);
            int key = cv::waitKey(1);
            if (key == 'q' || key == 27) break;
        }
    }

    std::cout << "[INFO] Shutting down vision node." << std::endl;
    if (img_rx_sock >= 0) close(img_rx_sock);
    if (cap.isOpened()) cap.release();
    if (!headless) cv::destroyAllWindows();
    close(udp_sock);
    return 0;
}
