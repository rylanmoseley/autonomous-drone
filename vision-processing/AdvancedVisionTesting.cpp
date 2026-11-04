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
        } else if (arg == "--help") {
            std::cout << "Usage: " << argv[0] << " [options]\n"
                      << "  --headless, -h    Run in headless mode without GUI display windows\n"
                      << "  --camera <src>    Camera device index (default: 0) or video file path\n"
                      << "  --ip <address>    Autopilot target IP address (default: 127.0.0.1)\n"
                      << "  --port <port>     Autopilot UDP port (default: 14553)\n"
                      << "  --help            Show this help message\n";
            return 0;
        }
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

    std::cout << "========================================================\n"
              << "Drone Vision Node Initialized\n"
              << "  Camera Source : " << camera_source << "\n"
              << "  Autopilot UDP : " << target_ip << ":" << target_port << "\n"
              << "  Mode          : " << (headless ? "Headless (Onboard SBC)" : "Interactive GUI") << "\n"
              << "  Square Width  : " << vision::KNOWN_SQUARE_WIDTH_M << " m (" << vision::KNOWN_SQUARE_WIDTH_IN << " in)\n"
              << "  Pentagon Width: " << vision::KNOWN_PENTAGON_WIDTH_M << " m (" << vision::KNOWN_PENTAGON_WIDTH_IN << " in)\n"
              << "========================================================" << std::endl;

    cv::VideoCapture cap;
    if (camera_source.find_first_not_of("0123456789") == std::string::npos) {
        cap.open(std::stoi(camera_source));
    } else {
        cap.open(camera_source);
    }

    if (!cap.isOpened()) {
        std::cerr << "[ERROR] Failed to open camera source: " << camera_source << std::endl;
        close(udp_sock);
        return -1;
    }

    vision::GateDetector detector(vision::FOCAL_LENGTH);
    cv::Mat frame;
    uint32_t sequence_id = 0;

    while (g_running) {
        cap >> frame;
        if (frame.empty()) {
            // For video file loops or camera frame drops
            if (camera_source.find_first_not_of("0123456789") != std::string::npos) {
                // Rewind video file
                cap.set(cv::CAP_PROP_POS_FRAMES, 0);
                continue;
            }
            break;
        }

        sequence_id++;
        uint64_t timestamp_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
            std::chrono::steady_clock::now().time_since_epoch()).count();

        vision::GateDetectionResult result = detector.processFrame(frame, !headless);

        if (result.detected) {
            VisionPacket packet = vision::GateDetector::toVisionPacket(result, sequence_id, timestamp_ns);

            ssize_t sent = sendto(udp_sock, &packet, sizeof(packet), 0,
                                  (struct sockaddr*)&target_addr, sizeof(target_addr));
            if (sent != sizeof(packet)) {
                std::cerr << "[WARN] Partial UDP packet sent: " << sent << " / " << sizeof(packet) << " bytes" << std::endl;
            }

            if (headless) {
                std::cout << "[VISION] Gate detected (" << result.shape 
                          << "): Pos(X,Y,Z)=[" << result.posX_m << ", " << result.posY_m << ", " << result.posZ_m << "] m "
                          << "Rot(P,Y,R)=[" << (int)result.pitch_deg << ", " << (int)result.yaw_deg << ", " << (int)result.roll_deg << "] deg "
                          << "UDP -> " << target_ip << ":" << target_port << std::endl;
            }
        }

        if (!headless) {
            cv::imshow("Advanced Drone Vision (solvePnP)", frame);
            int key = cv::waitKey(1);
            if (key == 'q' || key == 27) break;
        } else {
            // Regulate loop rate (~30 FPS) in headless mode
            std::this_thread::sleep_for(std::chrono::milliseconds(30));
        }
    }

    std::cout << "[INFO] Shutting down vision node." << std::endl;
    cap.release();
    if (!headless) {
        cv::destroyAllWindows();
    }
    close(udp_sock);
    return 0;
}
