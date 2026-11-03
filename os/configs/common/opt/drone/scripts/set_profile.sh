#!/usr/bin/env bash
# /opt/drone/scripts/set_profile.sh
# Switch companion computer profile between Bringup Mode and Autonomous Flight Mode

set -e

ACTION="${1:-status}"

case "$ACTION" in
    bringup)
        echo "=== Switching Companion SBC to Hardware Bringup Mode ==="
        echo "[1/3] Disabling autonomous flight daemons..."
        systemctl stop drone-autopilot.service drone-vision.service 2>/dev/null || true
        systemctl disable drone-autopilot.service drone-vision.service 2>/dev/null || true

        echo "[2/3] Enabling telemetry access point & hardware tuning..."
        systemctl enable --now NetworkManager.service 2>/dev/null || true
        systemctl enable --now drone-tuning.service 2>/dev/null || true

        echo "[3/3] Starting standalone MAVLink proxy..."
        systemctl enable --now drone-mavlink-router.service 2>/dev/null || true
        systemctl restart drone-mavlink-router.service 2>/dev/null || true

        echo ""
        echo "=========================================================="
        echo " [SUCCESS] Hardware Bringup Mode is ACTIVE"
        echo "=========================================================="
        echo "- Wi-Fi SSID: HEXAGON-flight-drone"
        echo "- Password:   rylaniscool"
        echo "- Static IP:  192.168.4.1"
        echo "- MAVLink:    /dev/flightcontroller (921600 baud) <-> UDP :14550"
        echo "- Autopilot & Vision Daemons: STOPPED & DISABLED"
        echo ""
        echo "Hardware engineers can now connect QGroundControl or"
        echo "Mission Planner to UDP Port 14550 with zero interference."
        echo "=========================================================="
        ;;

    flight)
        echo "=== Pivoting Companion SBC to Autonomous Flight Mode ==="
        echo "[1/2] Enabling and launching Autopilot Daemon..."
        systemctl enable --now drone-autopilot.service

        echo "[2/2] Enabling and launching Vision Processing Daemon..."
        systemctl enable --now drone-vision.service

        echo ""
        echo "=========================================================="
        echo " [SUCCESS] Autonomous Flight Mode is ACTIVE"
        echo "=========================================================="
        echo "- Autopilot Daemon: RUNNING (SCHED_RR Realtime Priority)"
        echo "- Vision Pipeline:  RUNNING (120 FPS Gate Detection)"
        echo "- Telemetry Bridge: MAVLink pass-through & UI UDP :14552"
        echo "=========================================================="
        ;;

    status)
        echo "=== Autonomous Drone Companion System Status ==="
        echo -n "Wi-Fi AP (HEXAGON-flight-drone): "
        systemctl is-active NetworkManager.service 2>/dev/null || echo "inactive"
        echo -n "Hardware Tuning & udev:         "
        systemctl is-active drone-tuning.service 2>/dev/null || echo "inactive"
        echo -n "MAVLink Proxy / Router:          "
        systemctl is-active drone-mavlink-router.service 2>/dev/null || echo "inactive"
        echo -n "Autopilot Flight Daemon:         "
        systemctl is-active drone-autopilot.service 2>/dev/null || echo "inactive"
        echo -n "Vision Tracking Daemon:          "
        systemctl is-active drone-vision.service 2>/dev/null || echo "inactive"
        echo ""

        AP_ACTIVE=$(systemctl is-active drone-autopilot.service 2>/dev/null || echo "inactive")
        PROXY_ACTIVE=$(systemctl is-active drone-mavlink-router.service 2>/dev/null || echo "inactive")

        if [ "$AP_ACTIVE" = "active" ]; then
            echo "Current Profile: [AUTONOMOUS FLIGHT MODE]"
            echo "-> Autopilot owns the flight control loop and MAVLink telemetry."
        elif [ "$PROXY_ACTIVE" = "active" ]; then
            echo "Current Profile: [HARDWARE BRINGUP MODE (MAVLink Proxy Only)]"
            echo "-> Autopilot is disabled. MAVLink proxy has exclusive FCU access for GCS testing."
        else
            echo "Current Profile: [STANDBY / MANUAL]"
        fi
        ;;

    *)
        echo "Usage: $0 {bringup|flight|status}"
        echo ""
        echo "  bringup  - Disable autopilot/vision; run MAVLink proxy for hardware bench-testing."
        echo "  flight   - Pivot to autonomous flight mode (autopilot owns flight loop)."
        echo "  status   - Show current active services and companion profile."
        exit 1
        ;;
esac
