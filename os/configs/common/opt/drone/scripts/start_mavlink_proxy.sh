#!/usr/bin/env bash
# /opt/drone/scripts/start_mavlink_proxy.sh
# Launcher for MAVLink proxy / routing service

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# 1. Check if mavlink-routerd is installed
if command -v mavlink-routerd >/dev/null 2>&1; then
    CONFIG_FILE="/etc/mavlink-router/main.conf"
    if [ -f "$CONFIG_FILE" ]; then
        echo "[MAVLINK-ROUTER] Launching mavlink-routerd with $CONFIG_FILE..."
        exec mavlink-routerd -c "$CONFIG_FILE"
    else
        echo "[MAVLINK-ROUTER] Launching mavlink-routerd (default routing)..."
        exec mavlink-routerd -e 127.0.0.1:14550 -e 192.168.4.255:14550 /dev/flightcontroller:921600
    fi
fi

# 2. Check if mavproxy is installed
if command -v mavproxy.py >/dev/null 2>&1; then
    echo "[MAVLINK-ROUTER] Launching mavproxy.py..."
    exec mavproxy.py --master=/dev/flightcontroller --baudrate=921600 \
        --out=udp:127.0.0.1:14550 \
        --out=udpbcast:192.168.4.255:14550 \
        --in=udp:127.0.0.1:14551 --daemon
fi

# 3. Fallback to built-in zero-dependency Python proxy
echo "[MAVLINK-ROUTER] Launching built-in Python MAVLink proxy..."
exec python3 "$SCRIPT_DIR/mavlink_proxy.py" \
    --serial /dev/flightcontroller \
    --baud 921600 \
    --ap-rx-ip 127.0.0.1 \
    --ap-rx-port 14550 \
    --ap-tx-port 14551 \
    --gcs-broadcast-ip 192.168.4.255 \
    --gcs-port 14550 \
    --gcs-in-port 14555
