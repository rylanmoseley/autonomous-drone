#!/usr/bin/env bash
# /opt/drone/scripts/apply_tuning.sh
# Realtime performance locking for drone flight computer

set -e

echo "[DRONE-TUNING] Applying realtime performance optimizations..."

# 1. Lock CPU Governors to Maximum Frequency Performance
if [ -d /sys/devices/system/cpu/cpu0/cpufreq ]; then
    for governor in /sys/devices/system/cpu/cpu*/cpufreq/scaling_governor; do
        if [ -f "$governor" ]; then
            echo "performance" > "$governor" 2>/dev/null || true
        fi
    done
    echo "[DRONE-TUNING] Set CPU scaling governor to performance."
fi

# 2. Network Transmit Queue Length
for iface in wlan0 eth0 usb0; do
    if [ -d "/sys/class/net/$iface" ]; then
        ip link set txqueuelen 1000 dev "$iface" 2>/dev/null || true
    fi
done

# 3. Disable Bluetooth Coexistence Throttling if unused for flight
if [ -d /sys/class/rfkill ]; then
    # Keep WiFi on, power down bluetooth to avoid 2.4GHz interference
    rfkill block bluetooth 2>/dev/null || true
fi

echo "[DRONE-TUNING] System performance optimization complete."
