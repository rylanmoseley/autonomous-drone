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

# 3. Wireless Regulatory Domain & Radio Management
if command -v iw >/dev/null 2>&1; then
    iw reg set US 2>/dev/null || true
    echo "[DRONE-TUNING] Wireless regulatory domain set to US."
fi

if command -v rfkill >/dev/null 2>&1; then
    rfkill unblock wifi 2>/dev/null || true
    rfkill unblock wlan 2>/dev/null || true
    rfkill unblock all 2>/dev/null || true
    echo "[DRONE-TUNING] Wireless interfaces unblocked via rfkill."
fi

echo "[DRONE-TUNING] System performance optimization complete."
