#!/usr/bin/env bash
# os/scripts/setup_bringup_on_pi.sh
# Instant hardware bringup setup for running Raspberry Pi OS Lite (64-bit)
# Configures HEXAGON-flight-drone Wi-Fi AP and standalone MAVLink proxy.

set -e

if [ "$(id -u)" -ne 0 ]; then
    echo "[ERROR] This script must be run as root: sudo $0"
    exit 1
fi

echo "=== Setting up Autonomous Drone Hardware Bringup on Raspberry Pi ==="

SSID="HEXAGON-flight-drone"
PSK="rylaniscool"

# 1. Install required packages if missing
echo "[1/5] Ensuring required networking packages..."
apt-get update -qq
apt-get install -y -qq network-manager wpasupplicant rfkill wireless-regdb dnsmasq-base python3

# 2. Configure Wireless Regulatory Domain & Unblock Wi-Fi
echo "[2/5] Setting regulatory domain to US and unblocking radios..."
iw reg set US 2>/dev/null || true
rfkill unblock wifi 2>/dev/null || true
rfkill unblock wlan 2>/dev/null || true
rfkill unblock all 2>/dev/null || true

# 3. Configure NetworkManager Wi-Fi AP
echo "[3/5] Configuring NetworkManager AP ($SSID)..."
echo "options brcmfmac roamoff=1 feature_disable=0x82000" > /etc/modprobe.d/brcmfmac.conf
nmcli con delete "$SSID" 2>/dev/null || true
nmcli con delete drone-ap 2>/dev/null || true
rm -f /etc/NetworkManager/system-connections/drone-ap.nmconnection
nmcli device wifi hotspot ifname wlan0 con-name "$SSID" ssid "$SSID" password "$PSK"
nmcli con modify "$SSID" ipv4.addresses 192.168.4.1/24
nmcli con modify "$SSID" connection.autoconnect yes
nmcli con modify "$SSID" connection.autoconnect-priority 100
# Disable PMF and force standard WPA2 (prevents client connection handshaking errors on Pi Zero 2W)
nmcli con modify "$SSID" wifi-sec.pmf 0
nmcli con modify "$SSID" wifi-sec.proto rsn
nmcli con modify "$SSID" wifi-sec.pairwise ccmp
nmcli con modify "$SSID" wifi-sec.group ccmp
nmcli con down "$SSID" 2>/dev/null || true
nmcli con up "$SSID" 2>/dev/null || true

# 4. Install MAVLink Proxy scripts
echo "[4/5] Installing MAVLink bringup proxy..."
mkdir -p /opt/drone/scripts /etc/mavlink-router /etc/systemd/system
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" 2>/dev/null && pwd || echo "")"
REPO_DIR="$(cd "$SCRIPT_DIR/../.." 2>/dev/null && pwd || echo "")"
RAW_BASE="https://raw.githubusercontent.com/rylanmoseley/autonomous-drone/feature/vision-global-drift-passthrough"

copy_or_download() {
    local rel_src="$1"
    local dst="$2"
    if [ -n "$REPO_DIR" ] && [ -f "$REPO_DIR/$rel_src" ]; then
        cp "$REPO_DIR/$rel_src" "$dst"
    else
        echo "  Downloading $(basename "$dst")..."
        curl -sSL "$RAW_BASE/$rel_src" -o "$dst"
    fi
}

copy_or_download "os/configs/common/opt/drone/scripts/mavlink_proxy.py" "/opt/drone/scripts/mavlink_proxy.py"
copy_or_download "os/configs/common/opt/drone/scripts/start_mavlink_proxy.sh" "/opt/drone/scripts/start_mavlink_proxy.sh"
copy_or_download "os/configs/common/opt/drone/scripts/set_profile.sh" "/opt/drone/scripts/set_profile.sh"
copy_or_download "os/configs/common/etc/mavlink-router/main.conf" "/etc/mavlink-router/main.conf"
copy_or_download "os/configs/common/etc/systemd/system/drone-mavlink-router.service" "/etc/systemd/system/drone-mavlink-router.service"

chmod +x /opt/drone/scripts/*.sh /opt/drone/scripts/*.py 2>/dev/null || true

# Add udev symlink for flight controller if not present
if [ ! -f /etc/udev/rules.d/99-drone-hardware.rules ]; then
    echo 'KERNEL=="ttyAMA0", SYMLINK+="flightcontroller", MODE="0666"' > /etc/udev/rules.d/99-drone-hardware.rules
    udevadm control --reload-rules 2>/dev/null || true
    udevadm trigger 2>/dev/null || true
fi

# 5. Enable MAVLink Proxy Service
echo "[5/5] Activating drone-mavlink-router.service..."
systemctl daemon-reload
systemctl enable --now drone-mavlink-router.service 2>/dev/null || true

echo "=== Bringup setup complete! ==="
echo "Wi-Fi AP:    $SSID"
echo "Password:    $PSK"
echo "Drone IP:    192.168.4.1"
echo "MAVLink UDP: 192.168.4.255:14550"
