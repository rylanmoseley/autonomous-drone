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
mkdir -p /etc/NetworkManager/system-connections
cat > /etc/NetworkManager/system-connections/drone-ap.nmconnection <<EOF
[connection]
id=$SSID
uuid=a1b2c3d4-e5f6-47a8-9b0c-droneap00001
type=wifi
interface-name=wlan0
autoconnect=true
autoconnect-priority=100
permissions=

[wifi]
mode=ap
ssid=$SSID
band=bg
channel=6

[wifi-security]
key-mgmt=wpa-psk
psk=$PSK
psk-flags=0

[ipv4]
method=shared
address1=192.168.4.1/24

[ipv6]
method=ignore
EOF

chmod 600 /etc/NetworkManager/system-connections/drone-ap.nmconnection
systemctl reload NetworkManager || systemctl restart NetworkManager
sleep 2
nmcli connection up "$SSID" 2>/dev/null || true

# 4. Install MAVLink Proxy scripts
echo "[4/5] Installing MAVLink bringup proxy..."
mkdir -p /opt/drone/scripts /etc/mavlink-router
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_DIR="$(cd "$SCRIPT_DIR/../.." && pwd)"

if [ -f "$REPO_DIR/os/configs/common/opt/drone/scripts/mavlink_proxy.py" ]; then
    cp "$REPO_DIR/os/configs/common/opt/drone/scripts/mavlink_proxy.py" /opt/drone/scripts/
    cp "$REPO_DIR/os/configs/common/opt/drone/scripts/start_mavlink_proxy.sh" /opt/drone/scripts/
    cp "$REPO_DIR/os/configs/common/opt/drone/scripts/set_profile.sh" /opt/drone/scripts/
    cp "$REPO_DIR/os/configs/common/etc/mavlink-router/main.conf" /etc/mavlink-router/
    cp "$REPO_DIR/os/configs/common/etc/systemd/system/drone-mavlink-router.service" /etc/systemd/system/
fi
chmod +x /opt/drone/scripts/*.sh /opt/drone/scripts/*.py 2>/dev/null || true

# 5. Enable MAVLink Proxy Service
echo "[5/5] Activating drone-mavlink-router.service..."
systemctl daemon-reload
systemctl enable --now drone-mavlink-router.service 2>/dev/null || true

echo "=== Bringup setup complete! ==="
echo "Wi-Fi AP:    $SSID"
echo "Password:    $PSK"
echo "Drone IP:    192.168.4.1"
echo "MAVLink UDP: 192.168.4.255:14550"
