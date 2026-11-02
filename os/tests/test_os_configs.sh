#!/usr/bin/env bash
# os/tests/test_os_configs.sh
# Automated validation suite for drone operating system configuration files

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
OS_DIR="$(cd "$SCRIPT_DIR/.." && pwd)"

FAILED=0
PASSED=0

assert_success() {
    local test_name="$1"
    shift
    if "$@"; then
        echo "  [PASS] $test_name"
        PASSED=$((PASSED + 1))
    else
        echo "  [FAIL] $test_name"
        FAILED=$((FAILED + 1))
    fi
}

echo "=== Testing Drone OS Configurations & Recipes ==="

# 1. Shell Script Syntax Checks
echo "1. Validating shell scripts (bash -n)..."
for sh_file in "$OS_DIR"/build_os.sh \
               "$OS_DIR"/configs/common/opt/drone/scripts/*.sh \
               "$OS_DIR"/scripts/*.sh; do
    if [ -f "$sh_file" ]; then
        assert_success "Syntax check: $(basename "$sh_file")" bash -n "$sh_file"
    fi
done

# 2. Systemd Unit Syntax Validation
echo "2. Validating systemd unit files..."
for unit in "$OS_DIR"/configs/common/etc/systemd/system/*.service; do
    assert_success "Unit file exists and non-empty: $(basename "$unit")" test -s "$unit"
    assert_success "Unit contains [Unit] section: $(basename "$unit")" grep -q "^\[Unit\]" "$unit"
    assert_success "Unit contains [Service] section: $(basename "$unit")" grep -q "^\[Service\]" "$unit"
    assert_success "Unit contains [Install] section: $(basename "$unit")" grep -q "^\[Install\]" "$unit"
    assert_success "Unit defines ExecStart: $(basename "$unit")" grep -q "^ExecStart=" "$unit"
done

# 3. NetworkManager Connection Configuration Validation
echo "3. Validating NetworkManager AP configuration..."
NM_FILE="$OS_DIR/configs/common/etc/NetworkManager/system-connections/drone-ap.nmconnection"
assert_success "NetworkManager config exists" test -f "$NM_FILE"
assert_success "SSID defined as Autonomous-Drone-AP" grep -q "^ssid=Autonomous-Drone-AP" "$NM_FILE"
assert_success "Mode configured as AP" grep -q "^mode=ap" "$NM_FILE"
assert_success "WPA-PSK security enabled" grep -q "^key-mgmt=wpa-psk" "$NM_FILE"
assert_success "Static IP 192.168.4.1/24 configured" grep -q "^address1=192.168.4.1/24" "$NM_FILE"
assert_success "Shared IPv4 method for auto-DHCP" grep -q "^method=shared" "$NM_FILE"

# 4. Udev Hardware Rules Validation
echo "4. Validating udev rules..."
UDEV_FILE="$OS_DIR/configs/common/etc/udev/rules.d/99-drone-hardware.rules"
assert_success "Udev rules exist" test -f "$UDEV_FILE"
assert_success "Flightcontroller symlink for UART4 (Radxa)" grep -q 'KERNEL=="ttyS4".*SYMLINK+="flightcontroller"' "$UDEV_FILE"
assert_success "Flightcontroller symlink for UART0 (Pi)" grep -q 'KERNEL=="ttyAMA0".*SYMLINK+="flightcontroller"' "$UDEV_FILE"
assert_success "Camera video devices permissions rule" grep -q 'KERNEL=="video\[0-9\]\*"' "$UDEV_FILE"
assert_success "I2C sensor permissions rule" grep -q 'KERNEL=="i2c-\[0-9\]\*"' "$UDEV_FILE"

# 5. Realtime Sysctl Tuning Validation
echo "5. Validating sysctl tuning..."
SYSCTL_FILE="$OS_DIR/configs/common/etc/sysctl.d/99-drone-tuning.conf"
assert_success "Sysctl config exists" test -f "$SYSCTL_FILE"
assert_success "Realtime runtime allocated" grep -q "^kernel.sched_rt_runtime_us = 950000" "$SYSCTL_FILE"
assert_success "Swappiness reduced" grep -q "^vm.swappiness = 10" "$SYSCTL_FILE"
assert_success "Socket buffer size enlarged" grep -q "^net.core.rmem_max = 4194304" "$SYSCTL_FILE"
assert_success "IP forwarding enabled" grep -q "^net.ipv4.ip_forward = 1" "$SYSCTL_FILE"

# 6. Avahi mDNS Service Validation
echo "6. Validating Avahi mDNS service definition..."
AVAHI_FILE="$OS_DIR/configs/common/etc/avahi/services/drone.service"
assert_success "Avahi service definition exists" test -f "$AVAHI_FILE"
assert_success "Web UI port 8080 registered" grep -q "<port>8080</port>" "$AVAHI_FILE"
assert_success "MAVLink UDP port 14550 registered" grep -q "<port>14550</port>" "$AVAHI_FILE"

# 7. Board-Specific Boot Configurations
echo "7. Validating board bootloader configurations..."
UENV_FILE="$OS_DIR/configs/radxa-zero-3w/boot/uEnv.txt"
assert_success "Radxa uEnv.txt exists" test -f "$UENV_FILE"
assert_success "Radxa overlays include uart4-m1" grep -q "radxa-zero3-uart4-m1" "$UENV_FILE"
assert_success "Radxa overlays include ov9281" grep -q "radxa-zero3-ov9281" "$UENV_FILE"

PI_CONFIG="$OS_DIR/configs/pi-zero-2w/boot/config.txt"
assert_success "Pi config.txt exists" test -f "$PI_CONFIG"
assert_success "Pi overlay includes ov9281" grep -q "dtoverlay=ov9281" "$PI_CONFIG"
assert_success "Pi enables hardware UART" grep -q "enable_uart=1" "$PI_CONFIG"

PI_CMDLINE="$OS_DIR/configs/pi-zero-2w/boot/cmdline.txt"
assert_success "Pi cmdline.txt exists" test -f "$PI_CMDLINE"
assert_success "Pi cmdline enables USB gadget ethernet" grep -q "modules-load=dwc2,g_ether" "$PI_CMDLINE"

# 8. Package Lists Validation
echo "8. Validating package lists..."
for pkg_list in "$OS_DIR"/configs/*/packages.list; do
    assert_success "Package list non-empty: $(basename "$(dirname "$pkg_list")")/$(basename "$pkg_list")" test -s "$pkg_list"
    assert_success "Contains network-manager: $(basename "$(dirname "$pkg_list")")" grep -q "^network-manager" "$pkg_list"
    assert_success "Contains v4l-utils: $(basename "$(dirname "$pkg_list")")" grep -q "^v4l-utils" "$pkg_list"
done

echo "============================================="
echo "Total Tests: $((PASSED + FAILED)) | Passed: $PASSED | Failed: $FAILED"
if [ "$FAILED" -gt 0 ]; then
    echo "OS Configuration Validation FAILED!"
    exit 1
fi
echo "OS Configuration Validation PASSED!"
