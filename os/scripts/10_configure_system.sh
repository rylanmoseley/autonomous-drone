#!/usr/bin/env bash
# os/scripts/10_configure_system.sh
# Apply configuration recipes, user accounts, and systemd services into rootfs

set -e
export PATH="/usr/sbin:/sbin:$PATH"

TARGET="${1:-radxa}"
ROOTFS_DIR="${2:-/tmp/drone-rootfs}"
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
OS_DIR="$(cd "$SCRIPT_DIR/.." && pwd)"

case "$TARGET" in
    radxa|radxa-zero-3w)
        HOSTNAME="drone-radxa"
        BOARD_DIR="$OS_DIR/configs/radxa-zero-3w"
        BOOT_MOUNT="/boot"
        ;;
    pi|pi-zero-2w|pi-armhf)
        HOSTNAME="drone-pi"
        BOARD_DIR="$OS_DIR/configs/pi-zero-2w"
        BOOT_MOUNT="/boot/firmware"
        ;;
    *)
        echo "[ERROR] Unknown target: $TARGET"
        exit 1
        ;;
esac

echo "=== [STAGE 1] Configuring Drone System for $TARGET ($HOSTNAME) ==="

# 1. Copy Common Configs (sysctl, udev, systemd services, NetworkManager, avahi, scripts)
echo "[INFO] Copying common configurations..."
cp -r "$OS_DIR/configs/common/"* "$ROOTFS_DIR/"

# 2. Copy Board-Specific Boot Files
echo "[INFO] Copying board bootloader files..."
mkdir -p "$ROOTFS_DIR/$BOOT_MOUNT"
if [ -d "$BOARD_DIR/boot" ]; then
    cp -r "$BOARD_DIR/boot/"* "$ROOTFS_DIR/$BOOT_MOUNT/"
fi

# 3. Secure NetworkManager connection file permissions (required by NetworkManager)
chmod 600 "$ROOTFS_DIR/etc/NetworkManager/system-connections/"*.nmconnection 2>/dev/null || true

# 4. Hostname & Hosts File
echo "$HOSTNAME" > "$ROOTFS_DIR/etc/hostname"
cat > "$ROOTFS_DIR/etc/hosts" <<EOF
127.0.0.1   localhost
127.0.1.1   $HOSTNAME drone.local
192.168.4.1 $HOSTNAME drone.local

# The following lines are desirable for IPv6 capable hosts
::1     ip6-localhost ip6-loopback
fe00::0 ip6-localnet
ff00::0 ip6-mcastprefix
ff02::1 ip6-allnodes
ff02::2 ip6-allrouters
EOF

# 5. /etc/fstab configuration
cat > "$ROOTFS_DIR/etc/fstab" <<EOF
# /etc/fstab: static file system information.
# <file system> <mount point>   <type>  <options>       <dump>  <pass>
/dev/mmcblk0p2  /               ext4    noatime,errors=remount-ro 0       1
/dev/mmcblk0p1  $BOOT_MOUNT     vfat    defaults                  0       2
tmpfs           /tmp            tmpfs   defaults,noatime,nosuid   0       0
tmpfs           /var/log        tmpfs   defaults,noatime,nosuid,size=64m 0 0
EOF

# 6. User Account Creation & Permissions
echo "[INFO] Configuring user accounts..."
if [ -x "$ROOTFS_DIR/bin/sh" ] && command -v chroot >/dev/null 2>&1; then
    for grp in dialout video i2c spi input gpio; do
        chroot "$ROOTFS_DIR" /bin/sh -c "getent group $grp >/dev/null || groupadd -r $grp" 2>/dev/null || true
    done

    chroot "$ROOTFS_DIR" /bin/sh -c "id -u drone >/dev/null 2>&1 || useradd -m -s /bin/bash -G sudo,dialout,video,i2c,spi,input drone" 2>/dev/null || true
    echo "drone:drone" | chroot "$ROOTFS_DIR" chpasswd 2>/dev/null || true
    echo "root:drone"  | chroot "$ROOTFS_DIR" chpasswd 2>/dev/null || true
else
    echo "[INFO] Shell not present in rootfs (skipping chroot useradd)."
fi

mkdir -p "$ROOTFS_DIR/etc/sudoers.d"

# Passwordless sudo for drone user
rm -f "$ROOTFS_DIR/etc/sudoers.d/010_drone-nopasswd"
cat > "$ROOTFS_DIR/etc/sudoers.d/010_drone-nopasswd" <<EOF
drone ALL=(ALL) NOPASSWD: ALL
EOF
chmod 0440 "$ROOTFS_DIR/etc/sudoers.d/010_drone-nopasswd"

# 7. Enable Systemd Services inside rootfs
PROFILE="${3:-bringup}"
echo "[INFO] Enabling drone systemd services (Profile: $PROFILE)..."
mkdir -p "$ROOTFS_DIR/etc/systemd/system/multi-user.target.wants" \
         "$ROOTFS_DIR/etc/systemd/system/basic.target.wants"

ln -sf /etc/systemd/system/drone-tuning.service "$ROOTFS_DIR/etc/systemd/system/basic.target.wants/drone-tuning.service"
ln -sf /etc/systemd/system/drone-mavlink-router.service "$ROOTFS_DIR/etc/systemd/system/multi-user.target.wants/drone-mavlink-router.service"

if [ "$PROFILE" = "flight" ]; then
    echo "[INFO] Enabling autonomous flight services (autopilot & vision)..."
    ln -sf /etc/systemd/system/drone-autopilot.service "$ROOTFS_DIR/etc/systemd/system/multi-user.target.wants/drone-autopilot.service"
    ln -sf /etc/systemd/system/drone-vision.service "$ROOTFS_DIR/etc/systemd/system/multi-user.target.wants/drone-vision.service"
else
    echo "[INFO] Profile 'bringup' selected: Hardware Bringup Mode active."
    echo "       MAVLink proxy & Wi-Fi AP enabled. Autopilot & Vision daemons are installed but DISABLED."
    echo "       (Run 'sudo /opt/drone/scripts/set_profile.sh flight' to pivot to autonomous flight)."
fi

ln -sf /lib/systemd/system/NetworkManager.service "$ROOTFS_DIR/etc/systemd/system/multi-user.target.wants/NetworkManager.service" 2>/dev/null || true
ln -sf /lib/systemd/system/wpa_supplicant.service "$ROOTFS_DIR/etc/systemd/system/multi-user.target.wants/wpa_supplicant.service" 2>/dev/null || true
ln -sf /lib/systemd/system/avahi-daemon.service "$ROOTFS_DIR/etc/systemd/system/multi-user.target.wants/avahi-daemon.service" 2>/dev/null || true
ln -sf /lib/systemd/system/ssh.service "$ROOTFS_DIR/etc/systemd/system/multi-user.target.wants/ssh.service" 2>/dev/null || true

# Prevent systemd from ever soft-blocking Wi-Fi across boots
ln -sf /dev/null "$ROOTFS_DIR/etc/systemd/system/systemd-rfkill.service" 2>/dev/null || true
ln -sf /dev/null "$ROOTFS_DIR/etc/systemd/system/systemd-rfkill.socket" 2>/dev/null || true

# Generate SSH host keys ahead of time
if [ -x "$ROOTFS_DIR/usr/bin/ssh-keygen" ]; then
    chroot "$ROOTFS_DIR" ssh-keygen -A 2>/dev/null || true
fi

echo "=== [STAGE 1] Configuration complete! ==="
