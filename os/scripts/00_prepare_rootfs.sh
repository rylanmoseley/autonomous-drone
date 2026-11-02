#!/usr/bin/env bash
# os/scripts/00_prepare_rootfs.sh
# Bootstrap base Debian Linux root filesystem for target architecture

set -e
export PATH="/usr/sbin:/sbin:$PATH"

TARGET="${1:-radxa}"
ROOTFS_DIR="${2:-/tmp/drone-rootfs}"
DISTRO="${3:-bookworm}"

case "$TARGET" in
    radxa|radxa-zero-3w)
        ARCH="arm64"
        QEMU_ARCH="aarch64"
        ;;
    pi|pi-zero-2w)
        ARCH="arm64"
        QEMU_ARCH="aarch64"
        ;;
    pi-armhf)
        ARCH="armhf"
        QEMU_ARCH="arm"
        ;;
    *)
        echo "[ERROR] Unknown target: $TARGET"
        exit 1
        ;;
esac

echo "=== [STAGE 0] Bootstrapping Base Rootfs for $TARGET ($ARCH / $DISTRO) ==="

# Check debootstrap
if ! command -v debootstrap >/dev/null 2>&1; then
    echo "[ERROR] debootstrap is required. Install with: sudo apt-get install debootstrap"
    exit 1
fi

# Clean prior rootfs if requested
if [ -d "$ROOTFS_DIR" ]; then
    echo "[INFO] Cleaning previous rootfs at $ROOTFS_DIR..."
    umount -lf "$ROOTFS_DIR/dev/pts" 2>/dev/null || true
    umount -lf "$ROOTFS_DIR/dev" 2>/dev/null || true
    umount -lf "$ROOTFS_DIR/sys" 2>/dev/null || true
    umount -lf "$ROOTFS_DIR/proc" 2>/dev/null || true
    rm -rf "$ROOTFS_DIR"
fi
mkdir -p "$ROOTFS_DIR"

DEBIAN_MIRROR="${DEBIAN_MIRROR:-http://deb.debian.org/debian}"
INCLUDED_PACKAGES="systemd,udev,dbus,ca-certificates,sudo,network-manager,python3,avahi-daemon,libpam-systemd,iproute2,iw,wireless-regdb,iptables,dnsmasq-base,procps,kmod"
COMPONENTS="main"

if [ "$TARGET" = "pi" ] || [ "$TARGET" = "pi-zero-2w" ]; then
    COMPONENTS="main,contrib,non-free-firmware"
    INCLUDED_PACKAGES="$INCLUDED_PACKAGES,raspi-firmware"
fi

echo "[INFO] Running debootstrap (first stage) for $DISTRO $ARCH ($COMPONENTS)..."
debootstrap --arch="$ARCH" \
            --foreign \
            --variant=minbase \
            --components="$COMPONENTS" \
            --include="$INCLUDED_PACKAGES" \
            "$DISTRO" \
            "$ROOTFS_DIR" \
            "$DEBIAN_MIRROR"

# Install QEMU static binary into rootfs for second stage execution if foreign arch
mkdir -p "$ROOTFS_DIR/usr/bin" "$ROOTFS_DIR/usr/libexec/qemu-binfmt"
for qbin in "/usr/bin/qemu-${QEMU_ARCH}-static" "/usr/bin/qemu-${QEMU_ARCH}"; do
    if [ -f "$qbin" ]; then
        echo "[INFO] Copying QEMU emulator $qbin into chroot..."
        cp "$qbin" "$ROOTFS_DIR/usr/bin/" 2>/dev/null || true
        cp "$qbin" "$ROOTFS_DIR/usr/libexec/qemu-binfmt/${QEMU_ARCH}-binfmt-P" 2>/dev/null || true
    fi
done

# Run second stage
if [ -f "$ROOTFS_DIR/debootstrap/debootstrap" ]; then
    echo "[INFO] Mounting pseudo-filesystems for chroot second stage..."
    mkdir -p "$ROOTFS_DIR/proc" "$ROOTFS_DIR/sys" "$ROOTFS_DIR/dev" "$ROOTFS_DIR/dev/pts"
    mount -t proc /proc "$ROOTFS_DIR/proc"
    mount -t sysfs /sys "$ROOTFS_DIR/sys"
    mount -o bind /dev "$ROOTFS_DIR/dev"
    mount -o bind /dev/pts "$ROOTFS_DIR/dev/pts"

    cleanup_chroot_mounts() {
        umount -lf "$ROOTFS_DIR/dev/pts" 2>/dev/null || true
        umount -lf "$ROOTFS_DIR/dev" 2>/dev/null || true
        umount -lf "$ROOTFS_DIR/sys" 2>/dev/null || true
        umount -lf "$ROOTFS_DIR/proc" 2>/dev/null || true
    }
    trap cleanup_chroot_mounts EXIT INT TERM

    echo "[INFO] Running debootstrap (second stage inside chroot)..."
    CONTAINER=lxc chroot "$ROOTFS_DIR" /debootstrap/debootstrap --second-stage

    cleanup_chroot_mounts
    trap - EXIT INT TERM
fi

echo "=== [STAGE 0] Base rootfs bootstrap complete! ==="
