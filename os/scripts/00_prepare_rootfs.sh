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
    rm -rf "$ROOTFS_DIR"
fi
mkdir -p "$ROOTFS_DIR"

DEBIAN_MIRROR="${DEBIAN_MIRROR:-http://deb.debian.org/debian}"

echo "[INFO] Running debootstrap (first stage) for $DISTRO $ARCH..."
debootstrap --arch="$ARCH" \
            --foreign \
            --variant=minbase \
            --include=systemd,udev,dbus,ca-certificates \
            "$DISTRO" \
            "$ROOTFS_DIR" \
            "$DEBIAN_MIRROR"

# Install QEMU static binary into rootfs for second stage execution if foreign arch
QEMU_BIN="/usr/bin/qemu-${QEMU_ARCH}-static"
if [ ! -f "$QEMU_BIN" ]; then
    QEMU_BIN="$(which "qemu-${QEMU_ARCH}-static" 2>/dev/null || which "qemu-${QEMU_ARCH}" 2>/dev/null || true)"
fi

if [ -n "$QEMU_BIN" ] && [ -f "$QEMU_BIN" ]; then
    echo "[INFO] Copying QEMU emulator $QEMU_BIN into chroot..."
    mkdir -p "$ROOTFS_DIR/usr/bin"
    cp "$QEMU_BIN" "$ROOTFS_DIR/usr/bin/"
fi

# Run second stage
if [ -f "$ROOTFS_DIR/debootstrap/debootstrap" ]; then
    echo "[INFO] Running debootstrap (second stage inside chroot)..."
    chroot "$ROOTFS_DIR" /debootstrap/debootstrap --second-stage
fi

echo "=== [STAGE 0] Base rootfs bootstrap complete! ==="
