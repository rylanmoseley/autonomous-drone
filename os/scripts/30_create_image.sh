#!/usr/bin/env bash
# os/scripts/30_create_image.sh
# Partition, format, and assemble flashable disk image (.img and .img.xz)

set -e
export PATH="/usr/sbin:/sbin:$PATH"

TARGET="${1:-radxa}"
ROOTFS_DIR="${2:-/tmp/drone-rootfs}"
OUTPUT_IMG="${3:-/tmp/drone-os.img}"
IMAGE_SIZE_MB="${4:-2560}" # Default 2.5 GB
COMPRESS="${5:-true}"

echo "=== [STAGE 3] Creating Bootable Disk Image: $OUTPUT_IMG ($IMAGE_SIZE_MB MB) ==="

mkdir -p "$(dirname "$OUTPUT_IMG")"
rm -f "$OUTPUT_IMG" "${OUTPUT_IMG}.xz"

# 1. Allocate raw sparse image file
echo "[INFO] Allocating ${IMAGE_SIZE_MB}MB image file..."
truncate -s "${IMAGE_SIZE_MB}M" "$OUTPUT_IMG"

# 2. Partition image with parted
# Sector size: 512 bytes. Leave 16MB offset for Rockchip bootloader (idbloader/u-boot).
echo "[INFO] Partitioning disk image (FAT32 boot + ext4 rootfs)..."
parted -s "$OUTPUT_IMG" mklabel msdos
parted -s "$OUTPUT_IMG" mkpart primary fat32 16MiB 272MiB
parted -s "$OUTPUT_IMG" set 1 boot on
parted -s "$OUTPUT_IMG" mkpart primary ext4 272MiB 100%

# 3. Format and Populate Partitions
BOOT_START_SECTOR=32768   # 16MiB * 1024 * 1024 / 512
BOOT_NUM_SECTORS=524288   # 256MiB
ROOT_START_SECTOR=557056  # 272MiB

BOOT_OFFSET=$((BOOT_START_SECTOR * 512))
ROOT_OFFSET=$((ROOT_START_SECTOR * 512))

# Check for loop device availability
if [ "$(id -u)" -eq 0 ] && command -v losetup >/dev/null 2>&1; then
    echo "[INFO] Using loop devices to format and populate partitions..."
    LOOP_DEV=$(losetup -Pf --show "$OUTPUT_IMG")
    
    # Format FAT32 boot
    mkfs.vfat -F 32 -n "DRONE_BOOT" "${LOOP_DEV}p1"
    
    # Format ext4 rootfs
    mkfs.ext4 -F -L "DRONE_ROOT" "${LOOP_DEV}p2"
    
    # Mount and copy files
    MOUNT_DIR=$(mktemp -d /tmp/drone-mnt-XXXXXX)
    mkdir -p "$MOUNT_DIR/root" "$MOUNT_DIR/boot"
    
    mount "${LOOP_DEV}p2" "$MOUNT_DIR/root"
    mkdir -p "$MOUNT_DIR/root/boot"
    mount "${LOOP_DEV}p1" "$MOUNT_DIR/root/boot"
    
    echo "[INFO] Copying rootfs into partitioned image..."
    cp -a "$ROOTFS_DIR/"* "$MOUNT_DIR/root/" 2>/dev/null || true
    
    sync
    umount "$MOUNT_DIR/root/boot" || true
    umount "$MOUNT_DIR/root" || true
    rmdir "$MOUNT_DIR/boot" "$MOUNT_DIR/root" "$MOUNT_DIR" 2>/dev/null || true
    losetup -d "$LOOP_DEV"
else
    echo "[INFO] Non-root/containerized build: Generating ext4 rootfs image and tarball..."
    # Generate standalone ext4 rootfs partition image
    ROOTFS_IMG="${OUTPUT_IMG}.rootfs.ext4"
    ROOTFS_SIZE_MB=$((IMAGE_SIZE_MB - 272))
    
    echo "[INFO] Building ${ROOTFS_SIZE_MB}MB ext4 root filesystem image..."
    if command -v mke2fs >/dev/null 2>&1; then
        mke2fs -d "$ROOTFS_DIR" -t ext4 -F -L "DRONE_ROOT" "$ROOTFS_IMG" "${ROOTFS_SIZE_MB}M"
        dd if="$ROOTFS_IMG" of="$OUTPUT_IMG" bs=512 seek="$ROOT_START_SECTOR" conv=notrunc status=none
        rm -f "$ROOTFS_IMG"
    fi
fi

# 4. Generate Checksum
echo "[INFO] Calculating SHA256 checksum..."
sha256sum "$OUTPUT_IMG" > "${OUTPUT_IMG}.sha256"

# 5. Compress image if requested
if [ "$COMPRESS" = "true" ] && command -v xz >/dev/null 2>&1; then
    echo "[INFO] Compressing image with xz (this may take a few minutes)..."
    xz -T0 -k -9 "$OUTPUT_IMG"
    sha256sum "${OUTPUT_IMG}.xz" > "${OUTPUT_IMG}.xz.sha256"
    echo "[INFO] Compressed image written to ${OUTPUT_IMG}.xz"
fi

echo "=== [STAGE 3] Disk image build complete: $OUTPUT_IMG ==="
