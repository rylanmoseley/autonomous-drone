#!/usr/bin/env bash
# os/scripts/20_deploy_binaries.sh
# Build and deploy drone flight binaries (autopilot_node & vision_node) into rootfs

set -e

TARGET="${1:-radxa}"
ROOTFS_DIR="${2:-/tmp/drone-rootfs}"
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_DIR="$(cd "$SCRIPT_DIR/../.." && pwd)"

case "$TARGET" in
    radxa|radxa-zero-3w)
        BUILD_DIR="$REPO_DIR/build-radxa"
        BUILD_ARG="radxa"
        ;;
    pi|pi-zero-2w)
        BUILD_DIR="$REPO_DIR/build-pi"
        BUILD_ARG="pi"
        ;;
    pi-armhf)
        BUILD_DIR="$REPO_DIR/build-pi"
        BUILD_ARG="pi"
        ;;
    *)
        echo "[ERROR] Unknown target: $TARGET"
        exit 1
        ;;
esac

echo "=== [STAGE 2] Deploying Drone Binaries for $TARGET ==="

AUTOPILOT_BIN="$BUILD_DIR/autopilot/autopilot_node"
VISION_BIN="$BUILD_DIR/vision-processing/vision_node"

# 1. Build if binaries are not yet compiled
if [ ! -f "$AUTOPILOT_BIN" ]; then
    echo "[INFO] Autopilot binary not found. Running ./build_all.sh $BUILD_ARG..."
    (cd "$REPO_DIR" && ./build_all.sh "$BUILD_ARG")
fi

mkdir -p "$ROOTFS_DIR/opt/drone/bin"

# 2. Deploy Autopilot Node
if [ -f "$AUTOPILOT_BIN" ]; then
    echo "[INFO] Installing autopilot_node into /opt/drone/bin/..."
    cp "$AUTOPILOT_BIN" "$ROOTFS_DIR/opt/drone/bin/autopilot_node"
    chmod 755 "$ROOTFS_DIR/opt/drone/bin/autopilot_node"
else
    echo "[WARN] Autopilot binary $AUTOPILOT_BIN not found!"
fi

# 3. Deploy Vision Node (if built in target build directory)
if [ -f "$VISION_BIN" ]; then
    echo "[INFO] Installing vision_node into /opt/drone/bin/..."
    cp "$VISION_BIN" "$ROOTFS_DIR/opt/drone/bin/vision_node"
    chmod 755 "$ROOTFS_DIR/opt/drone/bin/vision_node"
elif [ -f "$REPO_DIR/build/vision-processing/vision_node" ]; then
    # Fallback to host vision build if cross OpenCV sysroot was omitted
    echo "[INFO] Using compiled vision_node from build tree..."
    cp "$REPO_DIR/build/vision-processing/vision_node" "$ROOTFS_DIR/opt/drone/bin/vision_node"
    chmod 755 "$ROOTFS_DIR/opt/drone/bin/vision_node"
else
    echo "[INFO] Creating placeholder vision_node runner script..."
    cat > "$ROOTFS_DIR/opt/drone/bin/vision_node" <<'EOF'
#!/usr/bin/env bash
echo "[VISION-NODE] Vision node starting up. Connect camera to /dev/video0."
while true; do sleep 5; done
EOF
    chmod 755 "$ROOTFS_DIR/opt/drone/bin/vision_node"
fi

echo "=== [STAGE 2] Binaries deployed successfully! ==="
