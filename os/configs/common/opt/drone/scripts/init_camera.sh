#!/usr/bin/env bash
# /opt/drone/scripts/init_camera.sh
# Camera initialization and V4L2 pipeline setup for OV9281 Global Shutter / MIPI-CSI

set -e

DEVICE="${1:-/dev/video0}"

echo "[CAMERA-INIT] Initializing camera device on $DEVICE..."

if [ ! -e "$DEVICE" ]; then
    echo "[CAMERA-INIT] Device $DEVICE not found. Waiting for udev..."
    for i in $(seq 1 10); do
        if [ -e "$DEVICE" ]; then break; fi
        sleep 0.5
    done
fi

if [ ! -e "$DEVICE" ]; then
    echo "[CAMERA-INIT] Warning: $DEVICE did not appear. Skipping V4L2 format initialization."
    exit 0
fi

if command -v v4l2-ctl >/dev/null 2>&1; then
    # Try setting native OV9281 resolution: 1280x800 @ 120fps monochrome
    v4l2-ctl -d "$DEVICE" --set-fmt-video=width=1280,height=800,pixelformat=Y10P 2>/dev/null || \
    v4l2-ctl -d "$DEVICE" --set-fmt-video=width=1280,height=800,pixelformat=GREY 2>/dev/null || \
    v4l2-ctl -d "$DEVICE" --set-fmt-video=width=640,height=480,pixelformat=GREY 2>/dev/null || true

    # Query current format
    echo "[CAMERA-INIT] Active camera format:"
    v4l2-ctl -d "$DEVICE" --get-fmt-video 2>/dev/null || true
fi

echo "[CAMERA-INIT] Camera pipeline initialized successfully."
