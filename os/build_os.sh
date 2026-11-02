#!/usr/bin/env bash
# os/build_os.sh
# Unified Operating System Builder for Autonomous Drone SBCs (Radxa Zero 3W / Raspberry Pi)
#
# Usage:
#   ./os/build_os.sh [options] [radxa|pi|all]
#
# Options:
#   --target <radxa|pi>      Target hardware platform (default: radxa)
#   --distro <bookworm>      Debian distribution release (default: bookworm)
#   --output-dir <path>      Output directory for images (default: os/output)
#   --image-size <MB>        Raw disk image size in MB (default: 2560)
#   --no-compress            Skip xz compression (faster local development)
#   --skip-bootstrap         Reuse existing rootfs (skips debootstrap)
#   --skip-image             Only assemble rootfs without creating raw .img
#   --test-only              Run configuration validation tests and exit
#   --docker                 Run build inside an isolated Docker container
#   --help, -h               Show this help message

set -e
export PATH="/usr/sbin:/sbin:$PATH"

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_DIR="$(cd "$SCRIPT_DIR/.." && pwd)"

TARGET="radxa"
DISTRO="bookworm"
PROFILE="bringup"
OUTPUT_DIR="$SCRIPT_DIR/output"
IMAGE_SIZE_MB="2560"
COMPRESS="true"
SKIP_BOOTSTRAP="false"
SKIP_IMAGE="false"
USE_DOCKER="false"

# ANSI Color codes
GREEN='\033[0;32m'
BLUE='\033[0;34m'
YELLOW='\033[1;33m'
RED='\033[0;31m'
NC='\033[0m'

print_usage() {
    cat <<EOF
Usage: $0 [options] [radxa|pi|all]

Target Platforms:
  radxa, radxa-zero-3w    Radxa Zero 3W (Rockchip RK3566, 64-bit aarch64) [default]
  pi, pi-zero-2w          Raspberry Pi Zero 2W (Broadcom BCM2710A1, arm64/armhf)
  all                     Build operating system images for all supported targets

Options:
  --distro <name>         Debian distribution (bookworm, trixie; default: bookworm)
  --mode <bringup|flight> Image profile (default: bringup for hardware testing)
  --bringup               Configure for hardware bringup (MAVLink proxy only)
  --flight                Configure for autonomous flight (autopilot & vision enabled)
  --output-dir <path>     Directory to store finished images (default: os/output)
  --image-size <MB>       Disk image size in megabytes (default: 2560)
  --no-compress           Do not compress .img with xz (saves time on local test builds)
  --skip-bootstrap        Reuse existing rootfs tree (skips debootstrap download)
  --skip-image            Build and configure rootfs without packaging .img
  --test-only             Run configuration validation suite and exit
  --docker                Execute build inside isolated Docker build container
  -h, --help              Display this help menu
EOF
}

# Parse options
while [ "$#" -gt 0 ]; do
    case "$1" in
        radxa|radxa-zero-3w)
            TARGET="radxa"
            shift
            ;;
        pi|pi-zero-2w)
            TARGET="pi"
            shift
            ;;
        all)
            TARGET="all"
            shift
            ;;
        --target)
            TARGET="$2"
            shift 2
            ;;
        --distro)
            DISTRO="$2"
            shift 2
            ;;
        --mode)
            PROFILE="$2"
            shift 2
            ;;
        --bringup)
            PROFILE="bringup"
            shift
            ;;
        --flight)
            PROFILE="flight"
            shift
            ;;
        --output-dir)
            OUTPUT_DIR="$2"
            shift 2
            ;;
        --image-size)
            IMAGE_SIZE_MB="$2"
            shift 2
            ;;
        --no-compress)
            COMPRESS="false"
            shift
            ;;
        --skip-bootstrap)
            SKIP_BOOTSTRAP="true"
            shift
            ;;
        --skip-image)
            SKIP_IMAGE="true"
            shift
            ;;
        --test-only)
            "$SCRIPT_DIR/tests/test_os_configs.sh"
            exit 0
            ;;
        --docker)
            USE_DOCKER="true"
            shift
            ;;
        -h|--help)
            print_usage
            exit 0
            ;;
        *)
            echo -e "${RED}[ERROR] Unknown option: $1${NC}"
            print_usage
            exit 1
            ;;
    esac
done

if [ "$USE_DOCKER" = "true" ]; then
    echo -e "${BLUE}=== Launching Docker Containerized OS Builder ===${NC}"
    docker build -t drone-os-builder "$SCRIPT_DIR"
    docker run --rm --privileged \
               -v "$REPO_DIR:/workspace" \
               drone-os-builder \
               /workspace/os/build_os.sh --target "$TARGET" --distro "$DISTRO" --mode "$PROFILE" \
                                        $([ "$COMPRESS" = "false" ] && echo "--no-compress") \
                                        $([ "$SKIP_IMAGE" = "true" ] && echo "--skip-image")
    exit 0
fi

# Always run configuration tests first to ensure recipes and systemd units are valid
echo -e "${BLUE}=== Validating Drone OS Configuration Recipes ===${NC}"
"$SCRIPT_DIR/tests/test_os_configs.sh"

build_target() {
    local tgt="$1"
    local rootfs_dir="/tmp/drone-rootfs-${tgt}"
    local output_img="$OUTPUT_DIR/drone-os-${tgt}.img"

    echo ""
    echo -e "${GREEN}================================================================${NC}"
    echo -e "${GREEN} Building Drone OS for Target: ${tgt} (${DISTRO}, Profile: ${PROFILE})${NC}"
    echo -e "${GREEN} Output: ${output_img}${NC}"
    echo -e "${GREEN}================================================================${NC}"

    mkdir -p "$OUTPUT_DIR"

    # Stage 0: Bootstrap base Debian rootfs
    if [ "$SKIP_BOOTSTRAP" = "true" ] && [ -d "$rootfs_dir" ]; then
        echo -e "${YELLOW}[STAGE 0] Skipping bootstrap, reusing existing rootfs at $rootfs_dir${NC}"
    else
        echo -e "${BLUE}[STAGE 0] Bootstrapping rootfs...${NC}"
        "$SCRIPT_DIR/scripts/00_prepare_rootfs.sh" "$tgt" "$rootfs_dir" "$DISTRO"
    fi

    # Stage 1: Configure system, networking, users, udev, systemd services
    echo -e "${BLUE}[STAGE 1] Applying configurations, WiFi AP, and services (Profile: ${PROFILE})...${NC}"
    "$SCRIPT_DIR/scripts/10_configure_system.sh" "$tgt" "$rootfs_dir" "$PROFILE"

    # Stage 2: Cross-compile and deploy drone flight binaries
    echo -e "${BLUE}[STAGE 2] Building and deploying drone binaries (Profile: ${PROFILE})...${NC}"
    "$SCRIPT_DIR/scripts/20_deploy_binaries.sh" "$tgt" "$rootfs_dir" "$PROFILE"

    # Stage 3: Package into flashable partitioned disk image
    if [ "$SKIP_IMAGE" = "true" ]; then
        echo -e "${YELLOW}[STAGE 3] Skipped image generation (--skip-image requested). Rootfs ready at $rootfs_dir${NC}"
    else
        echo -e "${BLUE}[STAGE 3] Assembling raw disk image...${NC}"
        "$SCRIPT_DIR/scripts/30_create_image.sh" "$tgt" "$rootfs_dir" "$output_img" "$IMAGE_SIZE_MB" "$COMPRESS"
    fi

    echo -e "${GREEN}================================================================${NC}"
    echo -e "${GREEN} Successfully built Drone OS for ${tgt}!${NC}"
    if [ "$SKIP_IMAGE" != "true" ]; then
        echo -e "${GREEN} Artifact: ${output_img}${NC}"
        if [ "$COMPRESS" = "true" ]; then
            echo -e "${GREEN} Compressed: ${output_img}.xz${NC}"
        fi
    fi
    echo -e "${GREEN}================================================================${NC}"
}

case "$TARGET" in
    radxa|radxa-zero-3w)
        build_target "radxa-zero-3w"
        ;;
    pi|pi-zero-2w)
        build_target "pi-zero-2w"
        ;;
    all)
        build_target "radxa-zero-3w"
        build_target "pi-zero-2w"
        ;;
    *)
        echo -e "${RED}[ERROR] Invalid target: $TARGET${NC}"
        exit 1
        ;;
esac
