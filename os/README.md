# Autonomous Drone Operating System Subproject

This subproject provides the build infrastructure, board support packages, driver configurations, and system recipes to produce bootable, flight-ready Linux operating system images for the autonomous drone companion computers.

---

## 1. Supported Platforms & Hardware Architecture

| Platform | SoC / CPU | Architecture | Primary UART | MIPI-CSI Camera | Onboard WiFi |
| :--- | :--- | :--- | :--- | :--- | :--- |
| **Radxa Zero 3W** *(Primary)* | Rockchip RK3566 (Quad Cortex-A55 @ 1.6 GHz) | `aarch64` (ARM64) | Header Pins 16/18 (`/dev/ttyS4` via `radxa-zero3-uart4-m1.dtbo`) | 4-Lane MIPI CSI (Arducam OV9281 Global Shutter) | AW-CM256SM / AIC8800 |
| **Raspberry Pi Zero 2W** | Broadcom BCM2710A1 (Quad Cortex-A53 @ 1.0 GHz) | `aarch64` / `armhf` | Header Pins 8/10 (`/dev/ttyAMA0` via `miniuart-bt`) | 2-Lane MIPI CSI (`dtoverlay=ov9281`) | BCM43436 (Broadcom) |

---

## 2. Embedded Avionics & Drone Features

### 2.1. Global Shutter Camera Support (Arducam OV9281)
- **Sensor**: 1MP OmniVision OV9281 Monochrome Global Shutter (1280x800 @ 120 FPS).
- Zero rolling-shutter "jello" effect, maximizing visual pose estimation accuracy for gate tracking.
- Preconfigured V4L2 and Media Controller initialization in `/opt/drone/scripts/init_camera.sh`.
- Shipped with `v4l-utils`, `media-ctl`, and OpenCV 4 runtime libraries.

### 2.2. Autonomous WiFi Access Point (AP Mode)
- **SSID**: `Autonomous-Drone-AP`
- **Security**: WPA2-PSK (`DronePilot123`)
- **Static IP**: `192.168.4.1/24` on interface `wlan0`.
- **Automatic DHCP**: Managed by NetworkManager (`method=shared`), dynamically leasing `192.168.4.10` - `192.168.4.50` to connected ground control stations or laptops without needing manual configuration.
- **mDNS Service Discovery**: Avahi daemon advertises `drone.local` on ports:
  - `8080`: Ground Control Web UI
  - `14550`: MAVLink UDP Telemetry
  - `22`: SSH Terminal

### 2.3. Flight Controller Serial Telemetry
- Persistent symlink `/dev/flightcontroller` auto-created by udev rules ([`99-drone-hardware.rules`](configs/common/etc/udev/rules.d/99-drone-hardware.rules)):
  - Points to `/dev/ttyS4` on Radxa Zero 3W.
  - Points to `/dev/ttyAMA0` on Raspberry Pi.
  - Automatically claims USB virtual COM ports (`/dev/ttyACM*`).
- Granted `dialout` group permissions (`0666`) for non-root flight daemon execution.

### 2.4. Realtime Performance & System Tuning
- **CPU Governor**: Locked to `performance` mode at boot ([`apply_tuning.sh`](configs/common/opt/drone/scripts/apply_tuning.sh)) to prevent dynamic frequency scaling latency spikes during control loops.
- **Kernel RT Sched Tuning**: `kernel.sched_rt_runtime_us = 950000` dedicated to SCHED_RR realtime flight threads.
- **Socket Buffer Expansion**: `net.core.rmem_max = 4194304` to prevent dropped UDP telemetry packets under heavy load.
- **SD Card Protection**: Log partition mounted in memory (`tmpfs` on `/var/log` and `/tmp`) to protect against filesystem corruption during sudden LiPo disconnects.

### 2.5. Automatic Systemd Daemons
- **`drone-tuning.service`**: Applies hardware clocks and latency tuning at early boot.
- **`drone-autopilot.service`**: Autostarts `/opt/drone/bin/autopilot_node` with realtime scheduling priority.
- **`drone-vision.service`**: Autostarts `/opt/drone/bin/vision_node --headless` and initialises the camera subdevice.

---

## 3. Directory Layout

```
os/
├── README.md                      # This documentation
├── build_os.sh                    # Unified OS builder CLI
├── Dockerfile                     # Isolated Docker build container
├── configs/
│   ├── common/                    # Shared configurations across Radxa & Pi
│   │   ├── etc/
│   │   │   ├── sysctl.d/99-drone-tuning.conf
│   │   │   ├── udev/rules.d/99-drone-hardware.rules
│   │   │   ├── systemd/system/
│   │   │   │   ├── drone-autopilot.service
│   │   │   │   ├── drone-vision.service
│   │   │   │   └── drone-tuning.service
│   │   │   ├── NetworkManager/system-connections/
│   │   │   │   └── drone-ap.nmconnection
│   │   │   └── avahi/services/drone.service
│   │   └── opt/drone/scripts/
│   │       ├── init_camera.sh
│   │       └── apply_tuning.sh
│   ├── radxa-zero-3w/             # Radxa-specific boot & packages
│   │   ├── boot/uEnv.txt
│   │   └── packages.list
│   └── pi-zero-2w/                # Raspberry Pi-specific boot & packages
│       ├── boot/config.txt
│       ├── boot/cmdline.txt
│       └── packages.list
├── scripts/
│   ├── 00_prepare_rootfs.sh       # Debootstrap base Debian rootfs
│   ├── 10_configure_system.sh     # Network, services, users, udev recipes
│   ├── 20_deploy_binaries.sh      # Cross-compiles & deploys autopilot/vision binaries
│   └── 30_create_image.sh         # Partitions, formats (FAT32+ext4), and compresses .img.xz
└── tests/
    └── test_os_configs.sh         # Configuration test suite (55 assertions)
```

---

## 4. Building Locally

### Prerequisites
On Debian/Ubuntu hosts:
```bash
sudo apt-get update
sudo apt-get install -y debootstrap parted dosfstools e2fsprogs xz-utils qemu-user-binfmt qemu-user
```

### Quick Build Commands
```bash
# 1. Validate configuration files and recipes (fast test)
./os/build_os.sh --test-only

# 2. Build complete flashable image for Radxa Zero 3W
sudo ./os/build_os.sh radxa

# 3. Build for Raspberry Pi Zero 2W
sudo ./os/build_os.sh pi

# 4. Fast development build (skip compression)
sudo ./os/build_os.sh radxa --no-compress

# 5. Build inside Docker (no host dependencies required)
./os/build_os.sh --docker radxa
```

Finished images are generated in `os/output/`:
- `drone-os-radxa-zero-3w.img` (and compressed `drone-os-radxa-zero-3w.img.xz`)
- `drone-os-radxa-zero-3w.img.sha256`

---

## 5. Manual-Run CI Job (GitHub Actions)

A manual trigger workflow (`workflow_dispatch`) is provided in [`.github/workflows/build-os.yml`](../.github/workflows/build-os.yml).

### Triggering via GitHub CLI (`gh`):
```bash
# Trigger build for Radxa Zero 3W
gh workflow run "Build Drone Operating System" -f target=radxa-zero-3w -f compress=true

# Trigger build for Raspberry Pi Zero 2W
gh workflow run "Build Drone Operating System" -f target=pi-zero-2w -f compress=true
```

### Triggering via GitHub Web UI:
1. Navigate to the **Actions** tab on GitHub.
2. Select **"Build Drone Operating System"** from the left sidebar.
3. Click **"Run workflow"**, select the branch and target platform (`radxa-zero-3w` or `pi-zero-2w`), and click **"Run workflow"**.
4. Once completed, the flashable compressed `.img.xz` file and SHA-256 checksum will be available in the workflow **Artifacts** section.

---

## 6. Flashing to MicroSD Card / eMMC

### Using `dd` on Linux / macOS:
```bash
# Replace /dev/sdX with your target SD card device
xz -dc os/output/drone-os-radxa-zero-3w.img.xz | sudo dd of=/dev/sdX bs=4M status=progress conv=fsync
```

### Using Graphical Tools:
You can flash the `.img` or `.img.xz` directly using:
- **[Raspberry Pi Imager](https://www.raspberrypi.com/software/)** (Choose OS $\rightarrow$ Use custom)
- **[Balena Etcher](https://etcher.balena.io/)**
