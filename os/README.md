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
- **SSID**: `HEXAGON-flight-drone`
- **Security**: WPA2-PSK (`rylaniscool`)
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
- **`drone-mavlink-router.service`**: Multiplexes FCU serial telemetry (`/dev/flightcontroller` @ 921,600 baud) between the onboard Autopilot daemon (`127.0.0.1:14550`/`14551`) and Wi-Fi GCS broadcast (`192.168.4.255:14550`).
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
│   │   │   ├── mavlink-router/
│   │   │   │   └── main.conf
│   │   │   ├── systemd/system/
│   │   │   │   ├── drone-autopilot.service
│   │   │   │   ├── drone-mavlink-router.service
│   │   │   │   ├── drone-vision.service
│   │   │   │   └── drone-tuning.service
│   │   │   ├── NetworkManager/system-connections/
│   │   │   │   └── drone-ap.nmconnection
│   │   │   └── avahi/services/drone.service
│   │   └── opt/drone/scripts/
│   │       ├── init_camera.sh
│   │       ├── apply_tuning.sh
│   │       ├── set_profile.sh
│   │       ├── start_mavlink_proxy.sh
│   │       └── mavlink_proxy.py
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
    └── test_os_configs.sh         # Configuration test suite (72 assertions)
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

# 2. Build Hardware Bringup image for Raspberry Pi Zero 2W (MAVLink proxy only)
sudo ./os/build_os.sh pi --bringup

# 3. Build Hardware Bringup image for Radxa Zero 3W (MAVLink proxy only)
sudo ./os/build_os.sh radxa --bringup

# 4. Fast development build (skip xz compression)
sudo ./os/build_os.sh pi --bringup --no-compress

# 5. Build full Autonomous Flight image (autopilot & vision enabled)
sudo ./os/build_os.sh pi --flight

# 6. Build inside Docker (no host dependencies required)
./os/build_os.sh --docker pi --bringup
```

Finished images are generated in `os/output/`:
- `drone-os-pi-zero-2w.img` (and compressed `drone-os-pi-zero-2w.img.xz`)
- `drone-os-pi-zero-2w.img.sha256` (and `drone-os-pi-zero-2w.img.xz.sha256`)
- `drone-os-radxa-zero-3w.img` (and compressed `drone-os-radxa-zero-3w.img.xz`)

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

---

## 7. Early Development: Hardware Bringup Mode vs. Autonomous Flight Mode

For early hardware integration, the companion computer boots into **Hardware Bringup Mode**, running **only the MAVLink proxy and Wi-Fi AP**. This allows avionics, chassis, and propulsion teams to bench-test the aircraft without interference from autonomous flight software. Once hardware is validated, the system easily pivots to **Autonomous Flight Mode** where the Autopilot daemon takes ownership of the MAVLink flight loop.

| Feature | Hardware Bringup Mode (`--mode bringup`) | Autonomous Flight Mode (`--mode flight`) |
| :--- | :--- | :--- |
| **Primary Use** | Sensor calibration, ESC/motor testing, PID tuning | Autonomous gate navigation & competition flights |
| **Wi-Fi AP (`HEXAGON-flight-drone`)** | **Enabled** (`192.168.4.1`) | **Enabled** (`192.168.4.1`) |
| **MAVLink Proxy (`drone-mavlink-router`)** | **Enabled** (Exclusive FCU control for GCS) | **Enabled / Internalized** |
| **Autopilot Daemon (`drone-autopilot`)** | **Disabled** (Rotor safety & no interference) | **Enabled** (SCHED_RR Realtime priority) |
| **Vision Daemon (`drone-vision`)** | **Disabled** | **Enabled** (120 FPS camera tracking) |
| **QGroundControl / Mission Planner** | Direct bidirectional bench control | Monitor telemetry & send high-level missions |

---

### 7.1. Building a Hardware Bringup OS Image

The build system defaults to `--bringup` mode:

```bash
# Build for Raspberry Pi Zero 2W (Bringup Mode)
sudo ./os/build_os.sh pi --bringup

# Build for Radxa Zero 3W (Bringup Mode)
sudo ./os/build_os.sh radxa --bringup

# Fast local development build (skips xz compression)
sudo ./os/build_os.sh pi --bringup --no-compress
```

Flash the generated image (`os/output/drone-os-pi-zero-2w.img.xz` or `drone-os-radxa-zero-3w.img.xz`) to a MicroSD card:
```bash
# Using dd:
xz -dc os/output/drone-os-pi-zero-2w.img.xz | sudo dd of=/dev/sdX bs=4M status=progress conv=fsync

# Or using graphical tools:
# Select the .img or .img.xz directly in Raspberry Pi Imager or Balena Etcher.
```

---

### 7.2. Connecting ArduPilot Mission Planner & QGroundControl

Once the SBC boots with the Bringup image:

1. Connect your laptop to the Wi-Fi AP:
   - **SSID**: `HEXAGON-flight-drone`
   - **Password**: `rylaniscool`
   - *(Your laptop receives an IP like `192.168.4.10` via auto-DHCP; the drone is `192.168.4.1`)*.

#### Connecting with ArduPilot Mission Planner (Native UDP)
Mission Planner connects directly to the MAVLink telemetry broadcast via native UDP:
1. In the top-right corner of Mission Planner, click the connection dropdown (where COM ports appear) and select **`UDP`**.
2. Click the green **Connect** button.
3. In the port prompt, enter **`14550`** and click **OK**.
4. Mission Planner detects the heartbeat from `192.168.4.1`, downloads all ArduPilot parameters, and displays the HUD.

#### Connecting with QGroundControl
1. Open QGroundControl.
2. Under **Application Settings $\rightarrow$ Comm Links**:
   - Add a new **UDP** connection on Port **`14550`**.
   - QGroundControl will auto-detect the vehicle heartbeat and connect.

---

### 7.3. What the Hardware Team Can Test

With Bringup Mode active, the Autopilot daemon is dormant and the hardware team has exclusive, unrestricted access to the FCU:
- **Motor Spin & ESC Calibration**: Test motor order (1-4), verify rotation direction, and calibrate ESC throttle endpoints without autonomous interference.
- **Sensor Calibrations**: Complete 6-position Accelerometer, spherical Compass/Magnetometer, and Gyro calibrations.
- **Rangefinder & Optical Flow Validation**: Read real-time distance from the VL53L0X ToF rangefinder and surface tracking quality from the MTF-02P optical flow sensor.
- **Parameter Inspection & Flashing**: Read, modify, and burn ArduPilot parameters directly to the FCU's non-volatile EEPROM.

---

### 7.4. Switching Profiles On-Device (`set_profile.sh`)

Companion images include `/opt/drone/scripts/set_profile.sh` for instant profile switching over SSH (`ssh drone@192.168.4.1`):

```bash
# Check active profile and services:
sudo /opt/drone/scripts/set_profile.sh status

# Re-engage Bringup Mode (disables autopilot/vision, ensures proxy is active):
sudo /opt/drone/scripts/set_profile.sh bringup

# PIVOT: Activate Autonomous Flight Mode (autopilot & vision enabled):
sudo /opt/drone/scripts/set_profile.sh flight
```

---

### 7.5. In-Place Setup on Existing Linux SD Cards (No Re-Flashing)

If you already have a running Linux installation on the board:

1. **Activate Wi-Fi AP Mode (`HEXAGON-flight-drone`)**:
   ```bash
   sudo nmcli dev wifi hotspot ifname wlan0 ssid "HEXAGON-flight-drone" password "rylaniscool"
   sudo nmcli con mod Hotspot connection.id "HEXAGON-flight-drone" ipv4.addresses 192.168.4.1/24 ipv4.method shared
   sudo nmcli con up "HEXAGON-flight-drone"
   ```

2. **Add Flight Controller Serial Symlink**:
   ```bash
   # Raspberry Pi Zero 2W (UART0):
   echo 'KERNEL=="ttyAMA0", SYMLINK+="flightcontroller", OWNER="root", GROUP="dialout", MODE="0666"' | sudo tee /etc/udev/rules.d/99-drone-hardware.rules
   # Radxa Zero 3W (UART4):
   # echo 'KERNEL=="ttyS4", SYMLINK+="flightcontroller", OWNER="root", GROUP="dialout", MODE="0666"' | sudo tee /etc/udev/rules.d/99-drone-hardware.rules
   sudo udevadm control --reload-rules && sudo udevadm trigger
   ```

3. **Run MAVLink Proxy**:
   ```bash
   python3 os/configs/common/opt/drone/scripts/mavlink_proxy.py \
       --serial /dev/flightcontroller \
       --baud 921600 \
       --gcs-broadcast-ip 192.168.4.255 \
       --gcs-port 14550
   ```

