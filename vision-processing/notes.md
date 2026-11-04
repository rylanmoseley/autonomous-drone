# Vision Processing Subsystem

This directory contains the computer vision modules for autonomous drone gate tracking, shape identification, 6-DoF relative pose estimation, and UDP telemetry broadcast to the autopilot.

## Subsystem Architecture & Files

- **`include/gate_detector.h` & `src/gate_detector.cpp`**: Modular core vision library (`gate_detector`).
  - Hierarchical contour search (`cv::RETR_TREE`) isolating inner gate opening (child) from outer physical gate structure (parent).
  - Shape classification (`Circle`, `Square`, `Rectangle`, `Pentagon`, `Triangle`).
  - Quad and pentagon corner ordering algorithms.
  - 6-DoF pose estimation via `cv::solvePnP` with 3D model vertices in **SI meters**.
  - Packaging helper for `VisionGoalEstimate` in `shared/include/vision_packet.h`.
- **`AdvancedVisionTesting.cpp` (built as `vision_node` & `advanced_vision_testing`)**: Primary vision daemon.
  - Detects gates and computes full 6-DoF pose relative to camera.
  - Broadcasts `VisionGoalEstimate` packets over UDP to `127.0.0.1:14553` (`AutopilotNode::vision_loop`).
  - Supports headless execution for embedded flight computers (Radxa Zero 3W, Raspberry Pi Zero 2W).
  - CLI arguments: `--headless`, `--camera <index|file>`, `--ip <addr>`, `--port <port>`, `--help`.
- **`FocalLengthCalibration.cpp` (built as `focal_calibration`)**: Pinhole camera calibration utility.
  - Calculates camera focal length ($F = (P \times D) / W$) from a known target at a fixed distance.
  - Calibrated camera focal length: **`566.8`**.
- **`ColorVisionTesting.cpp` (built as `color_vision_testing`)**: HSV color segmentation prototype with interactive trackbars for color masking and shape classification.
- **`OpenCVTesting.cpp` (built as `opencv_testing`)**: Baseline prototype for Canny edge detection and initial contour extraction.
- **`PythonVisionTesting.py`**: Standalone Python vision prototype implementing auto-polarity Otsu thresholding and hierarchical gate detection.
- **`tests/test_vision.cpp` (built as `test_vision`)**: Comprehensive CTest suite testing corner ordering, shape classification, SI unit conversions, packet memory layout, and UDP socket loopback.

## Physical Dimensions & Calibration

| Parameter | Imperial | Metric (SI) | Description |
| :--- | :--- | :--- | :--- |
| **`FOCAL_LENGTH`** | `566.8` px | `566.8` px | Calibrated camera focal length |
| **Square Gate Opening** | `11.25` in | `0.28575` m | Width and height of inner opening |
| **Pentagon Gate Opening** | `11.75` in | `0.29845` m | Max width across upper corners |

> [!NOTE]
> All internal 3D model coordinates in `gate_detector` are defined in **SI meters** ($1\text{ in} = 0.0254\text{ m}$) so that `solvePnP` output translation (`tvec`) directly corresponds to flight dynamics and state estimator expectations in the autopilot.
> Orientation angles for `VisionGoalEstimate` are transmitted in **radians**.

## Building & Testing

```bash
# Build all targets (requires libopencv-dev)
cmake -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build -j$(nproc)

# Run tests
ctest --test-dir build --output-on-failure

# Run vision node in headless mode on SBC:
./build/vision-processing/vision_node --headless --camera 0 --port 14553
```
