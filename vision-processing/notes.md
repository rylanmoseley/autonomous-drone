# Vision Processing

This directory contains the computer vision modules for autonomous drone gate tracking, shape identification, and 6-DoF pose estimation.

## Files

- **`AdvancedVisionTesting.cpp`**: Primary gate detection algorithm.
  - Implements hierarchical contour search (`cv::RETR_TREE`) to isolate gate openings (child) inside the physical gate structure (parent).
  - Classifies gate shapes (Square, Pentagon, Circle) using circularity, fill ratio, and polygon approximation.
  - Computes full 6-DoF pose estimation (`X, Y, Z` translation and `Pitch, Yaw, Roll` rotation) using `cv::solvePnP`.
  - Outputs data directly compatible with `VisionPose` / `VisionGoalEstimate` in `shared/include/vision_packet.h`.
- **`FocalLengthCalibration.cpp`**: Pinhole camera calibration utility.
  - Calculates the camera's focal length (`F = (P * D) / W`) using a known target at a fixed distance.
  - Calibrated camera focal length: **`566.8`**.
- **`ColorVisionTesting.cpp`**: HSV color segmentation prototype with interactive trackbars for color masking and shape classification.
- **`OpenCVTesting.cpp`**: Baseline prototype for Canny edge detection and initial contour extraction.
- **`PythonVisionTesting.py`**: Standalone Python vision prototype implementing auto-polarity Otsu thresholding and hierarchical gate detection.

## Calibration & Dimensions

- **Camera Focal Length (`FOCAL_LENGTH`)**: `566.8`
- **Square Gate Opening (`KNOWN_SQUARE_WIDTH`)**: `11.25` inches
- **Pentagon Gate Opening (`KNOWN_PENTAGON_WIDTH`)**: `11.25` inches
