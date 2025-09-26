# seek_thermal_publisher

Minimal ROS 2 Jazzy ament_python package that streams a Seek Thermal camera using the official seekcamera Python bindings and publishes: (1) a raw radiometric Image and (2) an autoscaled white-hot preview Image.

## Topics

- /seek/radiometric_raw (sensor_msgs/Image): Raw radiometric frames published as `mono16` for fixed-point thermography or `32FC1` for floating thermography, preserving full precision for downstream processing.
- /seek/auto_scaled (sensor_msgs/Image): Display-only white-hot `mono8` view produced by per-frame contrast scaling of the radiometric frame for easy visualization in tools like rqt_image_view or Foxglove.

## Encodings

- Fixed-point thermography: `mono16` indicates a 16-bit single-channel image where each pixel encodes a radiometric value from the camera in a fixed-point format provided by the SDK.
- Floating thermography: `32FC1` indicates a single-channel 32-bit float image, preserving radiometric values as provided by the SDK without color mapping or AGC.

## What does the raw stream represent?

- When the SDK exposes THERMOGRAPHY_FIXED_10_6, the camera delivers a fixed-point thermography plane (often referred to as “U10.6”), which the package publishes unmodified as `mono16`, enabling downstream conversion to temperature units without loss.
- When the SDK exposes THERMOGRAPHY_FLOAT, the camera delivers a radiometric float plane, which the package publishes as `32FC1`, suitable for direct temperature processing.
- The preview topic is only for visualization and does not alter the radiometric data on `/seek/radiometric_raw`; it applies simple per-frame scaling to fill the `mono8` range for a clean white-hot display.

## Build and run

- Place the package in a ROS 2 workspace `src/`, build with `colcon build`, and source `install/setup.sh` as with any ament_python package.
- Launch: `ros2 launch seek_thermal_publisher thermal_publisher.launch.py`, which starts the node with defaults `/seek/radiometric_raw` and `/seek/auto_scaled` and a small percentile clipping for stable viewing.

## Notes

- This package prefers `THERMOGRAPHY_FIXED_10_6` and falls back to `THERMOGRAPHY_FLOAT` or `GRAYSCALE` depending on what the installed SDK and camera expose, matching the current seekcamera Python bindings.
- Logging uses rclpy’s keyword-argument throttling for clean console output (e.g., `throttle_duration_sec`).
- Tested with seekcamera-python v1.2.0 and SDK v4.4.2.20 with ROS2 Jazzy on Ubuntu 24.04.
