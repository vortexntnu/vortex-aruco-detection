# ArUco Detector
[![Industrial CI](https://github.com/vortexntnu/vortex-aruco-detection/actions/workflows/industrial-ci.yml/badge.svg)](https://github.com/vortexntnu/vortex-aruco-detection/actions/workflows/industrial-ci.yml)
[![pre-commit.ci status](https://results.pre-commit.ci/badge/github/vortexntnu/vortex-aruco-detection/main.svg)](https://results.pre-commit.ci/latest/github/vortexntnu/vortex-aruco-detection/main)
[![codecov](https://codecov.io/github/vortexntnu/vortex-aruco-detection/graph/badge.svg?token=JesdLrO5Aj)](https://codecov.io/github/vortexntnu/vortex-aruco-detection)

This package provides a detector for ArUco markers in ROS2.

# Subscriptions

### Image Raw Topic
Subscripes to a `sensor_msgs::msg::Image` topic specified in the params file. This is the image on which aruco detection is performed.

### Camera Info Topic
Subscribes to a `sensor_msgs::msg::CameraInfo` topic necessary to retrieve the 3d pose of an aruco marker.


# Publishers

### ArUco Marker Poses
Detects arUco markers on the image. Calculates the position of the markers and publishes poses over the topic as a `geometry_msgs::msg::PoseArray` message.

### Board Pose
If the `detect_board` param is set then the board position is published as a `geometry_msgs::msg::PoseStamped` message over the board pose topic. The position published is the centre of the rectangular board created in `src/aruco_detector.cpp` in the `createRectangularBoard` function.


### Marker Image
If the `visualize` param is set the detected arUco markers, and the board centre, if enabled, are visualized on the input image used for detection.

### Usage

To use the ArUco detector, follow one of these steps:

1. Run the ArUco detector node to use default parameters specified in `src/aruco_detector_ros.cpp`:


    ```bash
    ros2 run aruco_detector aruco_detector_node
    ```


2. Launch the ArUco detector node to use parameters specified in `config/aruco_detector_params.yaml`:

   ```bash
    ros2 launch aruco_detector aruco_detector.launch.py
    ```

### Configuration

The ArUco detector can be configured by modifying the parameters in the `config/aruco_detector_params.yaml` file.
