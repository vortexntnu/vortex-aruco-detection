# ArUco Detector

This package provides a detector for ArUco markers in ROS2.

# Subscriptions

### Image Raw Topic
Subscipes to a `sensor_msgs::msg::Image` topic specified in the params file. This is the image on which aruco detection is performed.

### Camera Info Topic
The node sets default camera calibration parameters, but listens for a `sensor_msgs::msg::CameraInfo` topic to override the calibration parameters.


# Publishers

### ArUco Marker Poses
Detects arUco markers on the image. Calculates the position of the markers and publishes over the `/aruco_marker_poses` topic as a `geometry_msgs::msg::PoseArray` message.

### Board Pose
If the `detect_board` param is set then the board posistion is published as a `geometry_msgs::msg::PoseStamped` message over the `/aruco_board_pose` topic. The position published is the centre of the rectangular board created in `src/aruco_detector.cpp` in the `createRectangularBoard` function. 

The board position is used for autonomous docking in the TAC Challenge. This task requires and accurate position and therefore the board position is etimated through a kalman filter, to ensure more stable outputs.  

### Marker Image 
If the `visualize` param is set the detected arUco markers, and the board centre, if enabled, are visualized on the input image used for detection. The image is published over the `/aruco_marker_image` as a `sensor_msgs::msg::Image` message.

### Usage

To use the ArUco detector, follow one of these steps:

1. Run the ArUco detector node to use default parameters specified in `src/aruco_detector_ros.cpp`:
   

    ```bash
    ros2 run aruco_detector aruco_detector_node
    ```


2. Launch the ArUco detector node to use parameters specified in `params/aruco_parameters.yaml`:
   
   ```bash
    ros2 launch aruco_detector aruco_detector_launch.py
    ```

### Configuration

The ArUco detector can be configured by modifying the parameters in the `params/aruco_parameters.yaml` file.

