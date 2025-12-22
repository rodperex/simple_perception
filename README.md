# simple_perception

Simple perception utilities for object tracking and detection format conversion.

## Overview

This package provides ROS 2 nodes for:
- **Entity tracking**: Track objects detected by YOLO and publish TF transforms
- **Detection conversion**: Convert YOLO detection messages to standard vision_msgs format
- **Depth estimation**: Support for monocular depth estimation using Depth-Anything-V2 (coming soon)

## Nodes

### entity_tracker_fake_3d
Tracks entities using 2D detections and estimates 3D position using camera parameters.

**Parameters:**
- `target_class` (string, default: "person"): Class of object to track
- `source_frame` (string, default: "base_link"): Base frame for tracking
- `target_frame` (string, default: "target"): TF frame name for tracked target
- `optical_frame` (string, default: "CameraTop_optical_frame"): Camera optical frame

**Subscribed Topics:**
- `/input_detection_2d` (vision_msgs/Detection2DArray): 2D detections
- `/camera_info` (sensor_msgs/CameraInfo): Camera calibration

**Published Topics:**
- `/tf` (tf2_msgs/TFMessage): Target transform

### entity_tracker
Tracks entities using 3D detections.

**Parameters:**
- `target_class` (string, default: "person"): Class of object to track
- `source_frame` (string, default: "base_link"): Base frame for tracking
- `target_frame` (string, default: "target"): TF frame name for tracked target

**Subscribed Topics:**
- `/input_detection_3d` (vision_msgs/Detection3DArray): 3D detections

**Published Topics:**
- `/tf` (tf2_msgs/TFMessage): Target transform

### yolo_to_standard
Converts YOLO detection messages to standard vision_msgs format.

**Subscribed Topics:**
- `input_detection_2d` (yolo_msgs/DetectionArray): YOLO 2D detections
- `input_detection_3d` (yolo_msgs/DetectionArray): YOLO 3D detections

**Published Topics:**
- `output_detection_2d` (vision_msgs/Detection2DArray): Standard 2D detections
- `output_detection_3d` (vision_msgs/Detection3DArray): Standard 3D detections

## Launch Files

### simple_perception.launch.py
Launches the complete perception stack including YOLO detector, detection converter, and entity tracker.

**Arguments:**
- `fake_3d` (bool, default: true): Use fake 3D tracker (true) or real 3D tracker (false)
- `target_class` (string, default: "tv"): Target class to track
- `source_frame` (string, default: "base_link"): Source frame
- `target_frame` (string, default: "target"): Target TF frame name
- `optical_frame` (string, default: "CameraTop_optical_frame"): Camera optical frame
- `input_image_topic` (string, default: "/image_rgb"): RGB image topic
- `input_depth_topic` (string, default: "/image_depth"): Depth image topic
- `input_depth_info_topic` (string, default: "/camera_rgb_info"): Camera info topic
- `camera_frame` (string, default: "CameraTop_frame"): Camera frame

**Usage:**
```bash
# Basic usage with fake 3D tracker (default)
ros2 launch simple_perception simple_perception.launch.py

# Use real 3D tracker
ros2 launch simple_perception simple_perception.launch.py fake_3d:=false

# Custom configuration
ros2 launch simple_perception simple_perception.launch.py \
    fake_3d:=true \
    target_class:=bottle \
    input_image_topic:=/camera/image_raw \
    input_depth_topic:=/camera/depth/image_raw
```

## Third-party Dependencies

This package requires the following third-party ROS 2 packages:

- **[yolo_ros](https://github.com/mgonzs13/yolo_ros)**: YOLO object detection for ROS 2
- **[depth_anything_v2_ros2](https://github.com/grupo-avispa/depth_anything_v2_ros2)**: Monocular depth estimation (for future depth support)

To clone the dependencies:

```bash
cd ~/ros2_ws/src/simple_perception
vcs import < thirdparty.repos
```

## Dependencies

- rclpy
- vision_msgs
- geometry_msgs
- sensor_msgs
- tf2_ros
- tf2_geometry_msgs
- yolo_msgs
- yolo_bringup

## Building

First, install third-party dependencies:

```bash
cd ~/ros2_ws/src/simple_perception
vcs import < thirdparty.repos
cd ~/ros2_ws
rosdep install --from-paths src --ignore-src -r -y
```

Then build the package:

```bash
cd ~/ros2_ws
colcon build --packages-select simple_perception
source install/setup.bash
```

## License

Apache-2.0
