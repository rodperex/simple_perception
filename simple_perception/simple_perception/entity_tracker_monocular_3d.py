#!/usr/bin/env python3
# Copyright 2025 Rodrigo Pérez-Rodríguez
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.

import rclpy
from rclpy.node import Node
from vision_msgs.msg import Detection2DArray
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PointStamped, TransformStamped
from tf2_ros import Buffer, TransformListener, TransformBroadcaster
from tf2_geometry_msgs import do_transform_point
from simple_perception_interfaces.srv import SetTargetClass
from cv_bridge import CvBridge
import numpy as np
import math
from message_filters import Subscriber, ApproximateTimeSynchronizer


class EntityTrackerMonocular3D(Node):
    """
    Entity tracker that uses monocular depth estimation (relative depth) to compute 3D positions.
    
    Strategy for converting relative to metric depth:
    - Uses a reference distance parameter to calibrate the depth scale
    - Assumes relative depth is proportional to real depth
    - Can be calibrated by measuring a known distance
    """
    
    def __init__(self):
        super().__init__('entity_tracker_3d_node')

        # Parameters
        self.declare_parameter('target_class', 'person')
        self.declare_parameter('source_frame', 'base_link')
        self.declare_parameter('target_frame', 'target')
        self.declare_parameter('optical_frame', 'CameraTop_optical_frame')
        self.declare_parameter('depth_scale_factor', 1.0)  # Calibration factor
        self.declare_parameter('reference_distance', 1.0)  # Known reference distance in meters
        self.declare_parameter('reference_depth_value', 1.0)  # Corresponding depth value at reference
        self.declare_parameter('min_depth', 0.3)  # Minimum valid depth in meters
        self.declare_parameter('max_depth', 10.0)  # Maximum valid depth in meters
        self.declare_parameter('use_bbox_center', True)  # Sample depth at bbox center vs median of bbox
        
        self.target_class = self.get_parameter('target_class').value
        self.source_frame = self.get_parameter('source_frame').value
        self.target_frame = self.get_parameter('target_frame').value
        self.optical_frame = self.get_parameter('optical_frame').value
        self.depth_scale_factor = self.get_parameter('depth_scale_factor').value
        self.reference_distance = self.get_parameter('reference_distance').value
        self.reference_depth_value = self.get_parameter('reference_depth_value').value
        self.min_depth = self.get_parameter('min_depth').value
        self.max_depth = self.get_parameter('max_depth').value
        self.use_bbox_center = self.get_parameter('use_bbox_center').value
        
        self.get_logger().info(f'Tracking target class: {self.target_class}')
        self.get_logger().info(f'Depth scale factor: {self.depth_scale_factor}')
        self.get_logger().info(f'Reference: {self.reference_depth_value} depth value = {self.reference_distance}m')
        
        # Add parameter callback
        self.add_on_set_parameters_callback(self.parameter_callback)
        
        self.configured = False
        self.bridge = CvBridge()
        self.latest_depth_image = None
        
        # Camera intrinsics
        self.f_x = None
        self.f_y = None
        self.c_x = None
        self.c_y = None

        # TF
        self.tf_buffer = Buffer(cache_time=rclpy.duration.Duration(seconds=3))
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.tf_broadcaster = TransformBroadcaster(self)

        # Camera info subscription
        self.camera_info_sub = self.create_subscription(
            CameraInfo,
            'camera_info',
            self.camera_info_callback,
            rclpy.qos.qos_profile_sensor_data
        )

        # Synchronized subscriptions for detections and depth
        # Use sensor data QoS profile to match publishers (BEST_EFFORT)
        self.detection_sub = Subscriber(
            self, 
            Detection2DArray,
            'input_detection_2d',
            qos_profile=rclpy.qos.qos_profile_sensor_data
        )
        
        self.depth_sub = Subscriber(
            self,
            Image,
            'depth_image',
            qos_profile=rclpy.qos.qos_profile_sensor_data
        )
        
        # Synchronize detection and depth messages
        self.sync = ApproximateTimeSynchronizer(
            [self.detection_sub, self.depth_sub],
            queue_size=10,
            slop=0.1  # 100ms tolerance
        )
        self.sync.registerCallback(self.synchronized_callback)

        # Service to set target class
        self.set_target_class_service = self.create_service(
            SetTargetClass,
            'set_perception_target',
            self.set_target_class_callback
        )
        self.get_logger().info('Service /set_perception_target is ready')
        self.get_logger().info('Waiting for synchronized detections and depth images...')

    def parameter_callback(self, params):
        """Callback for parameter changes at runtime."""
        from rcl_interfaces.msg import SetParametersResult
        
        for param in params:
            if param.name == 'target_class':
                self.target_class = param.value
                self.get_logger().info(f'Target class changed to: {self.target_class}')
            elif param.name == 'depth_scale_factor':
                self.depth_scale_factor = param.value
                self.get_logger().info(f'Depth scale factor changed to: {self.depth_scale_factor}')
        
        return SetParametersResult(successful=True)

    def set_target_class_callback(self, request, response):
        """Service callback to set the target class and enable/disable detection processing."""
        try:
            self.target_class = request.target_class
            self.get_logger().info(f'Target class changed via service to: {self.target_class}')
            if self.target_class != "none":
                self.enable_processing = True
                self.get_logger().info('Detection processing ENABLED')
            else:
                self.enable_processing = False
                self.get_logger().info('Detection processing DISABLED')
            response.success = True
            response.message = f'Target class successfully set to: {self.target_class}'
        except Exception as e:
            self.get_logger().error(f'Failed to set target class: {str(e)}')
            response.success = False
            response.message = f'Failed to set target class: {str(e)}'
        
        return response

    def camera_info_callback(self, msg: CameraInfo):
        """Extract camera intrinsic parameters."""
        # K = [fx, 0, cx, 0, fy, cy, 0, 0, 1]
        self.f_x = msg.k[0]  # fx
        self.f_y = msg.k[4]  # fy
        self.c_x = msg.k[2]  # cx
        self.c_y = msg.k[5]  # cy
        
        self.image_width = msg.width
        self.image_height = msg.height
        
        self.get_logger().info(f'Camera info: {msg.width}x{msg.height}')
        self.get_logger().info(f'Intrinsics: fx={self.f_x:.2f}, fy={self.f_y:.2f}, cx={self.c_x:.2f}, cy={self.c_y:.2f}')
        self.configured = True
        self.destroy_subscription(self.camera_info_sub)

    def synchronized_callback(self, detection_msg: Detection2DArray, depth_msg: Image):
        """Handle synchronized detection and depth messages."""

        if not hasattr(self, 'enable_processing'):
            self.enable_processing = False

        if not self.enable_processing:
            self.get_logger().debug('Detection processing DISABLED: target_class == "none" or not enabled.')
            return

        self.get_logger().debug('Synchronized detection and depth messages received')

        if not self.configured:
            self.get_logger().warn('Camera info not yet received', throttle_duration_sec=2.0)
            return

        # Convert depth image to numpy array
        try:
            # Depth image is 32FC1 (single channel float32)
            depth_image = self.bridge.imgmsg_to_cv2(depth_msg, desired_encoding='32FC1')
            self.latest_depth_image = depth_image
        except Exception as e:
            self.get_logger().error(f'Failed to convert depth image: {e}')
            return

        if not detection_msg.detections:
            return

        # Find first detection of the target class
        for detection in detection_msg.detections:
            if detection.results and detection.results[0].hypothesis.class_id == self.target_class:
                self.process_detection_3d(detection, depth_image, detection_msg.header)
                break

    def process_detection_3d(self, detection, depth_image, header):
        """Process a detection with depth information to compute 3D position."""
        
        self.get_logger().debug(f'Processing 3D detection of class: {self.target_class}')

        # Extract bounding box
        bbox = detection.bbox
        center_x = bbox.center.position.x
        center_y = bbox.center.position.y
        size_x = bbox.size_x
        size_y = bbox.size_y
        
        # Calculate bbox boundaries
        x_min = int(max(0, center_x - size_x / 2))
        x_max = int(min(self.image_width, center_x + size_x / 2))
        y_min = int(max(0, center_y - size_y / 2))
        y_max = int(min(self.image_height, center_y + size_y / 2))
        
        # Sample depth from the detection region
        if self.use_bbox_center:
            # Use depth at center pixel
            depth_value = depth_image[int(center_y), int(center_x)]
        else:
            # Use median depth in bbox (more robust to outliers)
            depth_region = depth_image[y_min:y_max, x_min:x_max]
            valid_depths = depth_region[~np.isnan(depth_region)]
            
            if len(valid_depths) == 0:
                self.get_logger().warn('No valid depth values in detection bbox')
                return
            
            depth_value = np.median(valid_depths)
        
        # Convert relative depth to metric depth
        # Using linear scaling based on reference calibration
        metric_depth = self.convert_relative_to_metric_depth(depth_value)
        
        # Validate depth
        if metric_depth < self.min_depth or metric_depth > self.max_depth:
            self.get_logger().warn(
                f'Depth {metric_depth:.2f}m outside valid range [{self.min_depth}, {self.max_depth}]',
                throttle_duration_sec=1.0
            )
            return
        
        # Project to 3D using camera intrinsics
        point_3d = self.project_to_3d(center_x, center_y, metric_depth)
        
        # Create point in camera optical frame
        target_point = PointStamped()
        target_point.header = header
        target_point.header.frame_id = self.optical_frame
        target_point.point.x = point_3d[0]
        target_point.point.y = point_3d[1]
        target_point.point.z = point_3d[2]
        
        # Transform to base frame and publish
        self.publish_target_tf(target_point)

    def convert_relative_to_metric_depth(self, relative_depth):
        """
        Convert relative depth value to metric depth.
        
        Approaches:
        1. Simple scaling: metric = relative * depth_scale_factor
        2. Reference calibration: metric = (relative / reference_depth_value) * reference_distance
        
        For best results, calibrate by:
        - Place object at known distance (e.g., 1.5m)
        - Observe the depth value
        - Set reference_depth_value and reference_distance parameters
        """
        if self.depth_scale_factor != 1.0:
            # Use manual scale factor
            return relative_depth * self.depth_scale_factor
        else:
            # Use reference calibration
            return (relative_depth / self.reference_depth_value) * self.reference_distance

    def project_to_3d(self, pixel_x, pixel_y, depth):
        """
        Project 2D pixel + depth to 3D point in camera frame.
        
        Standard pinhole camera model:
        X = (u - cx) * Z / fx
        Y = (v - cy) * Z / fy
        Z = depth
        """
        X = (pixel_x - self.c_x) * depth / self.f_x
        Y = (pixel_y - self.c_y) * depth / self.f_y
        Z = depth
        
        return np.array([X, Y, Z])

    def publish_target_tf(self, target_point: PointStamped):
        """Transform point to base frame and publish TF."""
        try:
            # Transform from optical frame to source frame
            source_2_detection = self.tf_buffer.lookup_transform(
                self.source_frame,
                self.optical_frame,
                rclpy.time.Time()
            )
            
            transformed_point = do_transform_point(target_point, source_2_detection)
            
        except Exception as e:
            self.get_logger().error(f'Transform error: {e}', throttle_duration_sec=1.0)
            return

        # Create and publish transform
        transform = TransformStamped()
        transform.header.stamp = self.get_clock().now().to_msg()
        transform.header.frame_id = self.source_frame
        transform.child_frame_id = self.target_frame
        transform.transform.translation.x = transformed_point.point.x
        transform.transform.translation.y = transformed_point.point.y
        transform.transform.translation.z = transformed_point.point.z
        transform.transform.rotation.x = 0.0
        transform.transform.rotation.y = 0.0
        transform.transform.rotation.z = 0.0
        transform.transform.rotation.w = 1.0

        angle = math.atan2(transformed_point.point.y, transformed_point.point.x)
        distance = math.sqrt(
            transformed_point.point.x**2 + 
            transformed_point.point.y**2 + 
            transformed_point.point.z**2
        )
        
        self.get_logger().info(
            f'Target {self.target_class} @ {math.degrees(angle):.1f}° | '
            f'{distance:.2f}m | '
            f'pos ({transformed_point.point.x:.2f}, {transformed_point.point.y:.2f}, '
            f'{transformed_point.point.z:.2f}) in {self.source_frame}'
        )

        self.tf_broadcaster.sendTransform(transform)


def main(args=None):
    rclpy.init(args=args)
    node = EntityTrackerMonocular3D()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
