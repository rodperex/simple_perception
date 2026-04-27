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
from geometry_msgs.msg import PointStamped
from tf2_ros import Buffer, TransformListener
from geometry_msgs.msg import TransformStamped
from tf2_geometry_msgs import do_transform_point
from tf2_ros import TransformBroadcaster
from sensor_msgs.msg import CameraInfo
from simple_perception_interfaces.srv import SetTargetClass
import math


class EntityTracker(Node):
    def __init__(self):
        super().__init__('entity_tracker_node')

        self.declare_parameter('target_class', 'person')
        self.declare_parameter('source_frame', 'odom')
        self.declare_parameter('target_frame', 'target')
        self.declare_parameter('optical_frame', 'CameraTop_optical_frame')

        self.target_class = self.get_parameter('target_class').value
        self.source_frame = self.get_parameter('source_frame').value
        self.target_frame = self.get_parameter('target_frame').value
        self.optical_frame = self.get_parameter('optical_frame').value
        self.get_logger().info(f'Tracking target class: {self.target_class}')

        # Add parameter callback to allow runtime changes
        self.add_on_set_parameters_callback(self.parameter_callback)

        self.enable_processing = self.target_class != "none"

        self.configured = False

        # self.tf_buffer = Buffer()
        self.tf_buffer = Buffer(cache_time=rclpy.duration.Duration(seconds=3))
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.tf_broadcaster = TransformBroadcaster(self)

        self.sub = self.create_subscription(
            Detection2DArray,
            'input_detection_2d',
            self.detection_callback,
            rclpy.qos.qos_profile_sensor_data
        )

        self.camera_info_sub = self.create_subscription(
            CameraInfo,
            'camera_info',
            self.camera_info_callback,
            rclpy.qos.qos_profile_sensor_data
        )

        # Create service to set target class
        self.set_target_class_service = self.create_service(
            SetTargetClass,
            'set_perception_target',
            self.set_target_class_callback
        )
        self.get_logger().info('Service /set_perception_target is ready')

    def parameter_callback(self, params):
        """Callback for parameter changes at runtime."""
        from rcl_interfaces.msg import SetParametersResult
        
        for param in params:
            if param.name == 'target_class':
                self.target_class = param.value
                self.get_logger().info(f'Target class changed to: {self.target_class}')
        
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
        # The intrinsic matrix K is a 9-element array (row-major order)
        # K = [fx, 0, cx, 0, fy, cy, 0, 0, 1]
        self.f_x = msg.k[0] # fx is K[0]
        self.c_x = msg.k[2] # cx is K[2]

        self.current_image_size = (msg.width, msg.height)
        self.get_logger().info(f'Got image of size: {msg.width}x{msg.height}')
        self.get_logger().info(f'Got camera intrinsics: fx={self.f_x:.2f}, cx={self.c_x:.2f}')
        self.configured = True
        self.destroy_subscription(self.camera_info_sub)

    def detection_callback(self, msg: Detection2DArray):
        if not hasattr(self, 'enable_processing'):
            self.enable_processing = False

        if not self.enable_processing:
            self.get_logger().debug('Detection processing DISABLED: target_class == "none" or not enabled.')
            return

        if not self.configured:
            self.get_logger().warn('Camera info not yet received, cannot compute angles')
            return

        if not msg.detections:
            self.get_logger().debug('No detections in the message')
            return

        self.get_logger().debug(f'Received {len(msg.detections)} detections, looking for class "{self.target_class}"')

        # Find first detection of the target class
        for detection in msg.detections:
            self.get_logger().debug(f'Checking detection with class_id: {detection.results[0].hypothesis.class_id if detection.results else "None"}')
            if detection.results and detection.results[0].hypothesis.class_id == self.target_class:
                self.get_logger().info(f'Detected target class "{self.target_class}" in image')
                self.publish_target_tf(detection)
                break

    def publish_target_tf(self, detection):

        if not self.configured:
            self.get_logger().warn('Camera info not yet received, cannot compute angles')
            return
        
        # Calculate angle relative to image center using camera intrinsics
        x_pixel = detection.bbox.center.position.x
        
        
        pixel_offset_x = x_pixel - self.c_x
        angle = math.atan(pixel_offset_x / self.f_x)
        
        self.get_logger().debug(f'Detected {self.target_class} at angle {math.degrees(angle):.1f} degrees ({self.optical_frame})')
        
        # Create a point at 1m distance based on the computed angle
        x_optical = math.tan(angle)
        y_optical = 0.0  
        z_optical = 1.0  # Fixed distance of 1 meter

        target_point = PointStamped()
        target_point.header = detection.header
        target_point.point.x = x_optical
        target_point.point.y = y_optical
        target_point.point.z = z_optical

        detection_time = detection.header.stamp
        detection_frame = self.optical_frame

        try:
            # Lookup the transform
            self.get_logger().debug(f'Looking up transform from {self.source_frame} to {detection_frame}')
            source_2_detection = self.tf_buffer.lookup_transform(
                self.source_frame,
                detection_frame,
                rclpy.time.Time() 
            )

            transformed_point = do_transform_point(target_point, source_2_detection)
        except Exception as e:
            self.get_logger().error(f'Transform error: {e}')
            return

        transform = TransformStamped()
        # transform.header.stamp = self.get_clock().now().to_msg()
        transform.header.stamp = detection_time
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
        self.get_logger().info(f'Target {self.target_class} @ angle {math.degrees(angle):.1f}° -> position ({transformed_point.point.x:.2f}, {transformed_point.point.y:.2f}, {transformed_point.point.z:.2f}) in {self.source_frame}')

        self.get_logger().info(f'Publishing transform for target at ({transformed_point.point.x:.2f}, {transformed_point.point.y:.2f}, {transformed_point.point.z:.2f})')
        self.tf_broadcaster.sendTransform(transform)
        

def main(args=None):
    rclpy.init(args=args)
    node = EntityTracker()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
