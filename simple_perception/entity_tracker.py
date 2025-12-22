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
from vision_msgs.msg import Detection3DArray
from geometry_msgs.msg import PointStamped
from tf2_ros import Buffer, TransformListener
from geometry_msgs.msg import TransformStamped
from tf2_geometry_msgs import do_transform_point
from tf2_ros import TransformBroadcaster
import math


class EntityTracker(Node):
    def __init__(self):
        super().__init__('entity_tracker_node')

        self.declare_parameter('target_class', 'person')
        self.declare_parameter('source_frame', 'base_link')
        self.declare_parameter('target_frame', 'target')
        
        self.target_class = self.get_parameter('target_class').value
        self.source_frame = self.get_parameter('source_frame').value
        self.target_frame = self.get_parameter('target_frame').value
        self.get_logger().info(f'Tracking target class: {self.target_class}')
        

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.tf_broadcaster = TransformBroadcaster(self)

        # Subscriber to Detection3DArray
        self.sub = self.create_subscription(
            Detection3DArray,
            'input_detection_3d',
            self.detection_callback,
            rclpy.qos.qos_profile_sensor_data
        )

    def detection_callback(self, msg: Detection3DArray):
        if not msg.detections:
            return

        # Find first detection of the target class
        for detection in msg.detections:
            if detection.results and detection.results[0].hypothesis.class_id == self.target_class:
                self.publish_target_tf(detection)
                break

    def publish_target_tf(self, detection):

        target_point = PointStamped()
        target_point.header = detection.header
        target_point.point.x = detection.bbox.center.position.x
        target_point.point.y = detection.bbox.center.position.y
        target_point.point.z = detection.bbox.center.position.z

        detection_time = detection.header.stamp
        detection_frame = detection.header.frame_id

        try:
            # Lookup the transform
            self.get_logger().debug(f'Looking up transform from {self.source_frame} to {detection_frame}')
            source_2_detection = self.tf_buffer.lookup_transform(
                self.source_frame,
                detection_frame,
                detection_time,  # Use the actual timestamp from the sensor data
                timeout=rclpy.duration.Duration(seconds=0.5) 
            )

            transformed_point = do_transform_point(target_point, source_2_detection)
        except Exception as e:
            self.get_logger().error(f'Transform error: {e}')
            return

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
