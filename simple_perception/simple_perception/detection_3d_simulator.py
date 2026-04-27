#!/usr/bin/env python3
# Copyright 2026 Rodrigo Pérez-Rodríguez
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
from vision_msgs.msg import Detection2DArray, Detection3DArray, Detection3D, ObjectHypothesisWithPose
from vision_msgs.msg import BoundingBox3D
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import Point, Pose, PoseWithCovariance, PoseWithCovarianceStamped
from std_msgs.msg import Header
from cv_bridge import CvBridge
import numpy as np
from message_filters import Subscriber, ApproximateTimeSynchronizer

class Detection3DSimulator(Node):
    def __init__(self):
        super().__init__('detection_3d_simulator_node')

        # Parameters
        self.declare_parameter('output_topic', 'simulated_detection_3d')
        self.declare_parameter('optical_frame', 'CameraTop_optical_frame')
        self.declare_parameter('depth_scale_factor', 1.0)
        self.declare_parameter('reference_distance', 1.0)
        self.declare_parameter('reference_depth_value', 1.0)
        self.declare_parameter('min_depth', 0.3)
        self.declare_parameter('max_depth', 10.0)
        self.declare_parameter('use_bbox_center', True)

        self.output_topic = self.get_parameter('output_topic').value
        self.optical_frame = self.get_parameter('optical_frame').value
        self.depth_scale_factor = self.get_parameter('depth_scale_factor').value
        self.reference_distance = self.get_parameter('reference_distance').value
        self.reference_depth_value = self.get_parameter('reference_depth_value').value
        self.min_depth = self.get_parameter('min_depth').value
        self.max_depth = self.get_parameter('max_depth').value
        self.use_bbox_center = self.get_parameter('use_bbox_center').value

        self.bridge = CvBridge()
        self.configured = False

        # Camera intrinsics
        self.f_x = None
        self.f_y = None
        self.c_x = None
        self.c_y = None
        self.image_width = None
        self.image_height = None

        # Camera info subscription
        self.camera_info_sub = self.create_subscription(
            CameraInfo,
            'camera_info',
            self.camera_info_callback,
            rclpy.qos.qos_profile_sensor_data
        )

        # Synchronized subscriptions for detections and depth
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
        self.sync = ApproximateTimeSynchronizer(
            [self.detection_sub, self.depth_sub],
            queue_size=10,
            slop=0.1
        )
        self.sync.registerCallback(self.synchronized_callback)

        # Publisher for Detection3DArray
        self.detection3d_pub = self.create_publisher(
            Detection3DArray,
            self.output_topic,
            10
        )
        self.get_logger().info(f'Publishing simulated Detection3DArray on topic: {self.output_topic}')

    def camera_info_callback(self, msg: CameraInfo):
        self.f_x = msg.k[0]
        self.f_y = msg.k[4]
        self.c_x = msg.k[2]
        self.c_y = msg.k[5]
        self.image_width = msg.width
        self.image_height = msg.height
        self.configured = True
        self.get_logger().info(f'Camera info received: {msg.width}x{msg.height}')
        self.destroy_subscription(self.camera_info_sub)

    def synchronized_callback(self, detection_msg: Detection2DArray, depth_msg: Image):
        if not self.configured:
            self.get_logger().warn('Camera info not yet received', throttle_duration_sec=2.0)
            return
        try:
            depth_image = self.bridge.imgmsg_to_cv2(depth_msg, desired_encoding='32FC1')
        except Exception as e:
            self.get_logger().error(f'Failed to convert depth image: {e}')
            return
        detections_3d = []
        for detection in detection_msg.detections:
            det3d = self.detection2d_to_3d(detection, depth_image, detection_msg.header)
            if det3d is not None:
                detections_3d.append(det3d)
        if detections_3d:
            msg3d = Detection3DArray()
            msg3d.header = detection_msg.header
            msg3d.detections = detections_3d
            self.detection3d_pub.publish(msg3d)

    def detection2d_to_3d(self, detection, depth_image, header):
        bbox = detection.bbox
        center_x = bbox.center.position.x
        center_y = bbox.center.position.y
        size_x = bbox.size_x
        size_y = bbox.size_y
        x_min = int(max(0, center_x - size_x / 2))
        x_max = int(min(self.image_width, center_x + size_x / 2))
        y_min = int(max(0, center_y - size_y / 2))
        y_max = int(min(self.image_height, center_y + size_y / 2))
        if self.use_bbox_center:
            depth_value = depth_image[int(center_y), int(center_x)]
        else:
            depth_region = depth_image[y_min:y_max, x_min:x_max]
            valid_depths = depth_region[~np.isnan(depth_region)]
            if len(valid_depths) == 0:
                self.get_logger().warn('No valid depth values in detection bbox')
                return None
            depth_value = np.median(valid_depths)
        metric_depth = self.convert_relative_to_metric_depth(depth_value)
        if metric_depth < self.min_depth or metric_depth > self.max_depth:
            self.get_logger().warn(f'Depth {metric_depth:.2f}m outside valid range [{self.min_depth}, {self.max_depth}]', throttle_duration_sec=1.0)
            return None
        point_3d = self.project_to_3d(center_x, center_y, metric_depth)
        # Build Detection3D message
        det3d = Detection3D()
        det3d.header = header
        det3d.results = []
        for result in detection.results:
            hyp = ObjectHypothesisWithPose()
            hyp.hypothesis = result.hypothesis
            hyp.pose.pose.position.x = point_3d[0]
            hyp.pose.pose.position.y = point_3d[1]
            hyp.pose.pose.position.z = point_3d[2]
            hyp.pose.pose.orientation.w = 1.0
            det3d.results.append(hyp)
        det3d.bbox.center.position.x = point_3d[0]
        det3d.bbox.center.position.y = point_3d[1]
        det3d.bbox.center.position.z = point_3d[2]
        det3d.bbox.size.x = 0.1
        det3d.bbox.size.y = 0.1
        det3d.bbox.size.z = 0.1
        return det3d

    def convert_relative_to_metric_depth(self, relative_depth):
        if self.depth_scale_factor != 1.0:
            return relative_depth * self.depth_scale_factor
        else:
            return (relative_depth / self.reference_depth_value) * self.reference_distance

    def project_to_3d(self, pixel_x, pixel_y, depth):
        X = (pixel_x - self.c_x) * depth / self.f_x
        Y = (pixel_y - self.c_y) * depth / self.f_y
        Z = depth
        return np.array([X, Y, Z])

def main(args=None):
    rclpy.init(args=args)
    node = Detection3DSimulator()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
