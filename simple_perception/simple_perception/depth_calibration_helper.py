#!/usr/bin/env python3
"""
Depth Calibration Helper

This script helps you calibrate the depth scale by subscribing to the depth image
and allowing you to click on a point to see the depth value.

Usage:
    ros2 run simple_perception depth_calibration_helper.py --ros-args -p depth_topic:=/depth_anything/depth

Then:
    1. Place an object at a known distance
    2. Click on that object in the depth image window
    3. Note the depth value shown
    4. Use that for reference_depth_value parameter
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np


class DepthCalibrationHelper(Node):
    def __init__(self):
        super().__init__('depth_calibration_helper')
        
        self.declare_parameter('depth_topic', '/depth_anything/depth')
        depth_topic = self.get_parameter('depth_topic').value
        
        self.bridge = CvBridge()
        self.depth_image = None
        self.click_coords = None
        
        self.subscription = self.create_subscription(
            Image,
            depth_topic,
            self.depth_callback,
            rclpy.qos.qos_profile_sensor_data  # Use sensor data QoS (BEST_EFFORT)
        )
        
        self.get_logger().info(f'Subscribed to depth topic: {depth_topic}')
        self.get_logger().info('Click on the depth image to see depth values at that point')
        self.get_logger().info('Press "q" to quit, "s" to save current view')
        
        # Set up the window and mouse callback
        cv2.namedWindow('Depth Calibration Helper')
        cv2.setMouseCallback('Depth Calibration Helper', self.mouse_callback)
        
    def mouse_callback(self, event, x, y, flags, param):
        """Handle mouse clicks on the depth image."""
        if event == cv2.EVENT_LBUTTONDOWN:
            self.click_coords = (x, y)
            
    def depth_callback(self, msg: Image):
        """Process incoming depth images."""
        try:
            # Convert to numpy array (32FC1)
            depth_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='32FC1')
            self.depth_image = depth_image
            
            # Normalize for visualization
            depth_normalized = cv2.normalize(depth_image, None, 0, 255, cv2.NORM_MINMAX)
            depth_colored = cv2.applyColorMap(depth_normalized.astype(np.uint8), cv2.COLORMAP_JET)
            
            # If user clicked, show information
            if self.click_coords is not None:
                x, y = self.click_coords
                if 0 <= y < depth_image.shape[0] and 0 <= x < depth_image.shape[1]:
                    depth_value = depth_image[y, x]
                    
                    # Draw crosshair
                    cv2.drawMarker(depth_colored, (x, y), (0, 255, 0), 
                                   cv2.MARKER_CROSS, 20, 2)
                    
                    # Add text with depth value
                    text = f"Depth: {depth_value:.3f}"
                    cv2.putText(depth_colored, text, (x + 10, y - 10),
                               cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
                    
                    self.get_logger().info(
                        f'Depth at ({x}, {y}): {depth_value:.3f}',
                        throttle_duration_sec=0.5
                    )
            
            # Add instructions
            cv2.putText(depth_colored, "Click to measure depth", (10, 30),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
            cv2.putText(depth_colored, "Press 'q' to quit, 's' to save", (10, 60),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
            
            # Show min/max in image
            min_depth = np.nanmin(depth_image)
            max_depth = np.nanmax(depth_image)
            cv2.putText(depth_colored, f"Min: {min_depth:.3f}", (10, depth_colored.shape[0] - 60),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
            cv2.putText(depth_colored, f"Max: {max_depth:.3f}", (10, depth_colored.shape[0] - 30),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
            
            # Display
            cv2.imshow('Depth Calibration Helper', depth_colored)
            key = cv2.waitKey(1)
            
            if key == ord('q'):
                self.get_logger().info('Press Ctrl+C in terminal to exit')
            elif key == ord('s') and self.depth_image is not None:
                filename = f'depth_calibration_{self.get_clock().now().nanoseconds}.png'
                cv2.imwrite(filename, depth_colored)
                self.get_logger().info(f'Saved image to {filename}')
                
        except Exception as e:
            self.get_logger().error(f'Error processing depth image: {e}')


def main(args=None):
    rclpy.init(args=args)
    
    node = DepthCalibrationHelper()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        cv2.destroyAllWindows()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
