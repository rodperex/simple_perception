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
# See the License for the specific language governing permissions and
# limitations under the License.

import os
from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    """
    Launch pipeline for monocular depth + 2D detections -> simulated Detection3DArray.

    This launch file starts:
    1. Depth Anything V2 ROS2 node for monocular depth estimation
    2. detection_3d_simulator node to convert 2D detections + depth into Detection3DArray

    You must launch your 2D detection node separately (e.g., YOLO).

    Calibration tips:
    - Place an object at a known distance (e.g., 1.5m)
    - Observe the depth value in the depth image at that location
    - Set reference_depth_value and reference_distance in detection_3d_simulator
    - Or use depth_scale_factor to fine-tune the scale
    """

    # Get default model path from depth_anything_v2_ros2 package
    depth_anything_dir = get_package_share_directory('depth_anything_v2_ros2')
    default_model_file = os.path.join(depth_anything_dir, 'models', 'depth_anything_v2_vits.pth')

    return LaunchDescription([
        # --- Arguments for Depth Anything V2 node ---
        DeclareLaunchArgument(
            'image_topic',
            default_value='/image_rgb',
            description='Input RGB image topic for depth estimation'
        ),
        DeclareLaunchArgument(
            'depth_output_topic',
            default_value='/depth_anything/depth',
            description='Output topic for depth images'
        ),
        DeclareLaunchArgument(
            'depth_model_file',
            default_value=default_model_file,
            description='Path to depth_anything model file'
        ),
        DeclareLaunchArgument(
            'depth_encoder',
            default_value='vits',
            description='Depth model encoder (vits, vitb, or vitl)'
        ),
        DeclareLaunchArgument(
            'depth_device',
            default_value='cuda:0',
            description='Device for depth estimation (cuda:0 or cpu)'
        ),
        DeclareLaunchArgument(
            'depth_max_depth',
            default_value='20.0',
            description='Maximum depth value for depth_anything (clip, not conversion)'
        ),

        # --- Arguments for Detection3D Simulator node ---
        DeclareLaunchArgument(
            'input_detection_2d_topic',
            default_value='/detections_2d',
            description='Input topic for Detection2DArray (from detector)'
        ),
        DeclareLaunchArgument(
            'depth_image_topic',
            default_value='/depth_anything/depth',
            description='Input topic for depth image (from depth_anything)'
        ),
        DeclareLaunchArgument(
            'camera_info_topic',
            default_value='/camera_info',
            description='Input topic for camera info'
        ),
        DeclareLaunchArgument(
            'detection_3d_topic',
            default_value='/simulated_detection_3d',
            description='Output topic for simulated Detection3DArray'
        ),
        DeclareLaunchArgument(
            'depth_scale_factor',
            default_value='1.0',
            description='Manual depth scaling factor (if not using reference calibration)'
        ),
        DeclareLaunchArgument(
            'reference_distance',
            default_value='1.5',
            description='Known reference distance in meters for calibration'
        ),
        DeclareLaunchArgument(
            'reference_depth_value',
            default_value='1.0',
            description='Depth image value corresponding to reference_distance'
        ),
        DeclareLaunchArgument(
            'min_depth',
            default_value='0.1',
            description='Minimum valid depth in meters'
        ),
        DeclareLaunchArgument(
            'max_depth',
            default_value='20.0',
            description='Maximum valid depth in meters'
        ),
        DeclareLaunchArgument(
            'use_bbox_center',
            default_value='True',
            description='Use bbox center for depth (True) or median of bbox region (False)'
        ),

        # --- Node: Depth Anything V2 (monocular depth estimation) ---
        Node(
            package='depth_anything_v2_ros2',
            executable='depth_anything_v2_ros2',
            name='depth_anything_v2',
            output='screen',
            parameters=[{
                'image_topic': LaunchConfiguration('image_topic'),
                'depth_image_topic': LaunchConfiguration('depth_output_topic'),
                'model_file': LaunchConfiguration('depth_model_file'),
                'encoder': LaunchConfiguration('depth_encoder'),
                'device': LaunchConfiguration('depth_device'),
                'max_depth': LaunchConfiguration('depth_max_depth'),
            }]
        ),

        # --- Node: Detection 3D Simulator (2D+depth -> Detection3DArray) ---
        Node(
            package='simple_perception',
            executable='detection_3d_simulator',
            name='detection_3d_simulator',
            output='screen',
            parameters=[{
                'output_topic': LaunchConfiguration('detection_3d_topic'),
                'depth_scale_factor': LaunchConfiguration('depth_scale_factor'),
                'reference_distance': LaunchConfiguration('reference_distance'),
                'reference_depth_value': LaunchConfiguration('reference_depth_value'),
                'min_depth': LaunchConfiguration('min_depth'),
                'max_depth': LaunchConfiguration('max_depth'),
                'use_bbox_center': LaunchConfiguration('use_bbox_center'),
            }],
            remappings=[
                ('input_detection_2d', LaunchConfiguration('input_detection_2d_topic')),
                ('depth_image', LaunchConfiguration('depth_image_topic')),
                ('camera_info', LaunchConfiguration('camera_info_topic')),
            ]
        ),
    ])