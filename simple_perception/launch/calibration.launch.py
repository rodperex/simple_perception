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

import os
from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """
    Launch calibration setup for entity_tracker_3d.
    
    This launch file starts:
    1. Depth Anything V2 ROS2 node for depth estimation
    2. Depth Calibration Helper for interactive calibration
    
    Usage:
    1. Launch this file
    2. Place an object at a known distance (e.g., 1.5m)
    3. Click on that object in the depth calibration window
    4. Note the depth value shown
    5. Use that value for reference_depth_value in entity_tracker_3d.launch.py
    """
    
    # Get default model path from depth_anything_v2_ros2 package
    depth_anything_dir = get_package_share_directory('depth_anything_v2_ros2')
    default_model_file = os.path.join(depth_anything_dir, 'models', 'depth_anything_v2_vits.pth')
    
    # Define nodes
    depth_node = Node(
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
    )
    
    calibration_node = Node(
        package='simple_perception',
        executable='depth_calibration_helper',
        name='depth_calibration_helper',
        output='screen',
        parameters=[{
            'depth_topic': LaunchConfiguration('depth_output_topic'),
        }]
    )
    
    return LaunchDescription([
        # Camera input arguments
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
        
        # Depth Anything V2 parameters
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
            default_value='10.0',
            description='Maximum depth value for depth_anything'
        ),
        
        # Nodes
        depth_node,
        calibration_node,

    ])
