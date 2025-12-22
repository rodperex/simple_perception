#!/usr/bin/env python3

# Copyright 2025 Rodrigo Pérez-Rodríguez
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#    http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python import get_package_share_directory


def generate_launch_description():
    """
    Launch file for simple perception stack.
    Includes entity tracker, YOLO converter, and YOLO detector.
    """
    
    # Declare launch arguments
    fake_3d_arg = DeclareLaunchArgument(
        'fake_3d',
        default_value='true',
        description='Use fake 3D tracker (true) or real 3D tracker (false)'
    )
    
    target_class_arg = DeclareLaunchArgument(
        'target_class',
        default_value='tv',
        description='Target class to track'
    )
    
    source_frame_arg = DeclareLaunchArgument(
        'source_frame',
        default_value='base_link',
        description='Source frame for tracking'
    )
    
    target_frame_arg = DeclareLaunchArgument(
        'target_frame',
        default_value='target',
        description='Target frame name for TF broadcast'
    )
    
    optical_frame_arg = DeclareLaunchArgument(
        'optical_frame',
        default_value='CameraTop_optical_frame',
        description='Camera optical frame'
    )
    
    input_image_topic_arg = DeclareLaunchArgument(
        'input_image_topic',
        default_value='/image_rgb',
        description='Input image topic for YOLO'
    )
    
    input_depth_topic_arg = DeclareLaunchArgument(
        'input_depth_topic',
        default_value='/image_depth',
        description='Input depth topic for YOLO'
    )
    
    input_depth_info_topic_arg = DeclareLaunchArgument(
        'input_depth_info_topic',
        default_value='/camera_rgb_info',
        description='Input depth info topic for YOLO'
    )
    
    camera_frame_arg = DeclareLaunchArgument(
        'camera_frame',
        default_value='CameraTop_frame',
        description='Camera frame for YOLO'
    )
    
    # Entity tracker node (fake 3D version - default)
    entity_tracker_fake_3d_node = Node(
        package='simple_perception',
        executable='entity_tracker_fake_3d',
        name='entity_tracker_node',
        output='screen',
        parameters=[{
            'target_class': LaunchConfiguration('target_class'),
            'source_frame': LaunchConfiguration('source_frame'),
            'target_frame': LaunchConfiguration('target_frame'),
            'optical_frame': LaunchConfiguration('optical_frame')
        }],
        remappings=[
            ('/input_detection_2d', '/detections_2d'),
            ('/camera_info', '/camera_rgb_info')
        ],
        condition=IfCondition(LaunchConfiguration('fake_3d'))
    )
    
    # Entity tracker node (3D version)
    entity_tracker_3d_node = Node(
        package='simple_perception',
        executable='entity_tracker',
        name='entity_tracker_node',
        output='screen',
        parameters=[{
            'target_class': LaunchConfiguration('target_class'),
            'source_frame': LaunchConfiguration('source_frame'),
            'target_frame': LaunchConfiguration('target_frame')
        }],
        remappings=[
            ('/input_detection_3d', '/detections_3d')
        ],
        condition=UnlessCondition(LaunchConfiguration('fake_3d'))
    )
    
    # YOLO to standard converter node
    yolo_to_standard_node = Node(
        package='simple_perception',
        executable='yolo_to_standard',
        name='yolo_to_standard_node',
        output='screen',
        remappings=[
            ('input_detection_3d', '/yolo/detections_3d'),
            ('input_detection_2d', '/yolo/detections'),
            ('output_detection_3d', '/detections_3d'),
            ('output_detection_2d', '/detections_2d')
        ]
    )
    
    # YOLO detector launch
    yolo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('yolo_bringup'),
                'launch',
                'yolo.launch.py'
            )
        ),
        launch_arguments={
            'input_image_topic': LaunchConfiguration('input_image_topic'),
            'input_depth_topic': LaunchConfiguration('input_depth_topic'),
            'input_depth_info_topic': LaunchConfiguration('input_depth_info_topic'),
            'target_frame': LaunchConfiguration('camera_frame'),
            'depth_image_reliability': '1'
        }.items()
    )
    
    return LaunchDescription([
        # Arguments
        fake_3d_arg,
        target_class_arg,
        source_frame_arg,
        target_frame_arg,
        optical_frame_arg,
        input_image_topic_arg,
        input_depth_topic_arg,
        input_depth_info_topic_arg,
        camera_frame_arg,
        
        # Nodes
        entity_tracker_fake_3d_node,  # Launched when fake_3d=true
        entity_tracker_3d_node,  # Launched when fake_3d=false
        yolo_to_standard_node,
        yolo_launch,
    ])
