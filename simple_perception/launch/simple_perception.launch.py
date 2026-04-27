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
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python import get_package_share_directory
from launch.substitutions import PythonExpression


def generate_launch_description():
    """
    Main launch file for the simple_perception stack.

    This launch file provides a flexible perception pipeline for robots, supporting both real 3D detections (depth sensors) and simulated 3D detections (monocular depth + 2D detections).

    Main arguments:
    - use_monocular: (bool) If true, launches the monocular depth + 2D->3D simulation pipeline (Depth Anything + detection_3d_simulator). If false, uses real 3D detections.
    - detection_3d_topic: (str) Input Detection3DArray topic for entity_tracker (default: /detections_3d, or /simulated_detection_3d if use_monocular).
    - fake_3d: (bool) Use simulated 3D tracker (true) or real 3D tracker (false). (legacy, usually true)
    - target_class: (str) Target class to track (default: person)
    - source_frame: (str) Source frame for tracking (default: base_link)
    - target_frame: (str) Target frame name for TF broadcast (default: target)
    - optical_frame: (str) Camera optical frame (default: CameraTop_optical_frame)
    - input_image_topic: (str) Input image topic for YOLO (default: /image_rgb)
    - input_depth_topic: (str) Input depth topic for YOLO (default: /depth_anything/depth)
    - input_depth_info_topic: (str) Input camera info topic for YOLO (default: /camera_rgb_info)
    - camera_frame: (str) Camera frame for YOLO (default: CameraTop_frame)

    Usage:
    - ros2 launch simple_perception simple_perception.launch.py [args]
    - If use_monocular is true, launches the monocular pipeline and entity_tracker automatically connects to /simulated_detection_3d.
    - If use_monocular is false, entity_tracker connects to /detections_3d (or the value of detection_3d_topic).
    - Depth calibration parameters are only in monocolular_depth.launch.py.

    All arguments can be remapped from the command line.
    """
 
    # --- Argument: Use monocular depth pipeline ---
    use_monocular_arg = DeclareLaunchArgument(
        'use_monocular',
        default_value='false',
        description='If true, launch monocular depth + 2D->3D simulation pipeline'
    )

    # --- Optional: Monocular depth + 2D->3D simulation pipeline ---
    # This pipeline is included only if use_monocular is true
    monocular_pipeline = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('simple_perception'),
                'launch',
                'monocolular_depth.launch.py'
            )
        ),
        condition=IfCondition(LaunchConfiguration('use_monocular')),
        launch_arguments={
            # You can pass arguments here if needed
        }.items()
    )

    # --- Argument: Output Detection3D topic (remappable) ---
    # Dynamically selects the topic based on use_monocular
    detection_3d_topic_arg = DeclareLaunchArgument(
        'detection_3d_topic',
        default_value=PythonExpression([
            "'/simulated_detection_3d' if '", # Added opening quote
            LaunchConfiguration('use_monocular'),
            "' == 'true' else '/detections_3d'" # Added closing quote
        ]),
        description='Input Detection3DArray topic for entity_tracker'
    )
    
    # --- Other launch arguments ---
    fake_3d_arg = DeclareLaunchArgument(
        'fake_3d',
        default_value='true',
        description='Use fake 3D tracker (true) or real 3D tracker (false)'
    )

    target_class_arg = DeclareLaunchArgument(
        'target_class',
        default_value='person',
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
        default_value='/depth_anything/depth',
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

    
    # --- Node: Entity tracker ---
    # Tracks entities using Detection3DArray input
    entity_tracker_node = Node(
        package='simple_perception',
        executable='entity_tracker',
        name='entity_tracker_node',
        output='screen',
        parameters=[{
            'target_class': LaunchConfiguration('target_class'),
            'source_frame': LaunchConfiguration('source_frame'),
            'target_frame': LaunchConfiguration('target_frame'),
        }],
        remappings=[
            ('input_detection_3d', LaunchConfiguration('detection_3d_topic')),
        ]
    )

    # --- Node: YOLO to standard converter ---
    # Converts YOLO outputs to standard Detection2D/3D topics
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

    # --- Include: YOLO detector launch ---
    # Launches YOLO detector with remapped topics and parameters
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
            'use_3d': 'True', # Set to False when using custom 3D processing
            'depth_image_reliability': '1', # Set to 2 when monocular depth is used
        }.items()
    )
    
    # --- LaunchDescription: Add all arguments, pipelines, and nodes ---
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
        use_monocular_arg,
        detection_3d_topic_arg,

        # Monocular pipeline (optional)
        monocular_pipeline,

        # Nodes
        entity_tracker_node,
        yolo_to_standard_node,
        yolo_launch,
    ])
