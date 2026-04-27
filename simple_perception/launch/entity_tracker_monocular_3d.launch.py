#!/usr/bin/env python3
# Launch file for entity_tracker_monocular_3d

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('target_class', default_value='person', description='Object class to track'),
        DeclareLaunchArgument('source_frame', default_value='base_link', description='Robot base frame'),
        DeclareLaunchArgument('target_frame', default_value='target', description='Published TF frame for tracked object'),
        DeclareLaunchArgument('optical_frame', default_value='CameraTop_optical_frame', description='Camera optical frame'),
        DeclareLaunchArgument('detection_topic', default_value='/detections_2d', description='Input Detection2DArray topic'),
        DeclareLaunchArgument('depth_topic', default_value='/depth_anything/depth', description='Input depth image topic'),
        DeclareLaunchArgument('camera_info_topic', default_value='/camera_info', description='Input camera info topic'),
        DeclareLaunchArgument('depth_scale_factor', default_value='1.0', description='Manual depth scaling (method B)'),
        DeclareLaunchArgument('reference_distance', default_value='1.0', description='Known distance for calibration (meters)'),
        DeclareLaunchArgument('reference_depth_value', default_value='1.0', description='Depth value at reference distance'),
        DeclareLaunchArgument('min_depth', default_value='0.3', description='Minimum valid depth (meters)'),
        DeclareLaunchArgument('max_depth', default_value='10.0', description='Maximum valid depth (meters)'),
        DeclareLaunchArgument('use_bbox_center', default_value='True', description='Sample center vs median of bbox'),

        Node(
            package='simple_perception',
            executable='entity_tracker_monocular_3d',
            name='entity_tracker_monocular_3d_node',
            output='screen',
            parameters=[{
                'target_class': LaunchConfiguration('target_class'),
                'source_frame': LaunchConfiguration('source_frame'),
                'target_frame': LaunchConfiguration('target_frame'),
                'optical_frame': LaunchConfiguration('optical_frame'),
                'depth_scale_factor': LaunchConfiguration('depth_scale_factor'),
                'reference_distance': LaunchConfiguration('reference_distance'),
                'reference_depth_value': LaunchConfiguration('reference_depth_value'),
                'min_depth': LaunchConfiguration('min_depth'),
                'max_depth': LaunchConfiguration('max_depth'),
                'use_bbox_center': LaunchConfiguration('use_bbox_center'),
            }],
            remappings=[
                ('input_detection_2d', LaunchConfiguration('detection_topic')),
                ('depth_image', LaunchConfiguration('depth_topic')),
                ('camera_info', LaunchConfiguration('camera_info_topic')),
            ]
        )
    ])
