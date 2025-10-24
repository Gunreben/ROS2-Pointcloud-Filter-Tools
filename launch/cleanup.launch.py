#!/usr/bin/env python3
"""
Cleanup Launch File for Point Cloud Filtering

This launch file chains two filters:
1. zed_alpha_filter_node - Filters points based on alpha channel
2. outlier_filter_node - Removes outliers using voxel-based filtering

Input: /zed/zed_node/point_cloud/cloud_registered
Output: /zed/zed_node/point_cloud/cloud_registered/filtered
"""

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        # Stage 1: Alpha channel filter
        Node(
            package='lidar_filter',
            executable='zed_alpha_filter_node',
            name='alpha_filter',
            parameters=[{
                'input_topic': '/zed/zed_node/point_cloud/cloud_registered',
                'output_topic': '/zed/point_cloud/alpha_filtered',
                'alpha_threshold': 255,
            }],
            output='screen',
        ),
        
        # Stage 2: Outlier filter
        Node(
            package='lidar_filter',
            executable='outlier_filter_node',
            name='outlier_filter',
            parameters=[{
                'input_topic': '/zed/point_cloud/alpha_filtered',
                'output_topic': '/zed/zed_node/point_cloud/cloud_registered/filtered',
                'voxel_size': 0.2,
                'min_points_per_voxel': 2,
                'max_valid_range': 100.0,
                'enable_voxel_filter': True,
            }],
            output='screen',
        ),
    ])

