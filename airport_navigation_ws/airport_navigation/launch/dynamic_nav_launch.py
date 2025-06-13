#!/usr/bin/env python3
# dynamic_nav_launch.py

import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pkg_dir = get_package_share_directory('airport_navigation')
    config_path = os.path.join(pkg_dir, 'config', 'navigation_params.yaml')
    rviz_config_path = os.path.join(pkg_dir, 'config', 'airport_nav.rviz')

    return LaunchDescription([
        # GNSS Source Node
        Node(
            package='airport_navigation',
            executable='gnss_source_node',
            name='gnss_source_node',
            output='screen',
            parameters=[config_path]
        ),

        # Destination Node
        Node(
            package='airport_navigation',
            executable='destination_node',
            name='destination_node',
            output='screen',
            parameters=[config_path]
        ),

        # Navigation Node
        Node(
            package='airport_navigation',
            executable='navigation_node',
            name='navigation_node',
            output='screen',
            parameters=[config_path]
        ),

        # Route Manager Node
        Node(
            package='airport_navigation',
            executable='route_manager_node',
            name='route_manager_node',
            output='screen',
            parameters=[config_path]
        ),

        # RViz Debug Subscriber Node
        Node(
            package='airport_navigation',
            executable='rviz_debug_node',
            name='rviz_debug_node',
            output='screen'
        ),

        # Diagnostics Node
        Node(
            package='airport_navigation',
            executable='diagnostics_node',
            name='diagnostics_node',
            output='screen'
        ),

        # RViz Visualization
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', rviz_config_path]
        )
    ])
