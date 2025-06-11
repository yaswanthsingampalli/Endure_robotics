#!/usr/bin/env python3
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'origin_lat',
            default_value='37.6188056',
            description='Origin latitude (WGS84)'
        ),
        DeclareLaunchArgument(
            'origin_lon',
            default_value='-122.3754167',
            description='Origin longitude (WGS84)'
        ),
        DeclareLaunchArgument(
            'origin_alt',
            default_value='4.0',
            description='Origin altitude (meters)'
        ),
        DeclareLaunchArgument(
            'utm_zone',
            default_value='10',
            description='UTM zone for projection'
        ),

        Node(
            package='airport_navigation',
            executable='gnss_source_node',
            name='gnss_source_node',
            output='screen',
            parameters=[{
                'origin_lat': LaunchConfiguration('origin_lat'),
                'origin_lon': LaunchConfiguration('origin_lon'),
                'origin_alt': LaunchConfiguration('origin_alt'),
                'utm_zone': LaunchConfiguration('utm_zone'),
            }],
        ),

        Node(
            package='airport_navigation',
            executable='destination_node',
            name='destination_node',
            output='screen',
        ),

        Node(
            package='airport_navigation',
            executable='navigation_node',
            name='navigation_node',
            output='screen',
        ),
    ])
