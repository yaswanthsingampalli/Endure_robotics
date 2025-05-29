from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='object_detection_test',
            executable='fake_camera_publisher',
            name='fake_camera_publisher',
            output='screen'
        ),
        Node(
            package='object_detection_test',
            executable='object_detector',
            name='object_detector',
            output='screen'
        )
    ])
