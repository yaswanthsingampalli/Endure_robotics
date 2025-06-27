# manual_control_launch.py
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='teleop_twist_keyboard',
            executable='teleop_twist_keyboard',
            name='keyboard_teleop',
            output='screen',
            prefix='xterm -e',  # Opens in terminal window
            remappings=[('/cmd_vel', '/cmd_vel')],  # Update topic if your robot uses something else
        ),
    ])

