# manual_control_launch.py
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Teleop keyboard node
        Node(
            package='teleop_twist_keyboard',
            executable='teleop_twist_keyboard',
            name='keyboard_teleop',
            output='screen',
            prefix='xterm -e',
            remappings=[('/cmd_vel', '/cmd_vel')],
        ),

        # Add rosbridge websocket server here
        Node(
            package='rosbridge_server',
            executable='rosbridge_websocket',
            name='rosbridge_websocket',
            output='screen',
            parameters=[
                {'default_call_service_timeout': 5.0},
                {'call_services_in_new_thread': True}
            ],
        ),
    ])

