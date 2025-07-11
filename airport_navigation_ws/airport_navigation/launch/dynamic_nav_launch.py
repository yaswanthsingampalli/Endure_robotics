import os
import threading
import time
import socket
import subprocess
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def is_port_in_use(port):
    result = subprocess.run(['lsof', '-ti', f'tcp:{port}'], capture_output=True, text=True)
    return bool(result.stdout.strip())

def wait_for_servers_and_open_browser():
    def is_port_open(host, port):
        try:
            with socket.create_connection((host, port), timeout=1):
                return True
        except Exception:
            return False

    print("🕒 Waiting for rosbridge (9090) and web UI (8080)...")
    for _ in range(30):
        if is_port_open("localhost", 9090) and is_port_open("localhost", 8080):
            print("✅ Both servers are up! Launching browser...")
            time.sleep(1)
            os.system("xdg-open http://localhost:8080")
            return
        time.sleep(1)
    print("⚠️ Timeout: servers not ready.")

def create_node(package, executable, name=None, namespace=None, parameters=None, extra_args=None):
    return Node(
        package=package,
        executable=executable,
        name=name or executable,
        namespace=namespace,
        output='screen',
        parameters=parameters or [],
        arguments=extra_args or [],
    )

def static_transform_node():
    return Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_map_tf',
        arguments=['0', '0', '0', '0', '0', '0', 'map', 'base_link'],
        output='screen'
    )

def generate_launch_description():
    # Start web UI on port 8080 only if not already running
    web_ui_path = os.path.join(os.path.expanduser('~'), 'airport_navigation_ws', 'web_ui')
    if not os.path.exists(web_ui_path):
        print(f"❌ ERROR: web_ui folder not found at: {web_ui_path}")
    elif not is_port_in_use(8080):
        print("🧠 Port 8080 not in use, starting web UI server...")
        threading.Thread(
            target=lambda: subprocess.Popen(['python3', '-m', 'http.server', '8080'], cwd=web_ui_path),
            daemon=True
        ).start()
        threading.Thread(target=wait_for_servers_and_open_browser, daemon=True).start()
    else:
        print("⚠️ Port 8080 already in use. Skipping web UI server start.")

    pkg_dir = get_package_share_directory('airport_navigation')
    config_file = os.path.join(pkg_dir, 'config', 'navigation_params.yaml')
    rviz_config = os.path.join(pkg_dir, 'config', 'airport_nav.rviz')

    nav_nodes = ['gnss_source_node']

    return LaunchDescription([
        create_node('airport_navigation', 'dummy_origin_publisher'),
        *[create_node('airport_navigation', node, parameters=[config_file]) for node in nav_nodes],
        create_node('airport_navigation', 'destination_node', namespace='destination_node', parameters=[config_file]),
        create_node('airport_navigation', 'route_manager_node', namespace='route_manager_node', parameters=[config_file]),
        create_node('airport_navigation', 'pose_publisher_node'),
        create_node('airport_navigation', 'diagnostics_node'),
        create_node('airport_navigation', 'marker_viz_node'),
        create_node('rosbridge_server', 'rosbridge_websocket', parameters=[
            {'default_call_service_timeout': 5.0},
            {'call_services_in_new_thread': True}
        ]),
        static_transform_node(),
        create_node('rviz2', 'rviz2', extra_args=['-d', rviz_config]),
    ])
