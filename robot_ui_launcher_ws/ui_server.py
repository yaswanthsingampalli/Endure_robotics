#ui_server.py
from flask import Flask, render_template, jsonify
import subprocess
import signal
import os

app = Flask(__name__)
ros_process = None
ros_log_file = None

os.makedirs('logs', exist_ok=True)

def kill_processes_on_ports(ports):
    for port in ports:
        try:
            result = subprocess.run(['lsof', '-ti', f'tcp:{port}'], capture_output=True, text=True)
            pids = result.stdout.strip().split('\n')
            for pid in pids:
                if pid:
                    print(f"💀 Killing process {pid} on port {port}")
                    os.kill(int(pid), signal.SIGKILL)
        except Exception as e:
            print(f"⚠️ Could not kill processes on port {port}: {e}")

@app.route('/')
def index():
    return render_template('index.html')

@app.route('/start')
def start_launch():
    global ros_process, ros_log_file

    # Kill processes on ports 8080 and 9090 before launching
    kill_processes_on_ports([8080, 9090])

    if ros_process is None or ros_process.poll() is not None:
        ros_log_file = open('logs/ros_output.log', 'w')
        ros_process = subprocess.Popen(
            ['ros2', 'launch', 'airport_navigation', 'dynamic_nav_launch.py'],
            stdout=ros_log_file,
            stderr=subprocess.STDOUT
        )
        return jsonify({"status": "Mission launched"})
    return jsonify({"status": "Already running"})

@app.route('/stop')
def stop_launch():
    global ros_process, ros_log_file
    if ros_process and ros_process.poll() is None:
        ros_process.send_signal(signal.SIGINT)
        ros_process.wait()
        if ros_log_file:
            ros_log_file.close()
            ros_log_file = None
        return jsonify({"status": "Mission stopped"})
    return jsonify({"status": "Not running"})

@app.route('/restart')
def restart_launch():
    stop_launch()
    return start_launch()

@app.route('/manual')
def manual_control():
    global ros_process, ros_log_file

    # Stop existing ROS process, but do NOT kill port 9090 (rosbridge)
    if ros_process and ros_process.poll() is None:
        ros_process.send_signal(signal.SIGINT)
        ros_process.wait()
        if ros_log_file:
            ros_log_file.close()
            ros_log_file = None

    # Launch manual control (teleop_twist_keyboard) WITHOUT killing rosbridge on 9090
    ros_log_file = open('logs/ros_output.log', 'w')
    ros_process = subprocess.Popen(
        ['ros2', 'launch', 'robot_controls', 'manual_control_launch.py'],
        stdout=ros_log_file,
        stderr=subprocess.STDOUT
    )

    return jsonify({"status": "Manual control enabled"})
 
@app.route('/manual_control')
def manual_control_page():
    return render_template('manual_control.html')

if __name__ == '__main__':
    app.run(host='0.0.0.0', port=8081)

