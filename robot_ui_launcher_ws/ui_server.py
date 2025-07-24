# ui_server.py
from flask import Flask, render_template, jsonify, Response
import subprocess
import signal
import os
from threading import Thread
import zed_mjpeg_stream

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
                    print(f"\U0001F480 Killing process {pid} on port {port}")
                    os.kill(int(pid), signal.SIGKILL)
        except Exception as e:
            print(f"\u26A0\ufe0f Could not kill processes on port {port}: {e}")

@app.route('/')
def index():
    return render_template('index.html')

@app.route('/start')
def start_launch():
    global ros_process, ros_log_file

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

    if ros_process and ros_process.poll() is None:
        ros_process.send_signal(signal.SIGINT)
        ros_process.wait()
        if ros_log_file:
            ros_log_file.close()
            ros_log_file = None

    ros_log_file = open('logs/ros_output.log', 'w')
    ros_process = subprocess.Popen(
        ['ros2', 'launch', 'robot_controls', 'manual_control_launch.py'],
        stdout=ros_log_file,
        stderr=subprocess.STDOUT
    )

    return render_template('manual_control.html')

@app.route('/stream')
def stream_page():
    try:
        from zed_mjpeg_stream import start_zed_stream
        start_zed_stream()  # Always ensure it’s started
        return render_template('stream.html')
    except Exception as e:
        print(f"⚠️ Stream page failed to load: {e}")
        return jsonify({"error": "Failed to load stream page."}), 500


@app.route('/video_feed')
def video_feed():
    from zed_mjpeg_stream import generate_mjpeg
    return Response(generate_mjpeg(),
                    mimetype='multipart/x-mixed-replace; boundary=frame')

if __name__ == '__main__':
    app.run(host='0.0.0.0', port=8081)

