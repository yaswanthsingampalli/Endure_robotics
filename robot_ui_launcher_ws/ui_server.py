from flask import Flask, render_template, jsonify
import subprocess
import signal
import os

app = Flask(__name__)
ros_process = None
ros_log_file = None

os.makedirs('logs', exist_ok=True)

@app.route('/')
def index():
    return render_template('index.html')

@app.route('/start')
def start_launch():
    global ros_process, ros_log_file
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
