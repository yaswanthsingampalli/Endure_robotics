#ui_server.py
from flask import Flask, render_template, jsonify, Response
import subprocess
import signal
import os
import socket
import logging
from threading import Lock
from urllib.request import urlopen
from urllib.error import URLError

# Configure logging
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(levelname)s - %(message)s',
    handlers=[
        logging.FileHandler('logs/ui_server.log'),
        logging.StreamHandler()
    ]
)
logger = logging.getLogger(__name__)

app = Flask(__name__)

# Thread-safe process management
class ProcessManager:
    def __init__(self):
        self.ros_process = None
        self.ros_log_file = None
        self.lock = Lock()
        os.makedirs('logs', exist_ok=True)
    
    def kill_processes_on_ports(self, ports):
        """Kill processes using specified ports"""
        for port in ports:
            try:
                result = subprocess.run(['lsof', '-ti', f'tcp:{port}'], 
                                      capture_output=True, text=True)
                pids = result.stdout.strip().split('\n')
                for pid in pids:
                    if pid:
                        logger.info(f"Killing process {pid} on port {port}")
                        os.kill(int(pid), signal.SIGKILL)
            except Exception as e:
                logger.warning(f"Could not kill processes on port {port}: {e}")
    
    def get_local_ip(self):
        """Get local IP address"""
        try:
            s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            s.connect(("10.255.255.255", 1))
            IP = s.getsockname()[0]
        except Exception:
            IP = "127.0.0.1"
        finally:
            s.close()
        return IP
    
    def is_process_running(self, process):
        """Check if process is still running"""
        if process is None:
            return False
        return process.poll() is None

process_manager = ProcessManager()

@app.route('/')
def index():
    return render_template('index.html')

@app.route('/start')
def start_launch():
    with process_manager.lock:
        logger.info("Starting mission...")
        
        # Kill conflicting processes
        process_manager.kill_processes_on_ports([8080, 9090])
        
        # Check if already running
        if process_manager.is_process_running(process_manager.ros_process):
            return jsonify({"status": "Already running"})
        
        try:
            # Open log file
            process_manager.ros_log_file = open('logs/ros_output.log', 'w')
            
            # Start ROS process
            process_manager.ros_process = subprocess.Popen(
                ['ros2', 'launch', 'airport_navigation', 'dynamic_nav_launch.py'],
                stdout=process_manager.ros_log_file,
                stderr=subprocess.STDOUT
            )
            
            logger.info("Mission started successfully")
            return jsonify({"status": "Mission launched"})
            
        except Exception as e:
            logger.error(f"Failed to start mission: {e}")
            return jsonify({"status": f"Error: {str(e)}"}), 500

@app.route('/stop')
def stop_launch():
    with process_manager.lock:
        logger.info("Stopping mission...")
        
        if not process_manager.is_process_running(process_manager.ros_process):
            return jsonify({"status": "Not running"})
        
        try:
            # Send SIGINT for graceful shutdown
            process_manager.ros_process.send_signal(signal.SIGINT)
            
            # Wait for process to terminate (with timeout)
            try:
                process_manager.ros_process.wait(timeout=10)
            except subprocess.TimeoutExpired:
                logger.warning("Process didn't terminate gracefully, forcing kill")
                process_manager.ros_process.kill()
                process_manager.ros_process.wait()
            
            # Close log file
            if process_manager.ros_log_file:
                process_manager.ros_log_file.close()
                process_manager.ros_log_file = None
            
            logger.info("Mission stopped successfully")
            return jsonify({"status": "Mission stopped"})
            
        except Exception as e:
            logger.error(f"Error stopping mission: {e}")
            return jsonify({"status": f"Error: {str(e)}"}), 500

@app.route('/restart')
def restart_launch():
    logger.info("Restarting mission...")
    stop_result = stop_launch()
    start_result = start_launch()
    return start_result

@app.route('/manual')
def manual_control():
    with process_manager.lock:
        logger.info("Switching to manual control...")
        
        # Stop any running mission
        if process_manager.is_process_running(process_manager.ros_process):
            try:
                process_manager.ros_process.send_signal(signal.SIGINT)
                process_manager.ros_process.wait(timeout=5)
            except:
                process_manager.ros_process.kill()
                process_manager.ros_process.wait()
            
            if process_manager.ros_log_file:
                process_manager.ros_log_file.close()
                process_manager.ros_log_file = None
        
        try:
            # Start manual control
            process_manager.ros_log_file = open('logs/ros_output.log', 'w')
            process_manager.ros_process = subprocess.Popen(
                ['ros2', 'launch', 'robot_controls', 'manual_control_launch.py'],
                stdout=process_manager.ros_log_file,
                stderr=subprocess.STDOUT
            )
            
            logger.info("Manual control started")
            return render_template('manual_control.html')
            
        except Exception as e:
            logger.error(f"Failed to start manual control: {e}")
            return jsonify({"error": str(e)}), 500

@app.route('/stream')
def stream_page():
    jetson_ip = process_manager.get_local_ip()
    return render_template('stream.html', jetson_ip=jetson_ip)

@app.route('/get_local_ip')
def get_local_ip_endpoint():
    ip = process_manager.get_local_ip()
    return jsonify({"ip": ip})

@app.route('/video_feed')
def video_feed():
    """Video streaming route. Proxy to the MJPEG server."""
    def generate():
        try:
            # Try to connect to the MJPEG stream server
            stream = urlopen('http://localhost:9999/video_feed', timeout=10)
            while True:
                chunk = stream.read(8192)  # Read larger chunks
                if not chunk:
                    break
                yield chunk
        except URLError as e:
            logger.error(f"Video feed connection error: {e}")
            # Return a simple error response
            yield b'--FRAME\r\nContent-Type: text/plain\r\n\r\nStream not available\r\n'
        except Exception as e:
            logger.error(f"Video feed error: {e}")
            yield b'--FRAME\r\nContent-Type: text/plain\r\n\r\nError occurred\r\n'
    
    return Response(generate(), mimetype='multipart/x-mixed-replace; boundary=FRAME')

@app.route('/health')
def health_check():
    """Health check endpoint"""
    status = {
        "ui_server": "healthy",
        "ros_process": "running" if process_manager.is_process_running(process_manager.ros_process) else "stopped"
    }
    return jsonify(status)

if __name__ == '__main__':
    app.run(host='0.0.0.0', port=8081, threaded=True)