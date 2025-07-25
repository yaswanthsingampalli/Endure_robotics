#start_server.sh
#!/bin/bash

# Configuration
PORT=8081
LOG_DIR=~/robot_ui_launcher_ws/logs

# Create log directory
mkdir -p $LOG_DIR

# Function to kill processes on specific ports
kill_port_processes() {
    local port=$1
    local pids=$(lsof -ti tcp:$port 2>/dev/null)
    if [ ! -z "$pids" ]; then
        echo "[INFO] Killing processes on port $port: $pids"
        kill -9 $pids 2>/dev/null
        sleep 1
    fi
}

# Kill existing processes
kill_port_processes $PORT
kill_port_processes 9999  # MJPEG streaming

# Build workspaces with error handling (building all packages)
echo "[INFO] Building robot_ui_launcher_ws..."
cd ~/robot_ui_launcher_ws
if ! colcon build --symlink-install; then
    echo "[ERROR] Failed to build robot_ui_launcher_ws"
    exit 1
fi

echo "[INFO] Building airport_navigation_ws (all packages)..."
cd ~/airport_navigation_ws
if ! colcon build --symlink-install; then
    echo "[ERROR] Failed to build airport_navigation_ws"
    exit 1
fi

# Source environments
source ~/airport_navigation_ws/install/setup.bash

# Start streaming server with restart capability
echo "[INFO] Starting MJPEG streaming server..."
nohup python3 ~/robot_ui_launcher_ws/stream_launcher.py > $LOG_DIR/janus_stream.log 2>&1 &
STREAM_PID=$!

# Wait for streaming to start
sleep 3

# Check if streaming started successfully
if ! kill -0 $STREAM_PID 2>/dev/null; then
    echo "[ERROR] Streaming server failed to start"
    exit 1
fi

# Start UI server with multiple workers for production
echo "[INFO] Starting UI Server on http://0.0.0.0:$PORT with Gunicorn..."
cd ~/robot_ui_launcher_ws
nohup gunicorn -w 2 -b 0.0.0.0:$PORT --timeout 120 --keep-alive 5 ui_server:app > $LOG_DIR/ui_server.log 2>&1 &

sleep 3

# Get local IP for user information
LOCAL_IP=$(hostname -I | awk '{print $1}')

# Function to open browser with multiple fallback methods
open_browser() {
    local url=$1
    
    echo "[INFO] Attempting to open browser to $url ..."
    
    # Try multiple browser opening methods
    if command -v xdg-open > /dev/null 2>&1; then
        xdg-open "$url" && return 0
    fi
    
    if command -v gnome-open > /dev/null 2>&1; then
        gnome-open "$url" && return 0
    fi
    
    if command -v open > /dev/null 2>&1; then
        open "$url" && return 0
    fi
    
    if command -v firefox > /dev/null 2>&1; then
        firefox "$url" && return 0
    fi
    
    if command -v google-chrome > /dev/null 2>&1; then
        google-chrome "$url" && return 0
    fi
    
    if command -v chromium-browser > /dev/null 2>&1; then
        chromium-browser "$url" && return 0
    fi
    
    # If all methods fail, just print the URL
    echo "[WARNING] Could not open browser automatically."
    echo "[INFO] Please open your browser and go to: $url"
    return 1
}

# Open browser
open_browser "http://localhost:$PORT" || open_browser "http://$LOCAL_IP:$PORT"

echo "[INFO] Server available at http://$LOCAL_IP:$PORT or http://localhost:$PORT"
echo "[INFO] Streaming server PID: $STREAM_PID"
echo "[INFO] Video stream available at http://$LOCAL_IP:$PORT/video_feed"

echo "[INFO] ✅ Server is running. Press Ctrl+C to exit."
wait