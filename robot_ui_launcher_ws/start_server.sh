#!/bin/bash

PORT=8081

# Check if port is in use, kill the process if found
PID=$(lsof -ti tcp:$PORT)
if [ ! -z "$PID" ]; then
  echo "[INFO] Port $PORT is in use by PID $PID. Killing process..."
  kill -9 $PID
  sleep 1
fi

# Build workspaces
echo "[INFO] Building robot_ui_launcher_ws..."
cd ~/robot_ui_launcher_ws
colcon build --symlink-install

echo "[INFO] Building airport_navigation_ws..."
cd ~/airport_navigation_ws
colcon build --symlink-install

# Source environment
source ~/airport_navigation_ws/install/setup.bash

# Start UI server
echo "[INFO] Starting UI Server on port $PORT..."
python3 ~/robot_ui_launcher_ws/ui_server.py
