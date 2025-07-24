#start_server.sh

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

# Start UI server with Gunicorn (1 worker)
echo "[INFO] Starting UI Server on http://0.0.0.0:$PORT with Gunicorn..."
cd ~/robot_ui_launcher_ws
#nohup gunicorn -w 1 -k gevent -b 0.0.0.0:$PORT ui_server:app > logs/ui_server.log 2>&1 &
nohup gunicorn -w 1 -k gevent -t 120 -b 0.0.0.0:$PORT ui_server:app > logs/ui_server.log 2>&1 &


sleep 2  # Give Gunicorn a moment to start

echo "[INFO] Opening browser to http://localhost:$PORT ..."
xdg-open "http://localhost:$PORT" || echo "[WARNING] Couldn't open browser automatically."

echo "[INFO] ✅ Server is running. Press Ctrl+C to exit this script monitor."
wait
