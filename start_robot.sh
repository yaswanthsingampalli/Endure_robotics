#!/bin/bash
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash

# Optional: kill any process using the webcam
fuser -k /dev/video0

# Start the ROS 2 launch file
ros2 launch object_detection_test test_detection.launch.py

