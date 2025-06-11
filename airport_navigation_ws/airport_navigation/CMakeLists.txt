cmake_minimum_required(VERSION 3.5)
project(airport_navigation)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(airport_navigation_interfaces REQUIRED)  # NEW

# Install Python package
ament_python_install_package(${PROJECT_NAME})

# Install scripts
install(PROGRAMS
  airport_navigation/gnss_source_node.py
  airport_navigation/destination_node.py
  airport_navigation/navigation_node.py
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch files
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
