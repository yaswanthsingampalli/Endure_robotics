cmake_minimum_required(VERSION 3.8)
project(airport_navigation_interfaces)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(geometry_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/CustomWaypoint.msg"
  "msg/GNSSOrigin.msg"        
  "srv/SetDestination.srv"
  "srv/SetRoute.srv"
  DEPENDENCIES builtin_interfaces geometry_msgs
)

ament_package()
