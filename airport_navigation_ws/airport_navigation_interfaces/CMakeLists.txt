cmake_minimum_required(VERSION 3.8)
project(airport_navigation_interfaces)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/SetDestination.srv"
  "srv/SetRoute.srv"  
 # "msg/GNSSOrigin.msg"
)


ament_package()

