cmake_minimum_required(VERSION 3.8)
project(interfaces)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/GlobalPath.msg"
  "msg/LocalTrajectory.msg"
  DEPENDENCIES std_msgs geometry_msgs trajectory_msgs
)

ament_export_dependencies(rosidl_default_runtime)

ament_package()
