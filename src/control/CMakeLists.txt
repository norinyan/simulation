cmake_minimum_required(VERSION 3.8)
project(control)

find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(gazebo_dev REQUIRED)
find_package(gazebo_ros REQUIRED)
find_package(interfaces REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(yaml-cpp REQUIRED)

add_library(ideal_trajectory_control SHARED src/ideal_control.cpp)
target_compile_features(ideal_trajectory_control PUBLIC cxx_std_17)
target_compile_options(ideal_trajectory_control PRIVATE -Wall -Wextra -Wpedantic)
target_include_directories(ideal_trajectory_control PRIVATE ${GAZEBO_INCLUDE_DIRS})
target_link_directories(ideal_trajectory_control PRIVATE ${GAZEBO_LIBRARY_DIRS})
target_link_libraries(ideal_trajectory_control ${GAZEBO_LIBRARIES} yaml-cpp)
ament_target_dependencies(ideal_trajectory_control
  ament_index_cpp gazebo_ros interfaces rclcpp geometry_msgs nav_msgs
  trajectory_msgs
)

install(TARGETS ideal_trajectory_control LIBRARY DESTINATION lib)

install(DIRECTORY urdf config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
