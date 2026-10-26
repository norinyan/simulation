cmake_minimum_required(VERSION 3.8)
project(local_planning)

if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  set(CMAKE_BUILD_TYPE Release CACHE STRING "Build type" FORCE)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(rclcpp REQUIRED)
find_package(interfaces REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(tinyxml2_vendor REQUIRED)
find_package(TinyXML2 REQUIRED)

add_executable(local_node local_node.cpp
  scan/scan_planner.cpp scan/scan_map.cpp
  scan/core/bspline_optimizer.cpp scan/core/uniform_bspline.cpp scan/core/dyn_a_star.cpp)
target_include_directories(local_node PRIVATE scan/core)
target_compile_features(local_node PRIVATE cxx_std_17)
target_compile_options(local_node PRIVATE -Wall -Wextra -Wpedantic)
target_link_libraries(local_node yaml-cpp Eigen3::Eigen tinyxml2::tinyxml2)
ament_target_dependencies(local_node
  ament_index_cpp rclcpp interfaces nav_msgs geometry_msgs trajectory_msgs)

install(TARGETS local_node DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})
install(FILES scan/core/LICENSE scan/core/NOTICE DESTINATION share/${PROJECT_NAME}/scan)
ament_package()
