cmake_minimum_required(VERSION 3.8)
project(global_planning)

find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(rclcpp REQUIRED)
find_package(interfaces REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(yaml-cpp REQUIRED)

include(pct/pct.cmake)

add_executable(global_node global_node.cpp pct/pct_planner.cpp pct/pct_core.cpp)
target_compile_features(global_node PRIVATE cxx_std_17)
target_compile_options(global_node PRIVATE -Wall -Wextra -Wpedantic)
target_link_libraries(global_node yaml-cpp pct_backend)
ament_target_dependencies(global_node
  ament_index_cpp rclcpp interfaces geometry_msgs nav_msgs std_msgs std_srvs)
install(TARGETS global_node DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})
install(PROGRAMS pct/map_processing/prepare_map.py
  DESTINATION lib/${PROJECT_NAME} RENAME pct_prepare_map)
install(FILES pct/map_processing/tomogram.py pct/map_processing/kernels.py
  DESTINATION lib/${PROJECT_NAME})
install(FILES pct/third_party/LICENSE pct/third_party/NOTICE pct/third_party/UPSTREAM
  pct/third_party/ADAPTATIONS.md DESTINATION share/${PROJECT_NAME}/pct)
install(FILES pct/third_party/osqp/LICENSE pct/third_party/osqp/NOTICE
  DESTINATION share/${PROJECT_NAME}/pct/osqp)
install(FILES pct/third_party/src/common/smoothing/LICENSE
  DESTINATION share/${PROJECT_NAME}/pct/smoothing)
ament_package()
