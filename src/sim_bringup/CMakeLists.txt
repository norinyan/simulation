cmake_minimum_required(VERSION 3.8)
project(sim_bringup)

find_package(ament_cmake REQUIRED)

install(DIRECTORY launch worlds config maps
  DESTINATION share/${PROJECT_NAME}
)
install(PROGRAMS scripts/dynamic_obstacles.py scripts/prepare_scene.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
