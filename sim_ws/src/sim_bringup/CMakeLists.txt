cmake_minimum_required(VERSION 3.8)
project(sim_bringup)

find_package(ament_cmake REQUIRED)
find_package(planning REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(yaml_cpp_vendor REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(gazebo_ros REQUIRED)
find_package(PkgConfig REQUIRED)
pkg_check_modules(GAZEBO REQUIRED IMPORTED_TARGET gazebo)
find_package(Qt5 REQUIRED COMPONENTS Widgets)
pkg_check_modules(OGRE REQUIRED OGRE)
find_path(OGRE_PAGING_INCLUDE_DIR OgrePagedWorldSection.h HINTS ${OGRE_INCLUDE_DIRS} PATH_SUFFIXES Paging REQUIRED)
find_path(OGRE_TERRAIN_INCLUDE_DIR OgreTerrain.h HINTS ${OGRE_INCLUDE_DIRS} PATH_SUFFIXES Terrain REQUIRED)

# Scene visualization is optional; dynamic models run in the server without a GUI.
add_library(scene_visualization SHARED src/scene_visualization.cpp)
target_compile_features(scene_visualization PRIVATE cxx_std_17)
target_include_directories(scene_visualization PRIVATE ${OGRE_INCLUDE_DIRS} ${OGRE_PAGING_INCLUDE_DIR} ${OGRE_TERRAIN_INCLUDE_DIR})
target_link_libraries(scene_visualization PkgConfig::GAZEBO Qt5::Widgets yaml-cpp)
install(TARGETS scene_visualization LIBRARY DESTINATION lib)

add_library(scene_controller SHARED src/scene_controller.cpp)
target_compile_features(scene_controller PRIVATE cxx_std_17)
target_compile_options(scene_controller PRIVATE -Wall -Wextra)
ament_target_dependencies(scene_controller rclcpp nav_msgs gazebo_ros)
target_link_libraries(scene_controller PkgConfig::GAZEBO)
add_library(scene_map SHARED src/scene_map.cpp src/scene_geometry.cpp)
target_compile_features(scene_map PRIVATE cxx_std_17)
ament_target_dependencies(scene_map rclcpp gazebo_ros)
target_link_libraries(scene_map PkgConfig::GAZEBO yaml-cpp planning::planning_common planning::planning__rosidl_typesupport_cpp)
install(TARGETS scene_map LIBRARY DESTINATION lib)
install(TARGETS scene_controller LIBRARY DESTINATION lib)
install(
  DIRECTORY worlds launch config
  DESTINATION share/${PROJECT_NAME}
  PATTERN "__pycache__" EXCLUDE
)

ament_package()
