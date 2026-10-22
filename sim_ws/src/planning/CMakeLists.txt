cmake_minimum_required(VERSION 3.8)
project(planning)

find_package(ament_cmake REQUIRED)
find_package(Ceres REQUIRED)

add_library(planning SHARED src/global/astar.cpp src/local/foot_motion.cpp)
add_library(planning::planning ALIAS planning)
target_compile_features(planning PUBLIC cxx_std_17)
target_include_directories(planning PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

add_library(teb_optimizer SHARED src/local/wheel/teb_optimizer.cpp)
add_library(planning::teb_optimizer ALIAS teb_optimizer)
target_compile_features(teb_optimizer PUBLIC cxx_std_17)
target_include_directories(teb_optimizer PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(teb_optimizer PUBLIC planning PRIVATE Ceres::ceres)

add_library(footstep_planner SHARED src/local/foot/footstep_planner.cpp)
add_library(planning::footstep_planner ALIAS footstep_planner)
target_compile_features(footstep_planner PUBLIC cxx_std_17)
target_include_directories(footstep_planner PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

target_link_libraries(footstep_planner PUBLIC planning)

install(DIRECTORY include/ DESTINATION include)
install(TARGETS planning teb_optimizer footstep_planner EXPORT export_planning
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
)

ament_export_targets(export_planning HAS_LIBRARY_TARGET)
ament_export_include_directories(include)

if(BUILD_TESTING)
  # Reuse private geometry helpers without copying the planner implementation.
  # This variant exports only acceptance functions; production exports only planners.
  add_library(foot_acceptance STATIC src/local/foot/footstep_planner.cpp)
  target_compile_features(foot_acceptance PRIVATE cxx_std_17)
  target_compile_definitions(foot_acceptance PRIVATE PLANNING_FOOT_ACCEPTANCE_ONLY)
  target_link_libraries(foot_acceptance PUBLIC planning)
  install(TARGETS foot_acceptance EXPORT export_planning ARCHIVE DESTINATION lib)
  enable_testing()
  add_executable(astar_test test/astar_test.cpp)
  target_link_libraries(astar_test planning)
  add_test(NAME astar_test COMMAND astar_test)
  add_executable(teb_test test/teb_test.cpp)
  target_link_libraries(teb_test planning teb_optimizer)
  add_test(NAME teb_test COMMAND teb_test)
  add_executable(foot_test test/foot_test.cpp)
  target_link_libraries(foot_test planning)
  add_test(NAME foot_test COMMAND foot_test)
endif()

ament_package()
