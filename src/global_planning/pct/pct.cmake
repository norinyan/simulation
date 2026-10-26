find_package(Eigen3 REQUIRED)
find_package(GTSAM REQUIRED)
set(PCT_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/pct/third_party")
set(CMAKE_POSITION_INDEPENDENT_CODE ON)
set(UNITTESTS OFF CACHE BOOL "" FORCE)
set(ENABLE_MKL_PARDISO OFF CACHE BOOL "" FORCE)
add_subdirectory("${PCT_ROOT}/osqp" "${CMAKE_CURRENT_BINARY_DIR}/pct_osqp" EXCLUDE_FROM_ALL)
file(GLOB PCT_SPLINE_SOURCES "${PCT_ROOT}/src/common/smoothing/*.cc" "${PCT_ROOT}/src/common/smoothing/solver/osqp/*.cc" "${PCT_ROOT}/src/common/smoothing/utils/*.cc")
file(GLOB_RECURSE PCT_FACTOR_SOURCES "${PCT_ROOT}/src/trajectory_optimization/gpmp_optimizer/*.cc")
add_library(pct_backend STATIC
  "${PCT_ROOT}/src/a_star/a_star_search.cc"
  "${PCT_ROOT}/src/map_manager/dense_elevation_map.cc"
  "${PCT_ROOT}/src/ele_planner/offline_ele_planner.cc"
  "${PCT_ROOT}/src/trajectory_optimization/height_smoother/height_smoother.cc"
  ${PCT_SPLINE_SOURCES} ${PCT_FACTOR_SOURCES})
target_include_directories(pct_backend PUBLIC "${PCT_ROOT}/src" "${PCT_ROOT}/osqp/include" "${CMAKE_CURRENT_BINARY_DIR}/pct_osqp/include")
target_link_libraries(pct_backend PUBLIC Eigen3::Eigen gtsam osqpstatic)
target_compile_options(pct_backend PRIVATE -Wno-return-type)
target_compile_features(pct_backend PUBLIC cxx_std_17)

