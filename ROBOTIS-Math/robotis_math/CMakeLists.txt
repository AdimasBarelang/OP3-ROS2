################################################################################
# Set minimum required version of cmake, project name and compile options
################################################################################
cmake_minimum_required(VERSION 3.5)
project(robotis_math)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

################################################################################
# Find ament packages and libraries for dependencies
################################################################################
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(Eigen3 REQUIRED)

################################################################################
# Setup for python modules and scripts
################################################################################

################################################################################
# Declare ROS messages, services and actions
################################################################################

################################################################################
# Declare ROS dynamic reconfigure parameters
################################################################################

################################################################################
# Declare dependencies for the package
################################################################################
set(dependencies
  rclcpp
  Eigen3
)

################################################################################
# Build
################################################################################
include_directories(
  include
  ${EIGEN3_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME}
  src/robotis_math_base.cpp
  src/robotis_trajectory_calculator.cpp
  src/minimum_jerk_trajectory.cpp
  src/minimum_jerk_trajectory_with_via_point.cpp
  src/fifth_order_polynomial_trajectory.cpp
  src/simple_trapezoidal_velocity_profile.cpp
  src/robotis_linear_algebra.cpp
  src/bezier_curve.cpp
  src/step_data_define.cpp
  src/preview_control.cpp
)

ament_target_dependencies(${PROJECT_NAME}
  ${dependencies}
)

################################################################################
# Install
################################################################################
install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION include
)

################################################################################
# Test
################################################################################

################################################################################
# Macro for ament package
################################################################################
ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})
ament_export_dependencies(${dependencies})
ament_package()