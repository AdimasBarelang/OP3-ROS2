################################################################################
# Set minimum required version of cmake, project name and compile options
################################################################################
cmake_minimum_required(VERSION 3.5)
project(op3_balance_control)

# Compile as C++14
set(CMAKE_CXX_STANDARD 14)

################################################################################
# Find ament packages and libraries for dependencies
################################################################################
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(robotis_math REQUIRED)

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
# Declare dependencies
################################################################################
ament_export_dependencies(rclcpp)
ament_export_dependencies(robotis_math)
ament_export_dependencies(Eigen3)

################################################################################
# Build
################################################################################
include_directories(
  include
  ${EIGEN3_INCLUDE_DIRS}
  /home/adimas/adimas_op3/src/ROBOTIS-Math/robotis_math/include
)

add_library(${PROJECT_NAME} src/op3_balance_control.cpp)
ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  robotis_math
)
target_link_libraries(${PROJECT_NAME} ${Eigen3_LIBRARIES})

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
# Ament Specific
################################################################################
ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})
ament_package()