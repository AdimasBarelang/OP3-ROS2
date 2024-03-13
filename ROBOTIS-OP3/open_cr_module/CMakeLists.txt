################################################################################
# Set minimum required version of cmake, project name and compile options
################################################################################
cmake_minimum_required(VERSION 3.5)
project(open_cr_module)

# Compile as C++14, supported in ROS Foxy and newer
set(CMAKE_CXX_STANDARD 14)

################################################################################
# Find ament and other ROS2 packages and libraries for dependencies
################################################################################
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(robotis_controller_msgs REQUIRED)
find_package(robotis_framework_common REQUIRED)
find_package(robotis_device REQUIRED)
find_package(robotis_math REQUIRED)
find_package(Boost REQUIRED COMPONENTS thread)
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
# Build
################################################################################
include_directories(
  include
  ${Boost_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  /home/adimas/adimas_op3/src/ROBOTIS-Math/robotis_math/include
)

add_library(${PROJECT_NAME} src/open_cr_module.cpp)
ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  std_msgs
  sensor_msgs
  robotis_controller_msgs
  robotis_framework_common
  robotis_device
  robotis_math
)
target_link_libraries(${PROJECT_NAME} ${Boost_LIBRARIES} ${Eigen3_LIBRARIES})

################################################################################
# Install
################################################################################
install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/
  DESTINATION include
)

################################################################################
# Test
################################################################################
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

################################################################################
# Declare configuration to be passed to dependent projects
################################################################################
ament_package()