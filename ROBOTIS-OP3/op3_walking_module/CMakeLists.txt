################################################################################
# Set minimum required version of cmake, project name and compile options
################################################################################
cmake_minimum_required(VERSION 3.5)
project(op3_walking_module)

################################################################################
# Find packages and libraries for catkin and system dependencies
################################################################################
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(robotis_controller_msgs REQUIRED)
find_package(op3_walking_module_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Boost REQUIRED COMPONENTS thread)
find_package(yaml_cpp_vendor REQUIRED)

################################################################################
# Declare dependencies
################################################################################
set(dependencies
  rclcpp
  std_msgs
  sensor_msgs
  robotis_controller_msgs
  op3_walking_module_msgs
  Eigen3
  Boost
  yaml_cpp_vendor
)

################################################################################
# Include directories
################################################################################
include_directories(
  include
  ${rclcpp_INCLUDE_DIRS}
  ${std_msgs_INCLUDE_DIRS}
  ${sensor_msgs_INCLUDE_DIRS}
  ${robotis_controller_msgs_INCLUDE_DIRS}
  ${op3_walking_module_msgs_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
  ${yaml_cpp_vendor_INCLUDE_DIRS}
  /home/adimas/adimas_op3/src/ROBOTIS-OP3/op3_walking_module/include
  /home/adimas/adimas_op3/src/ROBOTIS-Framework/robotis_framework_common/include
  /home/adimas/adimas_op3/src/ROBOTIS-Framework/robotis_device/include
  /home/adimas/adimas_op3/src/DynamixelSDK/dynamixel_sdk/include
  /home/adimas/adimas_op3/src/ROBOTIS-Math/robotis_math/include
  /home/adimas/adimas_op3/src/ROBOTIS-OP3/op3_kinematics_dynamics/include  # Add this line
)

################################################################################
# Declare the library
################################################################################
add_library(${PROJECT_NAME} SHARED
  src/op3_walking_module.cpp
)

################################################################################
# Add dependencies to the library
################################################################################
ament_target_dependencies(${PROJECT_NAME} ${dependencies})

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
# Macro for declaring ament package
################################################################################
ament_package()
