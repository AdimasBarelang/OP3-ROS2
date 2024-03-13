################################################################################
# Set minimum required version of cmake, project name and compile options
################################################################################
cmake_minimum_required(VERSION 3.5)
project(op3_walking_module_msgs)

################################################################################
# Find ament packages and libraries for dependencies
################################################################################
find_package(ament_cmake REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

################################################################################
# Setup for python modules and scripts
################################################################################

################################################################################
# Declare ROS messages, services and actions
################################################################################
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/WalkingParam.msg"
  "srv/GetWalkingParam.srv"
  "srv/SetWalkingParam.srv"
  DEPENDENCIES std_msgs
)

################################################################################
# Declare ROS dynamic reconfigure parameters
################################################################################

################################################################################
# Declare dependencies
################################################################################
ament_export_dependencies(rosidl_default_runtime)

################################################################################
# Install
################################################################################
install(DIRECTORY msg srv 
  DESTINATION share/${PROJECT_NAME})

################################################################################
# Test
################################################################################

ament_package()