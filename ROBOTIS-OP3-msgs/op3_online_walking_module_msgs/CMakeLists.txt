################################################################################
# Set minimum required version of cmake, project name and compile options
################################################################################
cmake_minimum_required(VERSION 3.5)
project(op3_online_walking_module_msgs)

################################################################################
# Find ament packages and libraries for dependencies
################################################################################
find_package(ament_cmake REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

################################################################################
# Setup for python modules and scripts
################################################################################

################################################################################
# Declare ROS messages, services and actions
################################################################################
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/JointPose.msg"
  "msg/KinematicsPose.msg"
  "msg/FootStepCommand.msg"
  "msg/FootStepArray.msg"
  "msg/PreviewRequest.msg"
  "msg/PreviewResponse.msg"
  "msg/WalkingParam.msg"
  "msg/Step2D.msg"
  "msg/Step2DArray.msg"
  "srv/GetJointPose.srv"
  "srv/GetKinematicsPose.srv"
  "srv/GetPreviewMatrix.srv"
  DEPENDENCIES std_msgs sensor_msgs geometry_msgs
)

################################################################################
# Declare ROS dynamic reconfigure parameters
################################################################################

################################################################################
# Declare dependencies
################################################################################
ament_export_dependencies(rosidl_default_runtime)

################################################################################
# Build
################################################################################

################################################################################
# Install
################################################################################
install(DIRECTORY msg srv DESTINATION share/${PROJECT_NAME})

################################################################################
# Test
################################################################################

################################################################################
# Ament Specific
################################################################################
ament_package()