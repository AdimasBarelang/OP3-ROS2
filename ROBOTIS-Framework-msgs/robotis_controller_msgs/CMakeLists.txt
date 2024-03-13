cmake_minimum_required(VERSION 3.5)
project(robotis_controller_msgs)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# Declare ROS messages and services
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/SyncWriteItem.msg"
  "msg/JointCtrlModule.msg"
  "msg/StatusMsg.msg"
  "msg/WriteControlTable.msg"
  "srv/GetJointModule.srv"
  "srv/SetJointModule.srv"
  "srv/SetModule.srv"
  "srv/LoadOffset.srv"
  DEPENDENCIES std_msgs sensor_msgs
)

# Export dependencies
ament_export_dependencies(rosidl_default_runtime)

# Package specific configuration
ament_package()
