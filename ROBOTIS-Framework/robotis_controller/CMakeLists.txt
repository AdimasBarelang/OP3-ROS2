cmake_minimum_required(VERSION 3.5)
project(robotis_controller)

# Set C++11 as the default compiler standard
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(YAML_CPP_INCLUDE_DIRS "/home/adimas/yaml-cpp/include")

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(robotis_controller_msgs REQUIRED)
find_package(dynamixel_sdk REQUIRED)
find_package(robotis_device REQUIRED)
find_package(robotis_framework_common REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(robotis_controller_msgs REQUIRED)
find_package(ament_index_cpp REQUIRED)


# Include directories
include_directories(
  include
  ${rclcpp_INCLUDE_DIRS}
  ${std_msgs_INCLUDE_DIRS}
  ${sensor_msgs_INCLUDE_DIRS}
  ${robotis_controller_msgs_INCLUDE_DIRS}
  ${dynamixel_sdk_INCLUDE_DIRS}
  ${robotis_device_INCLUDE_DIRS}
  ${robotis_framework_common_INCLUDE_DIRS}
  ${YAML_CPP_INCLUDE_DIRS}
)

# Check YAML-CPP version and add definition if needed
if(NOT ${YAML_CPP_VERSION} VERSION_LESS "0.5")
  add_definitions(-DHAVE_NEW_YAMLCPP)
endif()

# Create library
add_library(robotis_controller SHARED
  src/robotis_controller/robotis_controller.cpp
)

# Specify libraries to link a library or executable target against
ament_target_dependencies(robotis_controller
  rclcpp
  std_msgs
  sensor_msgs
  robotis_controller_msgs
  dynamixel_sdk
  robotis_device
  robotis_framework_common
  yaml-cpp
  ament_index_cpp

)

# Link yaml-cpp library
target_link_libraries(robotis_controller
  ${YAML_CPP_LIBRARIES}
)


ament_target_dependencies(robotis_controller robotis_controller_msgs)

# Install library
install(TARGETS robotis_controller
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# Install header files
install(DIRECTORY include/
  DESTINATION include
)

ament_export_include_directories(include)
ament_export_libraries(robotis_controller
${YAML_CPP_LIBRARIES}
)
ament_export_dependencies(
  rclcpp
  std_msgs
  sensor_msgs
  robotis_controller_msgs
  dynamixel_sdk
  robotis_device
  robotis_framework_common
  yaml-cpp
  ament_index_cpp
)

# Export package
ament_package()
