################################################################################
# Set minimum required version of CMake and project name
################################################################################
cmake_minimum_required(VERSION 3.5)
project(robotis_device)

################################################################################
# Find dependencies
################################################################################
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(dynamixel_sdk REQUIRED)

################################################################################
# Include directories
################################################################################
include_directories(include)

################################################################################
# Set C++ standard
################################################################################
set(CMAKE_CXX_STANDARD 14)

################################################################################
# Create a library
################################################################################
add_library(${PROJECT_NAME}
  src/robotis_device/robot.cpp
  src/robotis_device/sensor.cpp
  src/robotis_device/dynamixel.cpp
)

################################################################################
# Add dependencies to the library
################################################################################
ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  dynamixel_sdk
)

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

install(DIRECTORY devices
  DESTINATION share/${PROJECT_NAME}
)

################################################################################
# Export information
################################################################################
ament_export_include_directories(include)
ament_export_dependencies(rclcpp dynamixel_sdk)

################################################################################
# Add tests and examples
################################################################################
if(BUILD_TESTING)
  # Add tests here if needed
endif()

################################################################################
# Generate ament package
################################################################################
ament_package()

