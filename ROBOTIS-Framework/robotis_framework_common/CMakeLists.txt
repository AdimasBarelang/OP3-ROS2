cmake_minimum_required(VERSION 3.5)
project(robotis_framework_common)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(robotis_device REQUIRED)

# Include directories
include_directories(include)

# Set C++ standard
set(CMAKE_CXX_STANDARD 14)

# Create a library
add_library(${PROJECT_NAME}
  include/${PROJECT_NAME}/motion_module.h
  include/${PROJECT_NAME}/sensor_module.h
  include/${PROJECT_NAME}/singleton.h
)

# Add dependencies to the library
ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  robotis_device
)

# Set the language for the target
set_target_properties(${PROJECT_NAME} PROPERTIES LINKER_LANGUAGE CXX)

# Install
install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION include/${PROJECT_NAME}
)

# Export information
ament_export_include_directories(include)
ament_export_dependencies(rclcpp robotis_device)

# Add tests and examples
if(BUILD_TESTING)
  # Add tests here if needed
endif()

# Generate ament package
ament_package()

