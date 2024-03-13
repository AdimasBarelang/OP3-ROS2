################################################################################
# Set minimum required version of CMake, project name, and compile options
################################################################################
cmake_minimum_required(VERSION 3.5)
project(op3_kinematics_dynamics)

################################################################################
# Find dependencies
################################################################################
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(robotis_math REQUIRED)
find_package(Eigen3 REQUIRED)

################################################################################
# Include directories
################################################################################
include_directories(
  include
  ${EIGEN3_INCLUDE_DIRS}
  /home/adimas/adimas_op3/src/ROBOTIS-Math/robotis_math/include  # Add this line
)

################################################################################
# Add library
################################################################################
add_library(${PROJECT_NAME}
  src/link_data.cpp
  src/op3_kinematics_dynamics.cpp
)

################################################################################
# Add dependencies
################################################################################
ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  robotis_math
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
  DESTINATION include/${PROJECT_NAME}
)

################################################################################
# Export
################################################################################
ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})
ament_package()
