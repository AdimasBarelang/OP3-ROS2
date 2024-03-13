cmake_minimum_required(VERSION 3.5)
project(robotis_framework)

# Find dependencies
find_package(ament_cmake REQUIRED)

# Install package.xml
install(FILES package.xml
  DESTINATION share/${PROJECT_NAME}
)

ament_package()

