cmake_minimum_required(VERSION 3.5)
project(robotis_op3_msgs)

find_package(ament_cmake REQUIRED)
find_package(op3_action_module_msgs)
find_package(op3_offset_tuner_msgs)
find_package(op3_online_walking_module_msgs)
find_package(op3_walking_module_msgs)

ament_package()