cmake_minimum_required(VERSION 3.10)
project(camera_base)

add_definitions(-std=c++14 -Wall -Wextra -Wno-unknown-pragmas)

find_package(ament_cmake REQUIRED)

find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(camera_info_manager REQUIRED)

include_directories(
  include
)

add_library(${PROJECT_NAME}
  src/camera_node_base.cpp
  src/camera_ros_base.cpp
)

ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  sensor_msgs
  diagnostic_msgs
  camera_info_manager
)

install(TARGETS ${PROJECT_NAME}
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include)
ament_export_include_directories(include)

ament_export_dependencies(
  camera_info_manager
  diagnostic_msgs
  sensor_msgs
  image_transport
  builtin_interfaces
)

ament_package()
