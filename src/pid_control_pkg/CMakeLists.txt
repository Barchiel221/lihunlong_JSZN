cmake_minimum_required(VERSION 3.8)
project(pid_control_pkg)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

add_executable(pid_control_node src/pid_control.cpp)

target_include_directories(pid_control_node PRIVATE include)

ament_target_dependencies(pid_control_node
  rclcpp
  std_msgs
  geometry_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
)

install(TARGETS pid_control_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
