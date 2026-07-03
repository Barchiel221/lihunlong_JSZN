cmake_minimum_required(VERSION 3.8)
project(uart_to_stm32)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(serial_comm REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Boost REQUIRED COMPONENTS system thread)

add_executable(uart_to_stm32_node
  src/uart_to_stm32_node.cpp
  src/uart_to_stm32.cpp
)

target_include_directories(uart_to_stm32_node PRIVATE
  include
  ${EIGEN3_INCLUDE_DIRS}
)

ament_target_dependencies(uart_to_stm32_node
  rclcpp
  tf2_ros
  tf2_geometry_msgs
  geometry_msgs
  nav_msgs
  std_msgs
)

target_link_libraries(uart_to_stm32_node
  serial_comm::serial_comm
  Boost::system
  Boost::thread
)

install(TARGETS uart_to_stm32_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
