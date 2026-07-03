cmake_minimum_required(VERSION 3.8)
project(ego_px4_adapter)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(quadrotor_msgs REQUIRED)
find_package(px4_msgs REQUIRED)
find_package(Eigen3 REQUIRED)

add_executable(ego_px4_adapter_node src/ego_px4_adapter_node.cpp)
target_include_directories(ego_px4_adapter_node PRIVATE ${EIGEN3_INCLUDE_DIRS})
ament_target_dependencies(ego_px4_adapter_node rclcpp quadrotor_msgs px4_msgs)

install(TARGETS ego_px4_adapter_node DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME} OPTIONAL)

ament_package()
