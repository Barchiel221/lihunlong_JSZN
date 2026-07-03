cmake_minimum_required(VERSION 3.8)
project(top_launch_pkg)

find_package(ament_cmake REQUIRED)

install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

install(DIRECTORY rviz/
  DESTINATION share/${PROJECT_NAME}/rviz
)

install(PROGRAMS
  scripts/pointcloud_frame_republisher.py
  scripts/pose_array_goal_sequencer.py
  scripts/pose_relay.py
  scripts/px4_sensor_combined_to_imu.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
