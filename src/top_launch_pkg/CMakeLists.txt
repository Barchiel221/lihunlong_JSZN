cmake_minimum_required(VERSION 3.8)
project(top_launch_pkg)

find_package(ament_cmake REQUIRED)

install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

install(DIRECTORY rviz/
  DESTINATION share/${PROJECT_NAME}/rviz
)

# 改动 D: mission.yaml 任务文件(赛前现场只改这个文件)
install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)

ament_package()
