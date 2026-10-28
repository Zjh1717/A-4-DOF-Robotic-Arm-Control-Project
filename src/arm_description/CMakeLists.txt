cmake_minimum_required(VERSION 3.0.2)
project(arm_description)

find_package(catkin REQUIRED)

catkin_package()

# 安装资源目录到 install space（仅安装实际存在的目录）
install(DIRECTORY urdf rviz
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
