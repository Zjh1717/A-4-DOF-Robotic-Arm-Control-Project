cmake_minimum_required(VERSION 3.0.2)
project(arm_controller)

add_compile_options(-std=c++14)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  sensor_msgs
  trajectory_msgs
  geometry_msgs
  arm_driver
)

find_package(Eigen3 REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES    arm_controller_lib
  CATKIN_DEPENDS roscpp rospy std_msgs sensor_msgs trajectory_msgs geometry_msgs arm_driver
  DEPENDS EIGEN3
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
)

########################################
## 控制器核心库
########################################
add_library(arm_controller_lib
  src/mpc_solver.cpp
  src/arm_kinematics.cpp
  src/arm_ik.cpp
  src/trajectory_handler.cpp
)
add_dependencies(arm_controller_lib
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)
target_link_libraries(arm_controller_lib ${catkin_LIBRARIES})

########################################
## ROS 节点
########################################
add_executable(mpc_controller_node src/mpc_controller_node.cpp)
add_dependencies(mpc_controller_node
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)
target_link_libraries(mpc_controller_node
  arm_controller_lib
  ${catkin_LIBRARIES}
)

########################################
## install
########################################
install(TARGETS mpc_controller_node arm_controller_lib
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)
install(DIRECTORY launch config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
