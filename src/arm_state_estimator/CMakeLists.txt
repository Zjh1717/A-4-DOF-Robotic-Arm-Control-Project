cmake_minimum_required(VERSION 3.0.2)
project(arm_state_estimator)

add_compile_options(-std=c++14)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  sensor_msgs
)

find_package(Eigen3 REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES    arm_state_estimator_lib
  CATKIN_DEPENDS roscpp rospy std_msgs sensor_msgs
  DEPENDS EIGEN3
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
)

########################################
## 滤波器库
########################################
add_library(arm_state_estimator_lib
  src/kalman_filter.cpp
)
add_dependencies(arm_state_estimator_lib
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)
target_link_libraries(arm_state_estimator_lib ${catkin_LIBRARIES})

########################################
## ROS 节点
########################################
add_executable(state_estimator_node src/state_estimator_node.cpp)
add_dependencies(state_estimator_node
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)
target_link_libraries(state_estimator_node
  arm_state_estimator_lib
  ${catkin_LIBRARIES}
)

########################################
## install
########################################
install(TARGETS state_estimator_node arm_state_estimator_lib
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