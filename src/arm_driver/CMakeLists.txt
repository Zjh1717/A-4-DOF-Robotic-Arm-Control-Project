cmake_minimum_required(VERSION 3.0.2)
project(arm_driver)

add_compile_options(-std=c++14)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  sensor_msgs
  serial
  message_generation
)

########################################
## 自定义消息
########################################
add_message_files(
  FILES
  ArmJointState.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

########################################
## catkin 导出
########################################
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES    arm_driver_lib
  CATKIN_DEPENDS roscpp rospy std_msgs sensor_msgs serial message_runtime
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

########################################
## 驱动库（串口 + 舵机 + 控制基类）
########################################
add_library(arm_driver_lib
  src/serial_driver.cpp
  src/servo_driver.cpp
)
add_dependencies(arm_driver_lib
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)
target_link_libraries(arm_driver_lib ${catkin_LIBRARIES})

########################################
## ROS 节点
########################################
add_executable(arm_driver_node src/arm_driver_node.cpp)
add_dependencies(arm_driver_node
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)
target_link_libraries(arm_driver_node
  arm_driver_lib
  ${catkin_LIBRARIES}
)

########################################
## install
########################################
install(TARGETS arm_driver_node arm_driver_lib
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