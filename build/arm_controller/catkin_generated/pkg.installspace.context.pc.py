# generated from catkin/cmake/template/pkg.context.pc.in
CATKIN_PACKAGE_PREFIX = ""
PROJECT_PKG_CONFIG_INCLUDE_DIRS = "${prefix}/include;/usr/include/eigen3".split(';') if "${prefix}/include;/usr/include/eigen3" != "" else []
PROJECT_CATKIN_DEPENDS = "roscpp;rospy;std_msgs;sensor_msgs;trajectory_msgs;arm_driver".replace(';', ' ')
PKG_CONFIG_LIBRARIES_WITH_PREFIX = "-larm_controller_lib".split(';') if "-larm_controller_lib" != "" else []
PROJECT_NAME = "arm_controller"
PROJECT_SPACE_DIR = "/home/zjh/Linux_robot_arm/install"
PROJECT_VERSION = "0.0.0"
