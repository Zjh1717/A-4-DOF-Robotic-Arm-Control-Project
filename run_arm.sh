#!/usr/bin/env bash
# 一键启动真机机械臂：驱动(REPL串口) + 状态估计(KF) + MPC控制器 + RViz
#
# 用法：
#   ./run_arm.sh                              # 默认 /dev/ttyUSB0
#   ARM_PORT=/dev/ttyUSB1 ./run_arm.sh        # 指定串口
#   RUN_ARM_BUILD=1 ./run_arm.sh              # 强制重新 catkin_make
#   ./run_arm.sh arm_bringup arm_real.launch  # 等价于默认
set -euo pipefail

WS_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROS_DISTRO="${ROS_DISTRO:-noetic}"
ROS_SETUP="/opt/ros/${ROS_DISTRO}/setup.bash"

if [ ! -f "${ROS_SETUP}" ]; then
    echo "错误：未找到 ${ROS_SETUP}" >&2
    exit 1
fi
# shellcheck disable=SC1090
source "${ROS_SETUP}"

cd "${WS_DIR}"
if [ ! -f devel/setup.bash ] || [ "${RUN_ARM_BUILD:-0}" = "1" ]; then
    echo "==> 正在编译工作区 (catkin_make) ..."
    catkin_make
fi
# shellcheck disable=SC1090
source devel/setup.bash

PORT="${ARM_PORT:-/dev/ttyUSB0}"
if [ -e "${PORT}" ]; then
    echo "==> 检测到串口 ${PORT}"
else
    echo "==> 警告：未检测到串口 ${PORT}（可用 ARM_PORT=/dev/ttyXXX 覆盖）"
fi

if [ $# -gt 0 ]; then
    exec roslaunch "$@"
else
    exec roslaunch arm_bringup arm_real.launch port:="${PORT}"
fi
