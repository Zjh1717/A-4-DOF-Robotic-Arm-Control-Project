#!/usr/bin/env python3
"""键盘增量控制机械臂末端 xyz 坐标（走 /arm/target_pose -> IK -> 关节 MPC）。

键位（小写字母，终端内直接按键）：
  w / s      x 轴 + / -   （前 / 后）
  a / d      y 轴 - / +   （左 / 右）
  Space / z  z 轴 + / -   （上 / 下；Shift 键无法在终端单独捕获，故下降用 z）
  r          一键回到启动时的初始位姿
  q          退出

用法：
  python3 scripts/teleop_pose.py [step]
  step 为每次按键的位移增量（米），默认 0.01。

说明：
  - 按住方向键会连发（依赖终端按键自动重复；个别禁用了自动重复的终端按住只会走
    一步，此时需连按或改用带 key-up 事件的 pynput 实现）。
  - 脚本先读一次 /arm/joint_states，用正解求出当前末端 (x,y,z)，并锁定当前
    工具朝向（yaw/pitch），随后只增量平移位置。
  - 目标被夹到工作空间边界（半径 0.18 m、高度 -0.097~0.245 m）；若落入
    不可达“空洞”，控制器会保持不动并告警。
"""
import fcntl
import math
import os
import sys
import termios
import time
import tty
from typing import Dict, List, Optional, Tuple

import rospy
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import JointState

STEP_DEFAULT = 0.01
R_MAX = 0.18
Z_MIN = -0.097
Z_MAX = 0.245

# 键 -> (dx, dy, dz)；None 表示退出。
KEYMAP: Dict[str, Optional[Tuple[float, float, float]]] = {
    "w": (1.0, 0.0, 0.0),
    "s": (-1.0, 0.0, 0.0),
    "a": (0.0, -1.0, 0.0),
    "d": (0.0, 1.0, 0.0),
    " ": (0.0, 0.0, 1.0),
    "z": (0.0, 0.0, -1.0),
    "q": None,
}


def forward_kinematics(q: List[float]) -> Tuple[float, float, float]:
    """URDF 空间 4 关节正解 -> 末端 (x, y, z)。参数与 arm_kinematics.cpp 一致。"""
    link = 0.06
    shoulder = 0.065
    tr = link * (math.cos(q[1]) + math.cos(q[1] + q[2]) +
                 math.cos(q[1] + q[2] + q[3]))
    z = shoulder - link * (math.sin(q[1]) + math.sin(q[1] + q[2]) +
                           math.sin(q[1] + q[2] + q[3]))
    return tr * math.cos(q[0]), tr * math.sin(q[0]), z


def yaw_pitch_quaternion(yaw: float, pitch: float) -> Tuple[float, float, float, float]:
    """把 R = Rz(yaw)*Ry(pitch) 转成 ROS 四元数 (x, y, z, w)。"""
    cy, sy = math.cos(yaw), math.sin(yaw)
    cp, sp = math.cos(pitch), math.sin(pitch)
    r = [[cy * cp, -sy, cy * sp],
         [sy * cp, cy, sy * sp],
         [-sp, 0.0, cp]]
    tr = r[0][0] + r[1][1] + r[2][2]
    if tr > 0.0:
        s = math.sqrt(tr + 1.0) * 2.0
        w = 0.25 * s
        x = (r[2][1] - r[1][2]) / s
        y = (r[0][2] - r[2][0]) / s
        z = (r[1][0] - r[0][1]) / s
    elif r[0][0] > r[1][1] and r[0][0] > r[2][2]:
        s = math.sqrt(1.0 + r[0][0] - r[1][1] - r[2][2]) * 2.0
        w = (r[2][1] - r[1][2]) / s
        x = 0.25 * s
        y = (r[0][1] + r[1][0]) / s
        z = (r[0][2] + r[2][0]) / s
    elif r[1][1] > r[2][2]:
        s = math.sqrt(1.0 + r[1][1] - r[0][0] - r[2][2]) * 2.0
        w = (r[0][2] - r[2][0]) / s
        x = (r[0][1] + r[1][0]) / s
        y = 0.25 * s
        z = (r[1][2] + r[2][1]) / s
    else:
        s = math.sqrt(1.0 + r[2][2] - r[0][0] - r[1][1]) * 2.0
        w = (r[1][0] - r[0][1]) / s
        x = (r[0][2] + r[2][0]) / s
        y = (r[1][2] + r[2][1]) / s
        z = 0.25 * s
    return x, y, z, w


def set_raw_mode():
    fd = sys.stdin.fileno()
    old_tc = termios.tcgetattr(fd)
    tty.setcbreak(fd)
    old_fl = fcntl.fcntl(fd, fcntl.F_GETFL)
    fcntl.fcntl(fd, fcntl.F_SETFL, old_fl | os.O_NONBLOCK)
    return old_tc, old_fl


def restore_mode(state) -> None:
    old_tc, old_fl = state
    fd = sys.stdin.fileno()
    termios.tcsetattr(fd, termios.TCSADRAIN, old_tc)
    fcntl.fcntl(fd, fcntl.F_SETFL, old_fl)


def drain_keys() -> List[str]:
    """一次性读走当前所有待处理的按键（用于按住连发）。"""
    fd = sys.stdin.fileno()
    keys: List[str] = []
    while True:
        try:
            data = os.read(fd, 256)
        except (BlockingIOError, OSError):
            break
        if not data:
            break
        keys.extend(data.decode("utf-8", "ignore"))
    return keys


def main() -> None:
    step = float(sys.argv[1]) if len(sys.argv) > 1 else STEP_DEFAULT
    rospy.init_node("teleop_pose", anonymous=True)
    pub = rospy.Publisher("/arm/target_pose", PoseStamped, queue_size=1, latch=True)

    rospy.loginfo("waiting for /arm/joint_states ...")
    js = rospy.wait_for_message("/arm/joint_states", JointState, timeout=10.0)
    q: List[float] = [js.position[0], js.position[1], js.position[2], js.position[3]]
    init_x, init_y, init_z = forward_kinematics(q)
    yaw = q[0]
    pitch = q[1] + q[2] + q[3]
    qx, qy, qz, qw = yaw_pitch_quaternion(yaw, pitch)

    x, y, z = init_x, init_y, init_z
    rospy.loginfo("initial ee: (%.3f, %.3f, %.3f), step=%.3f m", x, y, z, step)
    print("w/s: x  a/d: y  Space/z: z  r: 回初始  q: 退出")

    def publish() -> None:
        msg = PoseStamped()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = "base_link"
        msg.pose.position.x = x
        msg.pose.position.y = y
        msg.pose.position.z = z
        msg.pose.orientation.x = qx
        msg.pose.orientation.y = qy
        msg.pose.orientation.z = qz
        msg.pose.orientation.w = qw
        pub.publish(msg)
        print("ee: (%.3f, %.3f, %.3f)" % (x, y, z))

    state = set_raw_mode()
    try:
        while not rospy.is_shutdown():
            keys = drain_keys()
            if not keys:
                time.sleep(0.02)
                continue
            for key in keys:
                if key == "q":
                    return
                if key == "r":
                    x, y, z = init_x, init_y, init_z
                    publish()
                    continue
                delta = KEYMAP.get(key)
                if delta is None:
                    continue
                dx, dy, dz = delta
                x += dx * step
                y += dy * step
                z += dz * step
                r = math.hypot(x, y)
                if r > R_MAX:
                    x *= R_MAX / r
                    y *= R_MAX / r
                z = max(Z_MIN, min(Z_MAX, z))
                publish()
    finally:
        restore_mode(state)


if __name__ == "__main__":
    main()
