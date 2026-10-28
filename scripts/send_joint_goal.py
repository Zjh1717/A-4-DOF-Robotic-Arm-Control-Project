#!/usr/bin/env python3
"""发送一个 4 关节目标（URDF 空间，弧度）给 MPC 控制器。

用法：
  python3 scripts/send_joint_goal.py q1 q2 q3 q4

示例（保持当前位姿，把 4 个值替换为 /arm/joint_states 读到的当前值）：
  python3 scripts/send_joint_goal.py 0.0 -1.5708 0.7854 0.7941

夹爪不参与运动学，本脚本不下发夹爪目标。
"""
import sys

import rospy
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


def main():
    if len(sys.argv) != 5:
        print(__doc__)
        sys.exit(1)
    try:
        q = [float(x) for x in sys.argv[1:5]]
    except ValueError:
        print("关节值必须是数字")
        sys.exit(1)

    rospy.init_node("send_joint_goal", anonymous=True)
    pub = rospy.Publisher(
        "/arm/end_effector_trajectory", JointTrajectory,
        queue_size=1, latch=True)
    rospy.sleep(0.5)  # 等待订阅者连接

    msg = JointTrajectory()
    msg.joint_names = ["joint1", "joint2", "joint3", "joint4"]
    point = JointTrajectoryPoint()
    point.positions = q
    point.time_from_start = rospy.Duration(2.0)
    msg.points = [point]

    pub.publish(msg)
    rospy.loginfo("sent goal %s", q)
    rospy.sleep(1.0)


if __name__ == "__main__":
    main()
