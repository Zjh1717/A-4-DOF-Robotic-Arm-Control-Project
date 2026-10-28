#!/usr/bin/env python3
"""发送末端目标位姿（base_link 系）给 MPC 的 IK 求解。

用法：
  python3 scripts/send_pose_goal.py x y z qx qy qz qw

  x y z   —— 末端 tool 位置（米，base_link 系，z 以底座底面为 0）
  qx qy qz qw —— 期望朝向四元数（ROS 顺序 x,y,z,w；不关心朝向可传 0 0 0 1）

示例（复位位姿对应的 tool 点，朝向水平朝前）：
  python3 scripts/send_pose_goal.py 0.1024 0.0 0.1674 0 0 0 1
"""
import sys
from typing import List

import rospy
from geometry_msgs.msg import PoseStamped


def main() -> None:
    if len(sys.argv) != 8:
        print(__doc__)
        sys.exit(1)
    vals: List[float] = [float(v) for v in sys.argv[1:8]]
    x, y, z, qx, qy, qz, qw = vals

    rospy.init_node("send_pose_goal", anonymous=True)
    pub = rospy.Publisher("/arm/target_pose", PoseStamped, queue_size=1, latch=True)
    rospy.sleep(0.5)  # 等待订阅者连接

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
    rospy.loginfo("sent pose (%.3f, %.3f, %.3f)", x, y, z)
    rospy.sleep(1.0)


if __name__ == "__main__":
    main()
