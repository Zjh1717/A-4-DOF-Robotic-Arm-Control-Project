#ifndef ARM_CONTROLLER_TRAJECTORY_HANDLER_H
#define ARM_CONTROLLER_TRAJECTORY_HANDLER_H

#include <Eigen/Dense>
#include <trajectory_msgs/JointTrajectory.h>

#include <vector>

namespace arm_controller {

// 接收关节空间参考轨迹，按时间线性插值采样，并提供看门狗。
class TrajectoryHandler {
public:
    TrajectoryHandler() = default;

    // 设置参考轨迹（positions 为 URDF 空间弧度）。
    void setTrajectory(const trajectory_msgs::JointTrajectory& traj,
                       double timeout_s = 2.0);

    // 在 time 时刻采样（time 为轨迹开始后的秒数）；失败返回 false。
    bool sample(double time, Eigen::VectorXd& q_ref) const;

    // 当前轨迹是否仍有效（时间未超时）。
    bool isActive(double time) const { return time <= end_time_; }

    bool empty() const { return points_.empty(); }
    int numJoints() const { return num_joints_; }

private:
    struct Sample {
        double t = 0.0;
        std::vector<double> q;
    };
    std::vector<Sample> points_;
    int num_joints_ = 0;
    double end_time_ = 0.0;
};

}  // namespace arm_controller

#endif  // ARM_CONTROLLER_TRAJECTORY_HANDLER_H
