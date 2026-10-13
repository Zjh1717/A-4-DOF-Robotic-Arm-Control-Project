#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/Float64MultiArray.h>
#include <trajectory_msgs/JointTrajectory.h>
#include "arm_controller/mpc_solver.h"

class MPCControllerNode {
public:
    MPCControllerNode() : nh_("~") {
        int horizon;
        double dt;
        nh_.param<int>("mpc/horizon", horizon, 10);
        nh_.param<double>("mpc/dt", dt, 0.05);

        solver_ = std::make_unique<arm_controller::MPCSolver>(
            5, horizon, dt);  // 4关节 + 夹爪

        // 加载MPC参数
        loadParams();

        // 订阅当前关节状态
        state_sub_ = nh_.subscribe(
            "/arm/joint_states", 1,
            &MPCControllerNode::jointStateCallback, this);

        // 订阅末端轨迹指令
        traj_sub_ = nh_.subscribe(
            "/arm/end_effector_trajectory", 1,
            &MPCControllerNode::trajectoryCallback, this);

        // 发布关节角位置指令给驱动
        cmd_pub_ = nh_.advertise<std_msgs::Float64MultiArray>(
            "/arm/joint_command", 1);

        // 控制循环定时器
        control_timer_ = nh_.createTimer(
            ros::Duration(dt),
            &MPCControllerNode::controlLoop, this);
    }

private:
    void loadParams() {
        // TODO: 从参数服务器读取权重和约束
        Eigen::VectorXd Q_pos(5), Q_vel(5), R(5);
        Q_pos << 100, 100, 100, 50, 10;
        Q_vel << 1, 1, 1, 1, 0.1;
        R << 0.01, 0.01, 0.01, 0.01, 0.01;
        solver_->setWeights(Q_pos, Q_vel, R);

        Eigen::VectorXd q_lower(5), q_upper(5);
        q_lower << -3.14, -1.57, -2.0, -1.57, 0.0;
        q_upper << 3.14, 1.57, 2.0, 1.57, 0.03;
        solver_->setJointLimits(q_lower, q_upper);
    }

    void jointStateCallback(
        const sensor_msgs::JointState::ConstPtr& msg) {
        if (msg->position.size() < 5) return;
        for (int i = 0; i < 5; ++i) {
            q_current_(i) = msg->position[i];
            dq_current_(i) = msg->velocity.size() > i
                             ? msg->velocity[i] : 0.0;
        }
        state_received_ = true;
    }

    void trajectoryCallback(
        const trajectory_msgs::JointTrajectory::ConstPtr& msg) {
        // 将末端轨迹序列转换为关节空间参考轨迹
        // TODO: 使用逆运动学将末端轨迹映射为关节角轨迹
        ref_trajectory_.clear();
        for (const auto& point : msg->points) {
            Eigen::VectorXd ref_q(5);
            for (int i = 0; i < 5 && i < (int)point.positions.size(); ++i)
                ref_q(i) = point.positions[i];
            ref_trajectory_.push_back(ref_q);
        }
        trajectory_received_ = true;
    }

    void controlLoop(const ros::TimerEvent&) {
        if (!state_received_ || !trajectory_received_) return;

        // 调用MPC求解器
        std::vector<Eigen::VectorXd> u_opt;
        bool success = solver_->solve(
            q_current_, dq_current_, ref_trajectory_, u_opt);

        if (!success || u_opt.empty()) {
            ROS_WARN_THROTTLE(1.0, "MPC solver failed");
            return;
        }

        // 取第一个控制量：q_target = q_current + u_0
        Eigen::VectorXd q_target = q_current_ + u_opt[0];

        // 发布关节指令
        std_msgs::Float64MultiArray cmd;
        cmd.data.resize(5);
        for (int i = 0; i < 5; ++i)
            cmd.data[i] = q_target(i);
        cmd_pub_.publish(cmd);
    }

    ros::NodeHandle nh_;
    std::unique_ptr<arm_controller::MPCSolver> solver_;
    ros::Subscriber state_sub_, traj_sub_;
    ros::Publisher cmd_pub_;
    ros::Timer control_timer_;

    Eigen::VectorXd q_current_ = Eigen::VectorXd::Zero(5);
    Eigen::VectorXd dq_current_ = Eigen::VectorXd::Zero(5);
    std::vector<Eigen::VectorXd> ref_trajectory_;
    bool state_received_ = false;
    bool trajectory_received_ = false;
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "mpc_controller_node");
    MPCControllerNode node;
    ros::spin();
    return 0;
}
