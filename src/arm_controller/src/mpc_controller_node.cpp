#include <ros/ros.h>
#include <XmlRpcValue.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/Float64MultiArray.h>
#include <trajectory_msgs/JointTrajectory.h>

#include <fstream>
#include <sstream>
#include <vector>

#include "arm_controller/arm_kinematics.h"
#include "arm_controller/mpc_solver.h"
#include "arm_controller/trajectory_handler.h"

namespace {

// 读取 rosparam 里的数值数组（XmlRpc）。
bool loadVectorParam(ros::NodeHandle& nh, const std::string& key,
                     std::vector<double>& out) {
    XmlRpc::XmlRpcValue v;
    if (!nh.getParam(key, v)) {
        return false;
    }
    if (v.getType() != XmlRpc::XmlRpcValue::TypeArray) {
        return false;
    }
    out.clear();
    for (int i = 0; i < v.size(); ++i) {
        out.push_back(static_cast<double>(v[i]));
    }
    return true;
}

}  // namespace

class MPCControllerNode {
public:
    MPCControllerNode() : nh_("~") {
        int horizon = 10;
        double dt = 0.05;
        int max_iter = 100;
        double lr = 0.05;
        nh_.param<int>("mpc/horizon", horizon, 10);
        nh_.param<double>("mpc/dt", dt, 0.05);
        nh_.param<int>("mpc/max_iter", max_iter, 100);
        nh_.param<double>("mpc/lr", lr, 0.05);
        horizon_ = horizon;
        dt_ = dt;

        // 运动学：从 robot_description 解析 URDF（4 个运动学关节）。
        std::string urdf;
        if (!ros::NodeHandle().getParam("/robot_description", urdf)) {
            nh_.param<std::string>("urdf_file", urdf, "");
            if (!urdf.empty()) {
                std::ifstream ifs(urdf);
                std::stringstream ss;
                ss << ifs.rdbuf();
                urdf = ss.str();
            }
        }
        if (urdf.empty() || !kin_.loadFromUrdf(urdf)) {
            ROS_FATAL("Failed to load URDF for kinematics");
            ros::shutdown();
            return;
        }
        ROS_INFO("Loaded kinematics: %d joints", kin_.numJoints());
        q_current_ = Eigen::VectorXd::Zero(kin_.numJoints());
        dq_current_ = Eigen::VectorXd::Zero(kin_.numJoints());

        solver_ = std::make_unique<arm_controller::MPCSolver>(
            kin_.numJoints(), horizon, dt);
        solver_->setMaxIterations(max_iter);
        solver_->setStepSize(lr);
        solver_->setJointLimits(kin_.lowerLimits(), kin_.upperLimits());
        solver_->setVelocityLimits(kin_.velocityLimits());
        loadWeights();

        state_sub_ = nh_.subscribe("/arm/joint_states", 1,
                                   &MPCControllerNode::jointStateCallback, this);
        // 参考轨迹：JointTrajectory（positions 按关节空间 URDF 弧度解释）。
        traj_sub_ = nh_.subscribe("/arm/end_effector_trajectory", 1,
                                  &MPCControllerNode::trajectoryCallback, this);
        cmd_pub_ = nh_.advertise<std_msgs::Float64MultiArray>(
            "/arm/joint_command", 1);

        control_timer_ = nh_.createTimer(ros::Duration(dt),
                                         &MPCControllerNode::controlLoop, this);
    }

private:
    void loadWeights() {
        std::vector<double> qp, qv, r;
        if (loadVectorParam(nh_, "mpc/Q_position", qp) &&
            static_cast<int>(qp.size()) >= kin_.numJoints()) {
            qp.resize(kin_.numJoints());
        }
        if (loadVectorParam(nh_, "mpc/Q_velocity", qv) &&
            static_cast<int>(qv.size()) >= kin_.numJoints()) {
            qv.resize(kin_.numJoints());
        }
        if (loadVectorParam(nh_, "mpc/R_control", r) &&
            static_cast<int>(r.size()) >= kin_.numJoints()) {
            r.resize(kin_.numJoints());
        }
        Eigen::VectorXd Qp = Eigen::VectorXd::Constant(kin_.numJoints(), 100.0);
        Eigen::VectorXd Qv = Eigen::VectorXd::Constant(kin_.numJoints(), 1.0);
        Eigen::VectorXd R = Eigen::VectorXd::Constant(kin_.numJoints(), 0.01);
        for (int i = 0; i < kin_.numJoints(); ++i) {
            if (!qp.empty()) Qp[i] = qp[i];
            if (!qv.empty()) Qv[i] = qv[i];
            if (!r.empty()) R[i] = r[i];
        }
        solver_->setWeights(Qp, Qv, R);
    }

    void jointStateCallback(const sensor_msgs::JointState::ConstPtr& msg) {
        if (msg->position.size() < static_cast<size_t>(kin_.numJoints())) {
            return;
        }
        for (int i = 0; i < kin_.numJoints(); ++i) {
            q_current_(i) = msg->position[i];
            dq_current_(i) = (msg->velocity.size() > static_cast<size_t>(i))
                                 ? msg->velocity[i]
                                 : 0.0;
        }
        if (msg->position.size() >= 5) {
            gripper_passthrough_ = msg->position[4];
        }
        state_received_ = true;
    }

    void trajectoryCallback(
        const trajectory_msgs::JointTrajectory::ConstPtr& msg) {
        trajectory_.setTrajectory(*msg);
        traj_start_time_ = ros::Time::now();
    }

    void controlLoop(const ros::TimerEvent&) {
        if (!state_received_) {
            return;
        }

        std::vector<Eigen::VectorXd> ref;
        if (!trajectory_.empty() &&
            trajectory_.isActive((ros::Time::now() - traj_start_time_).toSec())) {
            double t = (ros::Time::now() - traj_start_time_).toSec();
            Eigen::VectorXd q_ref;
            if (trajectory_.sample(t, q_ref)) {
                ref.resize(static_cast<size_t>(horizon_ + 1));
                // 供 MPC 使用的参考序列：从当前采样点起取 horizon+1 个采样。
                for (size_t k = 0; k < ref.size(); ++k) {
                    Eigen::VectorXd qk;
                    if (trajectory_.sample(t + k * dt_, qk)) {
                        ref[k] = qk;
                    } else {
                        ref[k] = q_current_;
                    }
                }
            }
        }
        if (ref.empty()) {
            // 无参考轨迹：保持当前位置（跟踪自身）。
            ref.push_back(q_current_);
        }

        std::vector<Eigen::VectorXd> u_opt;
        if (!solver_->solve(q_current_, dq_current_, ref, u_opt) ||
            u_opt.empty()) {
            ROS_WARN_THROTTLE(1.0, "MPC solve failed");
            return;
        }

        Eigen::VectorXd q_target = q_current_ + dt_ * u_opt[0];
        q_target = q_target.cwiseMax(kin_.lowerLimits())
                         .cwiseMin(kin_.upperLimits());

        std_msgs::Float64MultiArray cmd;
        cmd.data.resize(5);
        for (int i = 0; i < kin_.numJoints(); ++i) {
            cmd.data[i] = q_target(i);
        }
        cmd.data[4] = gripper_passthrough_;  // 夹爪原单位透传
        cmd_pub_.publish(cmd);
    }

    ros::NodeHandle nh_;
    arm_controller::ArmKinematics kin_;
    std::unique_ptr<arm_controller::MPCSolver> solver_;
    arm_controller::TrajectoryHandler trajectory_;
    ros::Subscriber state_sub_, traj_sub_;
    ros::Publisher cmd_pub_;
    ros::Timer control_timer_;

    int horizon_ = 10;
    double dt_ = 0.05;
    Eigen::VectorXd q_current_ = Eigen::VectorXd::Zero(4);
    Eigen::VectorXd dq_current_ = Eigen::VectorXd::Zero(4);
    double gripper_passthrough_ = 0.0;
    ros::Time traj_start_time_;
    bool state_received_ = false;
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "mpc_controller_node");
    MPCControllerNode node;
    ros::spin();
    return 0;
}
