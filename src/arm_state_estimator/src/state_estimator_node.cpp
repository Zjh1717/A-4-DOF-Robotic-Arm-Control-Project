#include <ros/ros.h>
#include <sensor_msgs/JointState.h>

#include <algorithm>
#include <array>
#include <string>
#include <vector>

#include "arm_state_estimator/kalman_filter.h"

class StateEstimatorNode {
public:
    StateEstimatorNode() : nh_("~") {
        double dt = 0.02;
        double q_pos = 1e-4;
        double q_vel = 1e-3;
        double r = 1e-3;
        nh_.param<double>("dt", dt, 0.02);
        nh_.param<double>("process_noise_pos", q_pos, 1e-4);
        nh_.param<double>("process_noise_vel", q_vel, 1e-3);
        nh_.param<double>("measurement_noise", r, 1e-3);
        nh_.param<int>("num_kinematic_joints", num_kinematic_, 4);
        nh_.param<int>("num_joints", num_joints_, 5);
        nh_.param<std::vector<std::string>>(
            "joint_names", joint_names_,
            {"joint1", "joint2", "joint3", "joint4", "tool_fixed"});
        // 内部数组固定为 5 个通道，防止误配置越界。
        num_joints_ = std::min(std::max(num_joints_, 1), 5);
        num_kinematic_ = std::min(std::max(num_kinematic_, 0), num_joints_);

        // 只为运动学关节建滤波器；夹爪（末尾）直接透传。
        for (int i = 0; i < num_kinematic_; ++i) {
            filters_[i] = std::make_unique<
                arm_state_estimator::JointKalmanFilter>(dt, q_pos, q_vel, r);
        }

        raw_sub_ = nh_.subscribe("/arm/joint_states_raw", 1,
                                 &StateEstimatorNode::rawStateCallback, this);
        filtered_pub_ = nh_.advertise<sensor_msgs::JointState>(
            "/arm/joint_states", 10);

        timer_ = nh_.createTimer(ros::Duration(dt),
                                 &StateEstimatorNode::estimatorLoop, this);
    }

private:
    void rawStateCallback(const sensor_msgs::JointState::ConstPtr& msg) {
        for (int i = 0; i < num_joints_ && i < (int)msg->position.size(); ++i) {
            latest_raw_[i] = msg->position[i];
        }
        raw_received_ = true;
    }

    void estimatorLoop(const ros::TimerEvent&) {
        if (!raw_received_) {
            return;
        }

        sensor_msgs::JointState filtered;
        filtered.header.stamp = ros::Time::now();
        filtered.name = joint_names_;
        filtered.position.resize(num_joints_);
        filtered.velocity.resize(num_joints_);

        for (int i = 0; i < num_kinematic_; ++i) {
            filters_[i]->predict(0.0);  // 无外部加速度输入
            filters_[i]->update(latest_raw_[i]);
            filtered.position[i] = filters_[i]->getPosition();
            filtered.velocity[i] = filters_[i]->getVelocity();
        }
        for (int i = num_kinematic_; i < num_joints_; ++i) {
            filtered.position[i] = latest_raw_[i];  // 夹爪等非运动学通道透传
            filtered.velocity[i] = 0.0;
        }

        filtered_pub_.publish(filtered);
    }

    ros::NodeHandle nh_;
    ros::Subscriber raw_sub_;
    ros::Publisher filtered_pub_;
    ros::Timer timer_;
    std::array<std::unique_ptr<arm_state_estimator::JointKalmanFilter>, 5>
        filters_;
    std::array<double, 5> latest_raw_ = {0, 0, 0, 0, 0};
    std::vector<std::string> joint_names_;
    int num_kinematic_ = 4;
    int num_joints_ = 5;
    bool raw_received_ = false;
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "state_estimator_node");
    StateEstimatorNode node;
    ros::spin();
    return 0;
}
