#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <array>
#include "arm_state_estimator/kalman_filter.h"

class StateEstimatorNode {
public:
    StateEstimatorNode() : nh_("~") {
        double dt;
        nh_.param<double>("dt", dt, 0.01);

        // 为5个关节分别创建卡尔曼滤波器
        for (int i = 0; i < 5; ++i) {
            filters_[i] = std::make_unique<
                arm_state_estimator::JointKalmanFilter>(dt);
        }

        // 订阅原始关节状态（含噪声）
        raw_sub_ = nh_.subscribe(
            "/arm/joint_states_raw", 1,
            &StateEstimatorNode::rawStateCallback, this);

        // 发布滤波后的关节状态
        filtered_pub_ = nh_.advertise<sensor_msgs::JointState>(
            "/arm/joint_states", 10);

        timer_ = nh_.createTimer(
            ros::Duration(dt),
            &StateEstimatorNode::estimatorLoop, this);
    }

private:
    void rawStateCallback(
        const sensor_msgs::JointState::ConstPtr& msg) {
        for (int i = 0; i < 5 && i < (int)msg->position.size(); ++i) {
            latest_raw_[i] = msg->position[i];
        }
        raw_received_ = true;
    }

    void estimatorLoop(const ros::TimerEvent&) {
        if (!raw_received_) return;

        sensor_msgs::JointState filtered;
        filtered.header.stamp = ros::Time::now();
        filtered.name = {"joint_0", "joint_1", "joint_2",
                         "joint_3", "joint_4_gripper"};
        filtered.position.resize(5);
        filtered.velocity.resize(5);

        for (int i = 0; i < 5; ++i) {
            filters_[i]->predict(0.0);  // 预测（加速度指令可外部传入）
            filters_[i]->update(latest_raw_[i]);  // 更新
            filtered.position[i] = filters_[i]->getPosition();
            filtered.velocity[i] = filters_[i]->getVelocity();
        }

        filtered_pub_.publish(filtered);
    }

    ros::NodeHandle nh_;
    std::array<std::unique_ptr<arm_state_estimator::JointKalmanFilter>, 5>
        filters_;
    ros::Subscriber raw_sub_;
    ros::Publisher filtered_pub_;
    ros::Timer timer_;
    std::array<double, 5> latest_raw_ = {0, 0, 0, 0, 0};
    bool raw_received_ = false;
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "state_estimator_node");
    StateEstimatorNode node;
    ros::spin();
    return 0;
}
