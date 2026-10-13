#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/Float64MultiArray.h>
#include "arm_driver/servo_driver.h"
#include "arm_driver/control_base.h"

class ArmDriverNode {
public:
    ArmDriverNode() : nh_("~") {
        // 从参数服务器读取配置
        std::string port;
        int baudrate;
        nh_.param<std::string>("port", port, "/dev/ttyUSB0");
        nh_.param<int>("baudrate", baudrate, 115200);

        // 创建实机驱动（多态：基类指针指向派生类）
        driver_ = std::make_unique<arm_driver::ServoDriver>(
            port, baudrate);

        if (!driver_->init()) {
            ROS_FATAL("Failed to initialize servo driver");
            ros::shutdown();
            return;
        }

        // 订阅MPC控制器的关节目标
        cmd_sub_ = nh_.subscribe(
            "/arm/joint_command", 1,
            &ArmDriverNode::jointCommandCallback, this);

        // 发布关节状态（供MPC和卡尔曼滤波使用）
        state_pub_ = nh_.advertise<sensor_msgs::JointState>(
            "/arm/joint_states", 10);

        // 定时读取关节状态
        read_timer_ = nh_.createTimer(
            ros::Duration(0.01),  // 100Hz
            &ArmDriverNode::readStateTimer, this);
    }

private:
    void jointCommandCallback(
        const std_msgs::Float64MultiArray::ConstPtr& msg) {
        if (msg->data.size() < 5) {
            ROS_WARN("Command must have 5 elements");
            return;
        }
        std::array<double, 5> target;
        std::copy_n(msg->data.begin(), 5, target.begin());
        driver_->sendJointCommand(target);
    }

    void readStateTimer(const ros::TimerEvent&) {
        arm_driver::JointData state;
        if (!driver_->readJointState(state)) return;

        sensor_msgs::JointState js;
        js.header.stamp = ros::Time::now();
        js.name = {"joint_0", "joint_1", "joint_2",
                   "joint_3", "joint_4_gripper"};
        js.position.assign(state.position.begin(),
                           state.position.end());
        js.velocity.assign(state.velocity.begin(),
                           state.velocity.end());
        state_pub_.publish(js);
    }

    ros::NodeHandle nh_;
    std::unique_ptr<arm_driver::ControlBase> driver_;
    ros::Subscriber cmd_sub_;
    ros::Publisher state_pub_;
    ros::Timer read_timer_;
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "arm_driver_node");
    ArmDriverNode node;
    ros::spin();
    return 0;
}
