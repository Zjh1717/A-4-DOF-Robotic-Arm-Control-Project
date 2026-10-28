#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/Float64MultiArray.h>

#include <array>
#include <string>
#include <vector>

#include "arm_driver/servo_driver.h"
#include "arm_driver/servo_protocol.h"

class ArmDriverNode {
public:
    ArmDriverNode() : nh_("~") {
        std::string port;
        int baudrate = 115200;
        int read_timeout_ms = 1000;
        int move_speed = 30;
        double read_rate_hz = 50.0;
        double joint2_offset_deg = arm_driver::kJoint2CodeToUrdfOffsetDeg;

        nh_.param<std::string>("port", port, "/dev/ttyUSB0");
        nh_.param<int>("baudrate", baudrate, 115200);
        nh_.param<int>("read_timeout_ms", read_timeout_ms, 1000);
        nh_.param<int>("move_speed", move_speed, 30);
        nh_.param<double>("read_rate_hz", read_rate_hz, 50.0);
        nh_.param<double>("joint2_offset_deg", joint2_offset_deg,
                          arm_driver::kJoint2CodeToUrdfOffsetDeg);
        nh_.param<std::vector<std::string>>(
            "joint_names", joint_names_,
            {"joint1", "joint2", "joint3", "joint4", "tool_fixed"});

        driver_ = std::make_unique<arm_driver::ServoDriver>(
            port, baudrate, read_timeout_ms, joint2_offset_deg, move_speed);

        if (!driver_->init()) {
            ROS_FATAL("Failed to initialize REPL servo driver on %s", port.c_str());
            ros::shutdown();
            return;
        }
        ROS_INFO("Connected to firmware via REPL on %s", port.c_str());

        // 订阅控制器下发的关节目标（前 4 个为 URDF 弧度，第 5 个夹爪原单位）。
        cmd_sub_ = nh_.subscribe("/arm/joint_command", 1,
                                 &ArmDriverNode::jointCommandCallback, this);

        // 发布原始关节状态（URDF 弧度），供状态估计器滤波后发布 /arm/joint_states。
        raw_state_pub_ = nh_.advertise<sensor_msgs::JointState>(
            "/arm/joint_states_raw", 10);

        ros::Duration period(1.0 / read_rate_hz);
        read_timer_ = nh_.createTimer(period, &ArmDriverNode::controlCycle, this);
    }

private:
    void jointCommandCallback(
        const std_msgs::Float64MultiArray::ConstPtr& msg) {
        if (msg->data.size() < arm_driver::kTotalJoints) {
            ROS_WARN_THROTTLE(1.0,
                              "joint_command must have %d elements, got %zu",
                              arm_driver::kTotalJoints, msg->data.size());
            return;
        }
        for (int i = 0; i < arm_driver::kTotalJoints; ++i) {
            pending_cmd_[i] = msg->data[i];
        }
        has_pending_cmd_ = true;
    }

    void controlCycle(const ros::TimerEvent&) {
        // 下位机 bundle.move 阻塞到执行完成；有指令先下发，再读回状态。
        if (has_pending_cmd_) {
            if (driver_->sendJointCommand(pending_cmd_)) {
                has_pending_cmd_ = false;
            } else {
                ROS_WARN_THROTTLE(1.0, "Failed to send joint command");
            }
        }

        arm_driver::JointData state;
        if (!driver_->readJointState(state)) {
            ROS_WARN_THROTTLE(1.0, "Failed to read joint state");
            return;
        }

        sensor_msgs::JointState js;
        js.header.stamp = ros::Time::now();
        js.name = joint_names_;
        js.position.resize(arm_driver::kTotalJoints);
        js.velocity.resize(arm_driver::kTotalJoints);
        js.effort.resize(arm_driver::kTotalJoints);
        for (int i = 0; i < arm_driver::kTotalJoints; ++i) {
            js.position[i] = state.position[i];
            js.velocity[i] = state.velocity[i];
            js.effort[i] = state.effort[i];
        }
        raw_state_pub_.publish(js);
    }

    ros::NodeHandle nh_;
    std::unique_ptr<arm_driver::ServoDriver> driver_;
    ros::Subscriber cmd_sub_;
    ros::Publisher raw_state_pub_;
    ros::Timer read_timer_;
    std::vector<std::string> joint_names_;
    std::array<double, arm_driver::kTotalJoints> pending_cmd_{};
    bool has_pending_cmd_ = false;
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "arm_driver_node");
    ArmDriverNode node;
    ros::spin();
    return 0;
}
