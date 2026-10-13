#ifndef ARM_DRIVER_CONTROL_BASE_H
#define ARM_DRIVER_CONTROL_BASE_H

#include <vector>
#include <array>

namespace arm_driver {

// 机械臂关节数据（4个旋转关节 + 1个夹爪）
struct JointData {
    std::array<double, 5> position;    // 关节角/夹爪位置 [rad] or [m]
    std::array<double, 5> velocity;    // 关节速度 [rad/s]
    std::array<double, 5> effort;      // 力矩 [Nm]
    double timestamp;
};

class ControlBase {
public:
    ControlBase() = default;
    virtual ~ControlBase() = default;

    // 初始化（连接硬件/加载仿真模型）
    virtual bool init() = 0;

    // 发送关节目标位置
    virtual bool sendJointCommand(
        const std::array<double, 5>& target_pos) = 0;

    // 读取当前关节状态
    virtual bool readJointState(JointData& state) = 0;

    // 设置关节轨迹（可选，用于轨迹插值）
    virtual bool sendJointTrajectory(
        const std::vector<std::array<double, 5>>& waypoints,
        const std::vector<double>& durations) = 0;

    // 紧急停止
    virtual void emergencyStop() = 0;

    // 关闭通信
    virtual void shutdown() = 0;
};

}  // namespace arm_driver
#endif
