#ifndef ARM_DRIVER_SERVO_DRIVER_H
#define ARM_DRIVER_SERVO_DRIVER_H

#include "arm_driver/control_base.h"
#include "arm_driver/serial_driver.h"
#include <memory>

namespace arm_driver {

class ServoDriver : public ControlBase {
public:
    ServoDriver(const std::string& port, int baudrate);
    ~ServoDriver() override;

    bool init() override;
    bool sendJointCommand(
        const std::array<double, 5>& target_pos) override;
    bool readJointState(JointData& state) override;
    bool sendJointTrajectory(
        const std::vector<std::array<double, 5>>& waypoints,
        const std::vector<double>& durations) override;
    void emergencyStop() override;
    void shutdown() override;

private:
    std::unique_ptr<SerialDriver> serial_;
    std::string port_;
    int baudrate_;

    // 舵机ID映射：关节索引 → 舵机总线ID
    std::array<int, 5> servo_ids_ = {0, 1, 2, 3, 4};

    // 舵机方向（+1或-1，根据实际安装方向）
    std::array<int, 5> servo_directions_ = {1, 1, 1, 1, 1};

    // 角度→舵机原始值转换
    int angleToRaw(double angle_rad, int joint_idx);
    double rawToAngle(int raw_value, int joint_idx);

    // 构建舵机控制帧
    std::vector<uint8_t> buildPosCommand(
        int servo_id, double angle_rad);
    // 构建读取角度请求帧
    std::vector<uint8_t> buildReadCommand(int servo_id);
    // 解析反馈帧
    bool parseFeedback(const std::vector<uint8_t>& frame,
                       int& servo_id, double& angle_rad);
};

}  // namespace arm_driver
#endif
