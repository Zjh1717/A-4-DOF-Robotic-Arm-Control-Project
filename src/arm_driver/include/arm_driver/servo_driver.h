#ifndef ARM_DRIVER_SERVO_DRIVER_H
#define ARM_DRIVER_SERVO_DRIVER_H

#include "arm_driver/control_base.h"
#include "arm_driver/serial_driver.h"
#include "arm_driver/servo_protocol.h"

#include <array>
#include <memory>
#include <string>

namespace arm_driver {

// 等价于 python_sdk.py 中 class Arm 的 C++ 串口驱动。
// 通过 CH340 USB 串口与 ESP32/MicroPython 固件通信，协议为 MicroPython REPL
// 文本命令（非二进制帧）。实现 ControlBase 接口，供 arm_driver_node 使用。
//
// 约定：
//  - sendJointCommand 接收 URDF 空间目标（前 4 关节弧度 + 夹爪原单位），
//    内部统一转换成固件角度（度）后通过 arm.bundle.move 下发。
//  - readJointState 读回固件角度（度），转换成 URDF 弧度返回（夹爪原样透传）。
class ServoDriver : public ControlBase {
public:
    ServoDriver(const std::string& port, int baudrate,
                int read_timeout_ms = 1000,
                double joint2_offset_deg = kJoint2CodeToUrdfOffsetDeg,
                int move_speed = 30);
    ~ServoDriver() override;

    bool init() override;
    bool sendJointCommand(
        const std::array<double, kTotalJoints>& target_pos) override;
    bool readJointState(JointData& state) override;
    bool sendJointTrajectory(
        const std::vector<std::array<double, kTotalJoints>>& waypoints,
        const std::vector<double>& durations) override;
    void emergencyStop() override;
    void shutdown() override;

    bool isConnected() const { return connected_; }

    // 低层 REPL 接口（对应 python_sdk.py 的 Arm 方法）。
    bool connect();
    bool ready(int retry = 5, double delay_s = 0.2);
    bool sendCmd(const std::string& cmd, bool need_ack = false,
                 std::string* reply = nullptr);
    bool moveJoints(const std::array<int, kTotalJoints>& ids,
                    const std::array<double, kTotalJoints>& vals_deg,
                    int speed);
    bool getJoints(std::array<double, kTotalJoints>& vals_deg);
    bool lockJoints();
    bool freeJoints();

private:
    std::unique_ptr<SerialDriver> serial_;
    std::string port_;
    int baudrate_;
    int read_timeout_ms_;
    double joint2_offset_deg_;
    int move_speed_;
    bool connected_ = false;

    std::string readReply(int max_bytes = 512);
};

}  // namespace arm_driver
#endif
