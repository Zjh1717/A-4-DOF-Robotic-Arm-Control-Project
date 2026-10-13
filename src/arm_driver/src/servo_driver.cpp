#include "arm_driver/servo_driver.h"

namespace arm_driver {

// ★ 必须存在，签名和 .h 完全一致
ServoDriver::ServoDriver(const std::string& port, int baudrate)
    : port_(port), baudrate_(baudrate) {
    serial_ = std::make_unique<SerialDriver>();
}

ServoDriver::~ServoDriver() {
    shutdown();
}

bool ServoDriver::init() {
    if (!serial_) return false;
    if (!serial_->open(port_, baudrate_)) {
        return false;
    }
    // TODO: 初始化各舵机（扭矩使能、模式设置等）
    return true;
}

// ... 其余 sendJointCommand / readJointState / emergencyStop / shutdown
//     按头文件里声明的签名逐一实现

}  // namespace arm_driver