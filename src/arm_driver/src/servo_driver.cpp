#include "arm_driver/servo_driver.h"

#include <ros/ros.h>

#include <algorithm>
#include <cstdlib>
#include <sstream>

namespace arm_driver {

ServoDriver::ServoDriver(const std::string& port, int baudrate,
                         int read_timeout_ms, double joint2_offset_deg,
                         int move_speed)
    : port_(port),
      baudrate_(baudrate),
      read_timeout_ms_(read_timeout_ms),
      joint2_offset_deg_(joint2_offset_deg),
      move_speed_(move_speed) {
    serial_ = std::make_unique<SerialDriver>();
}

ServoDriver::~ServoDriver() {
    shutdown();
}

bool ServoDriver::init() {
    return connect();
}

void ServoDriver::shutdown() {
    if (serial_) {
        serial_->close();
    }
    connected_ = false;
}

bool ServoDriver::connect() {
    if (connected_) {
        return true;
    }
    if (!serial_->open(port_, baudrate_)) {
        return false;
    }
    if (!ready()) {
        serial_->close();
        return false;
    }
    connected_ = true;
    return true;
}

bool ServoDriver::ready(int retry, double delay_s) {
    if (!serial_ || !serial_->isOpen()) {
        return false;
    }
    for (int i = 0; i < retry; ++i) {
        serial_->flush();
        std::vector<uint8_t> crlf = {'\r', '\n'};
        serial_->writeBytes(crlf);
        // 等待 '>>>' 提示符（等价 python_sdk.py 的 ready()）。
        ros::Duration(delay_s).sleep();
        std::string reply = readReply();
        if (reply.find(">>>") != std::string::npos) {
            return true;
        }
    }
    return false;
}

bool ServoDriver::sendCmd(const std::string& cmd, bool need_ack,
                          std::string* reply) {
    if (!connected_ || !serial_ || !serial_->isOpen()) {
        return false;
    }
    serial_->flush();
    std::string full = cmd + "\r\n";
    std::vector<uint8_t> bytes(full.begin(), full.end());
    if (!serial_->writeBytes(bytes)) {
        return false;
    }
    if (need_ack || reply) {
        std::string ret = readReply();
        // 剥离回显的命令文本（等价 python_sdk.py 的 ret.replace(cmd,'')），
        // 避免命令里的数字污染后续数值解析。
        size_t pos = ret.find(cmd);
        if (pos != std::string::npos) {
            ret.erase(pos, cmd.size());
        }
        if (reply) {
            *reply = ret;
        }
    }
    return true;
}

std::string ServoDriver::readReply(int max_bytes) {
    std::vector<uint8_t> data;
    serial_->readAvailable(data, read_timeout_ms_);
    if (static_cast<int>(data.size()) > max_bytes) {
        data.resize(max_bytes);
    }
    std::string text(data.begin(), data.end());
    // 剥离 '>>>' 提示符（等价 python_sdk.py send_cmd 的处理）。
    size_t pos;
    while ((pos = text.find(">>>")) != std::string::npos) {
        text.erase(pos, 3);
    }
    return text;
}

bool ServoDriver::getJoints(std::array<double, kTotalJoints>& vals_deg) {
    std::string reply;
    if (!sendCmd(makeGetJointsCommand(), true, &reply)) {
        return false;
    }
    // 固件返回 5 个空格分隔的浮点数（可能夹杂命令回显）。
    std::istringstream iss(reply);
    std::vector<double> vals;
    std::string tok;
    while (iss >> tok) {
        char* end = nullptr;
        double v = std::strtod(tok.c_str(), &end);
        if (end != tok.c_str()) {
            vals.push_back(v);
        }
    }
    if (vals.size() < kTotalJoints) {
        return false;
    }
    for (int i = 0; i < kTotalJoints; ++i) {
        vals_deg[i] = vals[i];
    }
    return true;
}

bool ServoDriver::moveJoints(const std::array<int, kTotalJoints>& ids,
                             const std::array<double, kTotalJoints>& vals_deg,
                             int speed) {
    std::ostringstream oss;
    oss.precision(3);
    oss << "arm.bundle.move(ids=[";
    for (int i = 0; i < kTotalJoints; ++i) {
        oss << ids[i] << (i + 1 < kTotalJoints ? "," : "");
    }
    oss << "],vals=[";
    for (int i = 0; i < kTotalJoints; ++i) {
        oss << vals_deg[i] << (i + 1 < kTotalJoints ? "," : "");
    }
    oss << "],speed=" << speed << ")";
    return sendCmd(oss.str(), false, nullptr);
}

bool ServoDriver::lockJoints() {
    return sendCmd(makeLockCommand(), false, nullptr);
}

bool ServoDriver::freeJoints() {
    return sendCmd(makeFreeCommand(), false, nullptr);
}

bool ServoDriver::sendJointCommand(
    const std::array<double, kTotalJoints>& target_pos) {
    // target_pos: [q0..q3 URDF 弧度, 夹爪原单位]。
    std::array<double, kKinematicJoints> urdf_rad{};
    std::array<double, kKinematicJoints> code_deg{};
    for (int i = 0; i < kKinematicJoints; ++i) {
        urdf_rad[i] = target_pos[i];
    }
    urdfKinematicToCode(urdf_rad, code_deg, joint2_offset_deg_);

    std::array<int, kTotalJoints> ids = {0, 1, 2, 3, kGripperJointId};
    std::array<double, kTotalJoints> vals{};
    for (int i = 0; i < kKinematicJoints; ++i) {
        vals[i] = code_deg[i];
    }
    vals[kGripperJointId] = target_pos[kGripperJointId];  // 夹爪原单位透传
    return moveJoints(ids, vals, move_speed_);
}

bool ServoDriver::readJointState(JointData& state) {
    std::array<double, kTotalJoints> code_deg{};
    if (!getJoints(code_deg)) {
        return false;
    }
    std::array<double, kKinematicJoints> code_kin{};
    std::array<double, kKinematicJoints> urdf_rad{};
    for (int i = 0; i < kKinematicJoints; ++i) {
        code_kin[i] = code_deg[i];
    }
    codeKinematicToUrdf(code_kin, urdf_rad, joint2_offset_deg_);

    for (int i = 0; i < kKinematicJoints; ++i) {
        state.position[i] = urdf_rad[i];
        state.velocity[i] = 0.0;  // 速度由 arm_state_estimator 的 KF 估计
        state.effort[i] = 0.0;
    }
    state.position[kGripperJointId] = code_deg[kGripperJointId];  // 原单位
    state.velocity[kGripperJointId] = 0.0;
    state.effort[kGripperJointId] = 0.0;
    state.timestamp = 0.0;  // 由节点用 ros::Time::now() 填充
    return true;
}

bool ServoDriver::sendJointTrajectory(
    const std::vector<std::array<double, kTotalJoints>>& waypoints,
    const std::vector<double>& durations) {
    // 下位机 bundle.move 为阻塞式点动，逐点下发即可；durations 暂不参与
    // 时序规划（TODO: 若固件支持分段插值，按 durations 做时间约束）。
    for (const auto& wp : waypoints) {
        if (!sendJointCommand(wp)) {
            return false;
        }
    }
    return true;
}

void ServoDriver::emergencyStop() {
    // 固件协议无专用急停指令，用 lock()（失能）作为尽力而为的停机。
    lockJoints();
}

}  // namespace arm_driver
