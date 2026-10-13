#ifndef ARM_DRIVER_SERIAL_DRIVER_H
#define ARM_DRIVER_SERIAL_DRIVER_H

#include <string>
#include <vector>
#include <cstdint>

namespace arm_driver {

class SerialDriver {
public:
    SerialDriver() = default;
    ~SerialDriver();

    bool open(const std::string& port, int baudrate);
    void close();
    bool isOpen() const { return fd_ >= 0; }

    // 发送指令帧
    bool sendFrame(const std::vector<uint8_t>& frame);

    // 接收数据帧（阻塞读取）
    bool receiveFrame(std::vector<uint8_t>& frame, int timeout_ms = 10);

    // 清除缓冲区
    void flush();

private:
    int fd_ = -1;
    // TODO: 根据您的舵机协议实现帧的构建与校验
    // 常见协议格式: [0xFF, 0xFF, ID, LEN, CMD, PARAM... , CHECKSUM]
};

}  // namespace arm_driver
#endif
