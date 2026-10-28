#ifndef ARM_DRIVER_SERIAL_DRIVER_H
#define ARM_DRIVER_SERIAL_DRIVER_H

#include <string>
#include <vector>
#include <cstdint>

namespace arm_driver {

// 协议无关的串口字节传输层（termios，Linux）。
// CH340 USB 串口，115200 8N1，无流控（依据 python_sdk.py）。
class SerialDriver {
public:
    SerialDriver() = default;
    ~SerialDriver();

    bool open(const std::string& port, int baudrate);
    void close();
    bool isOpen() const { return fd_ >= 0; }

    // 发送字节流（整段写入）。
    bool writeBytes(const std::vector<uint8_t>& data);

    // 在 timeout_ms 内读取当前可用的所有字节；返回读取到的字节数。
    int readAvailable(std::vector<uint8_t>& data, int timeout_ms);

    // 清除收发缓冲区。
    void flush();

    // 兼容旧接口的薄封装。
    bool sendFrame(const std::vector<uint8_t>& frame) { return writeBytes(frame); }
    bool receiveFrame(std::vector<uint8_t>& frame, int timeout_ms = 10) {
        return readAvailable(frame, timeout_ms) > 0;
    }

private:
    int fd_ = -1;
};

}  // namespace arm_driver
#endif
