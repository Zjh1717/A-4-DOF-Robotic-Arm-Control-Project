#include "arm_driver/serial_driver.h"

#include <cerrno>
#include <cstring>
#include <fcntl.h>
#include <poll.h>
#include <sys/types.h>
#include <termios.h>
#include <unistd.h>

namespace arm_driver {

SerialDriver::~SerialDriver() {
    close();
}

namespace {

// 波特率常量 -> termios speed 常量（只覆盖本项目用到的 115200）。
speed_t baudToSpeed(int baudrate) {
    switch (baudrate) {
        case 9600: return B9600;
        case 19200: return B19200;
        case 38400: return B38400;
        case 57600: return B57600;
        case 115200: return B115200;
        default: return B115200;
    }
}

}  // namespace

bool SerialDriver::open(const std::string& port, int baudrate) {
    if (isOpen()) {
        close();
    }

    // O_NONBLOCK 避免打开时被 DCD/CTS 阻塞，随后再恢复阻塞模式。
    fd_ = ::open(port.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
    if (fd_ < 0) {
        return false;
    }

    struct termios tio;
    std::memset(&tio, 0, sizeof(tio));
    if (tcgetattr(fd_, &tio) != 0) {
        close();
        return false;
    }

    cfmakeraw(&tio);
    tio.c_cflag |= (CLOCAL | CREAD);
    tio.c_cflag &= ~CSIZE;
    tio.c_cflag |= CS8;
    tio.c_cflag &= ~PARENB;
    tio.c_cflag &= ~CSTOPB;
    tio.c_cflag &= ~CRTSCTS;
    tio.c_iflag &= ~(IXON | IXOFF | IXANY);

    speed_t spd = baudToSpeed(baudrate);
    cfsetispeed(&tio, spd);
    cfsetospeed(&tio, spd);

    // VMIN=0, VTIME=1：读超时 100ms，便于分片读取。
    tio.c_cc[VMIN] = 0;
    tio.c_cc[VTIME] = 1;

    if (tcsetattr(fd_, TCSANOW, &tio) != 0) {
        close();
        return false;
    }

    int flags = fcntl(fd_, F_GETFL, 0);
    if (flags != -1) {
        fcntl(fd_, F_SETFL, flags & ~O_NONBLOCK);
    }

    tcflush(fd_, TCIOFLUSH);
    return true;
}

void SerialDriver::close() {
    if (fd_ >= 0) {
        ::close(fd_);
        fd_ = -1;
    }
}

bool SerialDriver::writeBytes(const std::vector<uint8_t>& data) {
    if (!isOpen() || data.empty()) {
        return false;
    }
    size_t written = 0;
    while (written < data.size()) {
        ssize_t n = ::write(fd_, data.data() + written, data.size() - written);
        if (n < 0) {
            if (errno == EINTR) continue;
            return false;
        }
        written += static_cast<size_t>(n);
    }
    return true;
}

int SerialDriver::readAvailable(std::vector<uint8_t>& data, int timeout_ms) {
    data.clear();
    if (!isOpen()) {
        return 0;
    }
    int waited = 0;
    const int slice_ms = 10;
    while (waited < timeout_ms) {
        struct pollfd pfd;
        pfd.fd = fd_;
        pfd.events = POLLIN;
        pfd.revents = 0;
        int r = poll(&pfd, 1, slice_ms);
        if (r > 0 && (pfd.revents & POLLIN)) {
            uint8_t buf[256];
            ssize_t n = ::read(fd_, buf, sizeof(buf));
            if (n > 0) {
                data.insert(data.end(), buf, buf + n);
                waited = 0;  // 有新数据则重置空闲计时
            }
        } else if (r == 0) {
            waited += slice_ms;
        } else {
            break;  // 出错或设备关闭
        }
    }
    return static_cast<int>(data.size());
}

void SerialDriver::flush() {
    if (isOpen()) {
        tcflush(fd_, TCIOFLUSH);
    }
}

}  // namespace arm_driver
