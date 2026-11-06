// linux_serial_io.hpp - SerialIO for a Linux serial port (Jetson Nano, WSL, any Linux PC)
//
// Linux counterpart of win_serial_io.hpp. Uses termios2 so any baud rate works,
// including non-standard ones like 256000 that older termios cannot set.
//
// Permission: the user needs to be in the dialout group to open the port:
//     sudo usermod -aG dialout $USER     (then log out and back in)
#pragma once
#include "rplidar_lite.hpp"

#include <asm/termbits.h>   // struct termios2, BOTHER (do not also include <termios.h>)
#include <sys/ioctl.h>
#include <fcntl.h>
#include <unistd.h>
#include <time.h>
#include <cerrno>
#include <string>

class LinuxSerialIO : public SerialIO {
public:
    ~LinuxSerialIO() { close(); }

    // port: "/dev/ttyUSB0" or just "ttyUSB0"
    bool open(const std::string& port, uint32_t baud) {
        close();
        std::string path = port;
        if (path.find('/') == std::string::npos) path = "/dev/" + path;

        // O_NONBLOCK so open() never waits for a carrier signal; switched back off below.
        fd_ = ::open(path.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
        if (fd_ < 0) return false;

        // Exclusive access, like opening a COM port with no sharing on Windows.
        if (ioctl(fd_, TIOCEXCL) != 0) { close(); return false; }

        struct termios2 tio = {};
        if (ioctl(fd_, TCGETS2, &tio) != 0) { close(); return false; }

        // Raw 8N1, no flow control, no line editing or character translation.
        tio.c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR | ICRNL |
                         IXON | IXOFF | IXANY);
        tio.c_oflag &= ~OPOST;
        tio.c_lflag &= ~(ECHO | ECHONL | ICANON | ISIG | IEXTEN);
        tio.c_cflag &= ~(CSIZE | PARENB | CSTOPB | CRTSCTS | CBAUD | (CBAUD << IBSHIFT));
        tio.c_cflag |= CS8 | CREAD | CLOCAL | BOTHER | (BOTHER << IBSHIFT);
        tio.c_ispeed = baud;
        tio.c_ospeed = baud;

        // Reads return immediately with whatever is already received.
        tio.c_cc[VMIN] = 0;
        tio.c_cc[VTIME] = 0;

        if (ioctl(fd_, TCSETS2, &tio) != 0) { close(); return false; }

        // Back to blocking mode: writes wait until the data is queued (reads still
        // return at once because of VMIN = 0 and VTIME = 0).
        int flags = fcntl(fd_, F_GETFL);
        if (flags >= 0) fcntl(fd_, F_SETFL, flags & ~O_NONBLOCK);

        ioctl(fd_, TCFLSH, TCIOFLUSH);   // drop anything already in the buffers

        // RTS off (Windows: RTS_CONTROL_DISABLE), then DTR off, same as the
        // Slamtec SDK does after opening.
        int rts = TIOCM_RTS;
        ioctl(fd_, TIOCMBIC, &rts);
        setDTR(false);
        return true;
    }

    void close() {
        if (fd_ >= 0) { ::close(fd_); fd_ = -1; }
    }

    int write(const uint8_t* data, int len) override {
        if (fd_ < 0) return 0;
        int total = 0;
        while (total < len) {
            ssize_t n = ::write(fd_, data + total, (size_t)(len - total));
            if (n < 0) {
                if (errno == EINTR) continue;   // interrupted by a signal (e.g. Ctrl+C): retry
                break;
            }
            total += (int)n;
        }
        return total;
    }

    int read(uint8_t* buffer, int maxLen) override {
        if (fd_ < 0) return 0;
        ssize_t n = ::read(fd_, buffer, (size_t)maxLen);
        return n > 0 ? (int)n : 0;
    }

    uint32_t millis() override {
        struct timespec ts;
        clock_gettime(CLOCK_MONOTONIC, &ts);
        return (uint32_t)((uint64_t)ts.tv_sec * 1000u + (uint64_t)ts.tv_nsec / 1000000u);
    }

    void sleepMs(uint32_t ms) override {
        struct timespec req = { (time_t)(ms / 1000), (long)(ms % 1000) * 1000000L };
        while (nanosleep(&req, &req) != 0 && errno == EINTR) {}   // keep sleeping if a signal interrupts
    }

    void setDTR(bool asserted) override {
        if (fd_ < 0) return;
        int dtr = TIOCM_DTR;
        ioctl(fd_, asserted ? TIOCMBIS : TIOCMBIC, &dtr);
    }

private:
    int fd_ = -1;
};
