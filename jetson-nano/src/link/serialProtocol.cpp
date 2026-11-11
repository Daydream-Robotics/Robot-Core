/** Nano-side serial transport. */
#include "protocol/serialProtocol.hpp"

#include <chrono>
#include <string>
#include <thread>
#include <fcntl.h>
#include <unistd.h>

/**
 * @brief writes a short string to a sysfs file
 *
 * @param path file to write, e.g. "/sys/class/gpio/export"
 * @param text text to write
 * @return bool false if the file couldn't be opened or the write failed
 */
static bool writeFile(const std::string& path, const std::string& text) {
    // open for writing only
    int file = open(path.c_str(), O_WRONLY);
    // file doesn't exist or no permission
    if (file < 0) {
        return false;
    }
    // sysfs takes the whole value in one write
    ssize_t written = write(file, text.data(), text.size());
    close(file);
    return written == static_cast<ssize_t>(text.size());
}

SerialProtocol::~SerialProtocol() {
    // close the DE pin value file if it was opened
    if (m_deFd >= 0) {
        close(m_deFd);
    }
    // close the UART if it was opened
    if (m_fd >= 0) {
        close(m_fd);
    }
}

bool SerialProtocol::setUpMicrocontrollerSerial(const char* port, speed_t baud, int deGpio) {
    // open serial device (read/write, no controlling terminal)
    m_fd = open(port, O_RDWR | O_NOCTTY);
    // failed to open port
    if (m_fd < 0) {
        return false;
    }
    struct termios tty;
    // retrieve current terminal attributes
    if (tcgetattr(m_fd, &tty) != 0) {
        close(m_fd);
        m_fd = -1;
        return false;
    }
    // set input/output baud rate
    cfsetospeed(&tty, baud);
    cfsetispeed(&tty, baud);
    // configure 8 data bits, enable receiver, ignore modem control lines
    tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8 | CLOCAL | CREAD;
    // no parity, 1 stop bit, no hardware flow control
    tty.c_cflag &= ~(PARENB | CSTOPB | CRTSCTS);
    // disable software flow control and special handling of input bytes
    tty.c_iflag &= ~(IXON | IXOFF | IXANY | IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR | ICRNL);
    // disable canonical mode, echo, signals, and extended input processing
    tty.c_lflag &= ~(ECHO | ECHONL | ICANON | ISIG | IEXTEN);
    // disable output processing (raw output mode)
    tty.c_oflag &= ~OPOST;
    // read behavior: return what's there, or wait at most 0.1s if nothing is
    tty.c_cc[VMIN] = 0;
    tty.c_cc[VTIME] = 1;
    // apply configuration immediately
    if (tcsetattr(m_fd, TCSANOW, &tty) != 0) {
        close(m_fd);
        m_fd = -1;
        return false;
    }
    // throw away anything received before now
    tcflush(m_fd, TCIOFLUSH);

    // transceiver direction pin through sysfs
    if (deGpio >= 0) {
        std::string gpio = std::to_string(deGpio);
        std::string dir = "/sys/class/gpio/gpio" + gpio;
        // fails harmlessly if already exported
        writeFile("/sys/class/gpio/export", gpio);
        // make it an output, starting low = receive. right after export udev may not have set
        // the permissions on direction yet, so retry every 10 ms for up to 1 s
        bool isOutput = false;
        for (int attempt = 0; attempt < 100 && !isOutput; attempt++) {
            isOutput = writeFile(dir + "/direction", "low");
            if (!isOutput) {
                std::this_thread::sleep_for(std::chrono::milliseconds(10));
            }
        }
        // still not an output, so DE can't be driven
        if (!isOutput) {
            return false;
        }
        // keep the value file open so setDE() is a single write
        m_deFd = open((dir + "/value").c_str(), O_WRONLY);
        // GPIO missing or no permission
        if (m_deFd < 0) {
            return false;
        }
    }
    return true;
}

bool SerialProtocol::isValidPacketType(uint16_t type) {
    // the nano only receives what the brain sends
    return type == packets::STATE || type == packets::LOG;
}

int SerialProtocol::readBytes() {
    // free space left in m_rxBuffer
    size_t space = m_rxBuffer.size() - m_rxBufferPos;
    // port not opened yet, or no room
    if (m_fd < 0 || space == 0) {
        return 0;
    }
    // append to the end of the buffered data. waits up to 0.1s if nothing is there
    ssize_t n = read(m_fd, m_rxBuffer.data() + m_rxBufferPos, space);
    // timed out with nothing, or read failed
    if (n <= 0) {
        return 0;
    }
    // count the new bytes as buffered
    m_rxBufferPos += n;
    return static_cast<int>(n);
}

bool SerialProtocol::writeBytes(const uint8_t* data, size_t n) {
    // port not opened yet
    if (m_fd < 0) {
        return false;
    }
    // take the bus
    setDE(true);
    // keep writing until every byte is handed to the UART
    while (n > 0) {
        // write the rest of the frame to the UART
        ssize_t written = write(m_fd, data, n);
        // write failed, release the bus before giving up
        if (written <= 0) {
            setDE(false);
            return false;
        }
        // move past what was written
        data += written;
        n -= written;
    }
    // wait for the UART to finish sending
    tcdrain(m_fd);
    // hold the bus ~2 byte times before releasing it
    auto until = std::chrono::steady_clock::now() + std::chrono::microseconds(25);
    // busy-wait
    while (std::chrono::steady_clock::now() < until) {}
    // release the bus so the brain can talk
    setDE(false);
    return true;
}

void SerialProtocol::setDE(bool on) {
    // no DE pin with an auto-direction transceiver
    if (m_deFd >= 0) {
        // sysfs value: "1" = high = transmit, "0" = low = receive. pwrite at offset 0 so the file never needs a seek
        (void)pwrite(m_deFd, on ? "1" : "0", 1, 0);
    }
}
