/**
 * @file serialProtocol.cpp
 * @brief framing for the brain ⇄ Nano link. shared code first, then one block per platform
 */

#include "daydream/utils/serial/serialProtocol.hpp"

#include <algorithm>
#include <cstring>

#ifdef __v5_brain__
    #include <unistd.h>
    #include "pros/apix.h"
    #include "pros/error.h"
    #include "pros/rtos.hpp"
#endif

#ifdef __jetson_nano__
    #include <chrono>
    #include <string>
    #include <fcntl.h>
    #include <unistd.h>
#endif

static constexpr uint8_t SYNC_0 = 0x55; ///< first sync byte of every frame
static constexpr uint8_t SYNC_1 = 0xAA; ///< second sync byte of every frame

SerialProtocol::SerialProtocol(int bufferSize)
    : m_bufferSize(std::max<int>(bufferSize, MAX_FRAME_SIZE)),
      m_rxBuffer(m_bufferSize),
      m_rxBufferPos(0) {}

bool SerialProtocol::send(uint16_t type, const void* payload, uint16_t size) {
    // payload too big for one frame
    if (size > packets::MAX_PAYLOAD) {
        return false;
    }
    // build the whole frame in one buffer so it goes out in one write with no gaps
    uint8_t frame[MAX_FRAME_SIZE];
    // sync bytes
    frame[0] = SYNC_0;
    frame[1] = SYNC_1;
    // type bytes
    frame[2] = type & 0xFF;
    frame[3] = (type >> 8) & 0xFF;
    // size bytes
    frame[4] = size & 0xFF;
    frame[5] = (size >> 8) & 0xFF;
    // payload
    std::memcpy(frame + HEADER_SIZE, payload, size);
    // CRC over type + size + payload (not the sync bytes)
    uint16_t crc = crc16(frame + 2, HEADER_SIZE - 2 + size);
    frame[HEADER_SIZE + size] = crc & 0xFF;
    frame[HEADER_SIZE + size + 1] = (crc >> 8) & 0xFF;

    return writeBytes(frame, HEADER_SIZE + size + CRC_SIZE);
}

std::optional<SerialProtocol::Frame> SerialProtocol::receive() {
    // a frame may already be buffered, e.g. a STATE that arrived with a LOG
    // parse before reading
    if (std::optional<Frame> frame = parseFrame()) {
        return frame;
    }
    // get new bytes and try again
    readBytes();
    return parseFrame();
}

std::optional<SerialProtocol::Frame> SerialProtocol::parseFrame() {
    std::optional<Frame> result;
    while (m_rxBufferPos >= 2) {
        const uint8_t* buf = m_rxBuffer.data();

        // scan for sync bytes 0x55 0xAA
        if (buf[0] != SYNC_0 || buf[1] != SYNC_1) {
            skipBytes(1);
            continue;
        }
        // wait for the rest of the header
        if (m_rxBufferPos < HEADER_SIZE) {
            break;
        }

        // read/copy the header
        BinaryHeader header;
        std::memcpy(&header, buf, HEADER_SIZE);

        // validate packet type and rough size
        // on any failure skip 1 byte
        if (!isValidPacketType(header.type) || !packets::sizeInRange(header.type, header.size)) {
            m_badFrames++;
            skipBytes(1);
            continue;
        }

        // wait for the payload and CRC
        size_t frameSize = HEADER_SIZE + header.size + CRC_SIZE;
        if (m_rxBufferPos < frameSize) {
            break;
        }

        const uint8_t* payload = buf + HEADER_SIZE;

        // validate exact payload size
        if (!packets::sizeExact(header.type, payload, header.size, m_commandGroups)) {
            m_badFrames++;
            skipBytes(1);
            continue;
        }

        // read and verify CRC
        uint16_t recvCrc = payload[header.size] | (payload[header.size + 1] << 8);
        uint16_t computedCrc = crc16(buf + 2, HEADER_SIZE - 2 + header.size);
        if (computedCrc != recvCrc) {
            m_crcErrors++;
            skipBytes(1);
            continue;
        }

        // valid frame so copy it out and remove it from buffer
        // construct a Frame inside the optional
        result.emplace();
        result->type = header.type;
        result->size = header.size;
        // copy before skipBytes moves the data
        std::memcpy(result->payload, payload, header.size);
        // now remove the whole frame
        skipBytes(frameSize);
        break;
    }

    // a frame is partway in if the buffer starts with a sync byte
    m_inFrame = m_rxBufferPos >= 1 && m_rxBuffer[0] == SYNC_0;
    return result;
}

uint16_t SerialProtocol::crc16(const uint8_t* data, size_t len, uint16_t crc) {
    // process each byte in buffer
    for (size_t i = 0; i < len; i++) {
        // xor current byte into the top of the CRC register
        crc ^= static_cast<uint16_t>(data[i]) << 8;
        // process all the bits of the current byte
        for (int b = 0; b < 8; b++) {
            // if most sig bit (bit 15) is set, shift left and apply polynomial
            crc = (crc & 0x8000) ? (crc << 1) ^ 0x1021 : crc << 1;
        }
    }
    return crc;
}

void SerialProtocol::skipBytes(size_t numBytes) {
    numBytes = std::min(numBytes, m_rxBufferPos);
    // shift remaining buffer data to the front
    std::memmove(m_rxBuffer.data(), m_rxBuffer.data() + numBytes, m_rxBufferPos - numBytes);
    m_rxBufferPos -= numBytes;
}


#ifdef __v5_brain__

SerialProtocol::~SerialProtocol() = default;

bool SerialProtocol::setUpVexSerial(uint8_t portNumber, int32_t baud) {
    m_port = std::make_unique<pros::Serial>(portNumber, baud);
    // PROS_ERR here means the port number is bad or the port is in use
    return m_port->get_read_avail() != PROS_ERR;
}

bool SerialProtocol::setUpVexSerial() {
    // disable COBS encoding in the PROS serial layer (raw byte streaming mode), as the previous protocol did
    if (pros::c::serctl(SERCTL_DISABLE_COBS, nullptr) == PROS_ERR) {
        // COBS still on: the Nano would get COBS-wrapped bytes it can't parse
        return false;
    }
    m_usb = true;
    return true;
}

bool SerialProtocol::isValidPacketType(uint16_t type) {
    return type == packets::CONFIG || type == packets::COMMAND;
}

int SerialProtocol::readBytes() {
    // if USB read straight from stdin
    if (m_usb) {
        // free space left in m_rxBuffer
        size_t space = m_rxBuffer.size() - m_rxBufferPos;
        // PROS writes a 0 one byte past the end of what it reads, so leave a spare byte
        if (space < 2) {
            return 0;
        }
        // waits until at least one byte arrives, then returns what's there
        ssize_t n = ::read(STDIN_FILENO, m_rxBuffer.data() + m_rxBufferPos, space - 1);
        if (n <= 0) {
            return 0;
        }
        // count the new bytes as buffered
        m_rxBufferPos += n;
        return static_cast<int>(n);
    }
    //if smart port:
    // port not opened yet
    if (!m_port) {
        return 0;
    }
    // free space left in m_rxBuffer
    size_t space = m_rxBuffer.size() - m_rxBufferPos;
    // bytes waiting in the PROS receive buffer
    int32_t available = m_port->get_read_avail();
    // nothing to read, no room, or the port errored
    if (space == 0 || available <= 0 || available == PROS_ERR) {
        return 0;
    }
    // read only what's available and fits
    int32_t toRead = std::min<int32_t>(available, static_cast<int32_t>(space));
    // append to the end of the buffered data
    int32_t read = m_port->read(m_rxBuffer.data() + m_rxBufferPos, toRead);
    // read failed
    if (read <= 0 || read == PROS_ERR) {
        return 0;
    }
    // count the new bytes as buffered
    m_rxBufferPos += read;
    return read;
}

bool SerialProtocol::writeBytes(const uint8_t* data, size_t n) {
    // if USB write straight to stdout
    if (m_usb) {
        // keep writing until every byte is handed to PROS
        while (n > 0) {
            // write the rest of the frame to stdout in one call
            ssize_t written = ::write(STDOUT_FILENO, data, n);
            // write failed
            if (written <= 0) {
                return false;
            }
            // move past what was written
            data += written;
            n -= written;
        }
        return true;
    }
    //if smart port:
    // port not opened yet
    if (!m_port) {
        return false;
    }
    // keep writing until every byte is handed to PROS
    while (n > 0) {
        // space left in the PROS transmit buffer
        int32_t freeSpace = m_port->get_write_free();
        // port errored
        if (freeSpace == PROS_ERR) {
            return false;
        }
        // transmit buffer full so give 1 ms to drain
        if (freeSpace <= 0) {
            pros::delay(1);
            continue;
        }
        // write only what fits right now
        int32_t toWrite = std::min<int32_t>(freeSpace, static_cast<int32_t>(n));
        // cast away const to pass the bytes to the smart port
        int32_t written = m_port->write(const_cast<uint8_t*>(data), toWrite);
        // write failed
        if (written <= 0 || written == PROS_ERR) {
            return false;
        }
        // move past what was written
        data += written;
        n -= written;
    }
    return true;
}

#endif

#ifdef __jetson_nano__

/**
 * @brief writes a short string to a sysfs file
 *
 * @param path file to write, e.g. "/sys/class/gpio/export"
 * @param text text to write. failures are ignored
 */
static void writeFile(const std::string& path, const std::string& text) {
    // open for writing only
    int file = open(path.c_str(), O_WRONLY);
    // file doesn't exist or no permission
    if (file < 0) {
        return;
    }
    // sysfs takes the whole value in one write
    (void)write(file, text.data(), text.size());
    close(file);
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
        // output, starting low = receive
        writeFile(dir + "/direction", "low");
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
#endif