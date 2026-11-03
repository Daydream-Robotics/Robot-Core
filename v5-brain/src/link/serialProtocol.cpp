/** Brain-side serial transport. */
#include "api.h"
#include "protocol/serialProtocol.hpp"

#include <algorithm>
#include <unistd.h>
#include "pros/apix.h"
#include "pros/error.h"
#include "pros/rtos.hpp"

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
    // if smart port:
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
    // if smart port:
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
