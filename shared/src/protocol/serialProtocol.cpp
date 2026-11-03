/** Platform-independent framing and parser. */
#ifdef __v5_brain__
#include "api.h"
#endif
#include "protocol/serialProtocol.hpp"

#include <algorithm>
#include <cstring>

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
    // a frame may already be buffered, e.g. a state that arrived with a log
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
