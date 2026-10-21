/**
 * @file serialProtocol.hpp
 * @brief Framing layer for the V5 Brain ⇄ Jetson Nano serial link.
 *
 * Turns payloads into frames and frames back into payloads
 * Both sides compile the same file. The
 * only difference is the platform define below.
 *
 * Frame format: [0x55][0xAA][type u16][size u16][payload][CRC-16 u16], all little-endian.
 */
#pragma once

#include <atomic>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <optional>
#include <vector>

#include "daydream/utils/serial/packets.hpp"

#define __vexbrain__

#ifdef __vexbrain__
    #include "pros/apix.h"
#endif

#ifdef __microcontroller__
    #include <termios.h>
#endif

/**
 * @file serialProtocol.hpp
 * @brief Framing layer for the V5 Brain ⇄ Jetson Nano serial link.
 *
 * Turns payloads into frames and frames back into payloads
 * (SERIAL_PROTOCOL.md sections 2-3). Both sides compile the same file. The
 * only difference is the platform define below.
 *
 * Frame format: [0x55][0xAA][type u16][size u16][payload][CRC-16 u16], all little-endian.
 */

#pragma once

#include <atomic>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <optional>
#include <vector>

#include "daydream/utils/serial/packets.hpp"

#define __vexbrain__   // Nano copy: #define __microcontroller__   ← the ONLY difference

#ifdef __vexbrain__
    #include "pros/serial.hpp"
#endif

#ifdef __microcontroller__
    #include <termios.h>
#endif

/**
 * @brief Sends and receives framed packets over the RS-485 link.
 *
 * receive() is only called by the reader and send() only by the writer. They
 * share no buffers, so the class needs no lock. midFrame() and the error
 * counters are atomic because the other thread reads them.
 */
class SerialProtocol {
public:
    #pragma pack(push, 1)
    /// @brief Binary packet header
    struct BinaryHeader {
        uint16_t sync; ///< frame start marker (0x55 0xAA)
        uint16_t type; ///< packets::PacketType
        uint16_t size; ///< payload size in bytes
    };
    #pragma pack(pop)

    /// @brief One received frame that passed every check.
    struct Frame {
        uint16_t type; ///< packets::PacketType
        uint16_t size; ///< payload size in bytes
        uint8_t payload[packets::MAX_PAYLOAD]; ///< payload bytes. only the first `size` are valid
    };

    static constexpr size_t HEADER_SIZE = sizeof(BinaryHeader); ///< bytes before the payload
    static constexpr size_t CRC_SIZE = 2; ///< bytes after the payload
    static constexpr size_t MAX_FRAME_SIZE = HEADER_SIZE + packets::MAX_PAYLOAD + CRC_SIZE; ///< largest possible frame

    #ifdef __microcontroller__
    static constexpr speed_t DEFAULT_BAUD = B921600; ///< link baud rate
    #endif

    std::atomic<uint16_t> crc_errors{0}; ///< frames dropped for a bad CRC. reported in STATE.rx_errors.
    std::atomic<uint16_t> bad_frames{0}; ///< frames dropped for a bad type or size. reported in STATE.rx_errors

    /**
     * @brief Constructs the protocol without opening a port.
     *
     * Call setUpVexSerial() or setUpMicrocontrollerSerial() afterwards.
     *
     * @param buffer_size Receive buffer size in bytes. Raised to MAX_FRAME_SIZE if smaller.
     */
    SerialProtocol(int buffer_size = 2 * MAX_FRAME_SIZE);


    /// @brief Closes the port.
    ~SerialProtocol();

    #ifdef __vexbrain__
    /**
     * @brief Opens a smart port as a generic serial port for the link.
     *
     * @param port Smart port number, 1–21.
     * @param baud Baud rate, e.g. 921600.
     * @return bool false if the port number is bad or the port is in use.
     */
    bool setUpVexSerial(uint8_t port, int32_t baud);
    #endif

    #ifdef __microcontroller__
    /**
     * @brief Opens and configures the UART for the link.
     *
     * Raw 8N1, no flow control. read() returns what is available, or waits at
     * most 100 ms if nothing is.
     *
     * @param port UART device, e.g. "/dev/ttyTHS1".
     * @param baud termios speed constant, e.g. B921600.
     * @param de_gpio sysfs GPIO number wired to the transceiver's DE and /RE pins.
     *        -1 = auto-direction transceiver, no pin to drive.
     * @return bool False if the port or the GPIO couldn't be opened.
     */
    bool setUpMicrocontrollerSerial(const char* port, speed_t baud = DEFAULT_BAUD, int de_gpio = -1);
    #endif


    /**
     * @brief Sends one frame: header, payload and CRC-16 in a single write.
     *
     * @param type packets::PacketType.
     * @param payload Payload bytes.
     * @param size Payload size in bytes, at most packets::MAX_PAYLOAD.
     * @return bool false if the payload is too large or the write failed.
     */
    bool send(uint16_t type, const void* payload, uint16_t size);
    
    /**
     * @brief Sends a payload built with packets::PayloadWriter.
     *
     * @param type packets::PacketType.
     * @param writer Writer holding the payload. Its byte count is the payload size.
     * @return bool false if the payload is too large or the write failed.
     */
    bool send(uint16_t type, const packets::PayloadWriter& writer) {
        return send(type, writer.buf, static_cast<uint16_t>(writer.n));
    }

    /**
     * @brief Returns the next valid frame, if a full one has arrived.
     *
     * Reads whatever bytes are available, then parses rx_buffer. Frames with
     * the wrong direction, a bad size or a bad CRC are dropped and counted.
     * Returns at most one frame per call, so call it in a loop until it
     * returns std::nullopt.
     *
     * Brain: never blocks. Nano: blocks up to 100 ms waiting for bytes.
     *
     * @return std::optional<Frame> The frame, or std::nullopt if no full valid frame is buffered.
     */
    std::optional<Frame> receive();

    /**
     * @brief Whether a frame is partway received.
     *
     * @return bool true if rx_buffer starts with a sync byte.
     */
    bool midFrame() const { 
        return in_frame; 
    }

    /**
     * @brief Sets the group count used for the exact COMMAND size check.
     *
     * @param groups Group count of the applied config. 0 = not configured.
     */
    void setCommandGroups(uint8_t groups) { 
        command_groups = groups; 
    }

    /**
     * @brief Computes CRC-16/CCITT-FALSE (poly 0x1021, init 0xFFFF).
     *
     * Check value: crc16("123456789") == 0x29B1.
     *
     * @param data Bytes to checksum. For a frame: type + size + payload.
     * @param len Number of bytes.
     * @param crc Starting value, to continue a previous CRC.
     * @return uint16_t The CRC.
     */
    static uint16_t crc16(const uint8_t* data, size_t len, uint16_t crc = 0xFFFF);

private:
    int buffer_size;  ///< receive buffer size in bytes
    std::vector<uint8_t> rx_buffer; ///< persistent receive buffer
    size_t rx_buffer_pos; ///< bytes currently in rx_buffer

    std::atomic<bool> in_frame{false}; ///< see midFrame()
    uint8_t command_groups = 0; ///< see setCommandGroups()

    /**
     * @brief Whether this side accepts a packet type.
     *
     * Brain: CONFIG and COMMAND. Nano: STATE and LOG.
     *
     * @param type Header `type` field.
     * @return bool True if the type is sent by the other side.
     */
    static bool isValidPacketType(uint16_t type);

    /**
     * @brief Moves the bytes available on the port into rx_buffer.
     *
     * @return int number of bytes read. 0 if none, or if rx_buffer is full.
     */
    int readBytes();

    /**
     * @brief Writes all bytes to the port.
     *
     * @param data Bytes to write.
     * @param n Number of bytes.
     * @return bool false if the port failed partway.
     */
    bool writeBytes(const uint8_t* data, size_t n);

    /**
     * @brief Removes bytes from the front of rx_buffer.
     *
     * @param num_bytes Number of bytes to remove. Clamped to rx_buffer_pos.
     */
    void skipBytes(size_t num_bytes);

    #ifdef __vexbrain__
    std::unique_ptr<pros::Serial> port; ///< link smart port. null until setUpVexSerial()
    #endif

    #ifdef __microcontroller__
    int fd = -1; ///< UART file descriptor. -1 until setUpMicrocontrollerSerial()
    int de_fd = -1; ///< sysfs value file of the DE GPIO. -1 = none

    /**
     * @brief Drives the transceiver direction pin.
     *
     * @param on True = transmit, false = receive.
     */
    void setDE(bool on);
    #endif
};


