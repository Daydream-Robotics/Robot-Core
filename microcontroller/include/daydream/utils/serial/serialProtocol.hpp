/**
 * @file serialProtocol.hpp
 * @brief framing layer for the V5 Brain ⇄ Jetson Nano serial link
 *
 * turns payloads into frames and frames back into payloads
 * both sides compile the same file. the
 * only difference is the platform define below
 *
 * Frame format: [0x55][0xAA][type u16][size u16][payload][CRC-16 u16], all little-endian
 */
#pragma once

#include <atomic>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <optional>
#include <vector>

#include "daydream/utils/serial/packets.hpp"

// ! PLATFORM NOTE PUT THIS IN CONSTANTS.H and include it so that its the smae on nano vs brain
#define __jetson_nano__   

#ifdef __v5_brain__
    #include "pros/serial.hpp"
#endif

#ifdef __jetson_nano__
    #include <termios.h>
#endif

class SerialLink;

/**
 * @brief sends and receives framed packets over the RS-485 link or USB link
 *
 * only SerialLink uses it: everything except Transport and crc16() is private,
 * and SerialLink is a friend. use SerialLink to talk to the other side
 *
 * receive() is only called by the reader and send() only by the writer. they
 * share no buffers, so the class needs no lock. midFrame() and the error
 * counters are atomic because the other thread reads them
 */
class SerialProtocol {
public:
    /// @brief which physical link the protocol uses
    enum class Transport : uint8_t {
        RS485, ///< brain smart port <-> Nano UART, through RS-485 transceivers
        USB ///< brain USB port (PROS stdin/stdout) <-> Nano /dev/ttyACM1
    };

    /// @brief closes the port
    ~SerialProtocol();

    /**
     * @brief computes CRC-16/CCITT-FALSE (poly 0x1021, init 0xFFFF)
     *
     * check value: crc16("123456789") == 0x29B1
     *
     * @param data bytes to checksum. for a frame, type + size + payload
     * @param len number of bytes
     * @param crc starting value, to continue a previous CRC
     * @return uint16_t the CRC
     */
    static uint16_t crc16(const uint8_t* data, size_t len, uint16_t crc = 0xFFFF);

private:
    friend class SerialLink;

    #pragma pack(push, 1)
    /// @brief binary packet header
    struct BinaryHeader {
        uint16_t sync; ///< frame start marker (0x55 0xAA)
        uint16_t type; ///< packets::PacketType
        uint16_t size; ///< payload size in bytes
    };
    #pragma pack(pop)

    /// @brief one received frame that passed every check
    struct Frame {
        uint16_t type; ///< packets::PacketType
        uint16_t size; ///< payload size in bytes
        uint8_t payload[packets::MAX_PAYLOAD]; ///< payload bytes
    };

    static constexpr size_t HEADER_SIZE = sizeof(BinaryHeader); ///< bytes before the payload
    static constexpr size_t CRC_SIZE = 2; ///< bytes after the payload
    static constexpr size_t MAX_FRAME_SIZE = HEADER_SIZE + packets::MAX_PAYLOAD + CRC_SIZE; ///< largest possible frame

    #ifdef __jetson_nano__
    static constexpr speed_t DEFAULT_BAUD = B921600; ///< link baud rate
    #endif

    std::atomic<uint16_t> m_crcErrors{0}; ///< frames dropped for a bad CRC, reported in STATE.rxErrors
    std::atomic<uint16_t> m_badFrames{0}; ///< frames dropped for a bad type or size, reported in STATE.rxErrors

    /**
     * @brief constructs the protocol without opening a port
     *
     * call setUpVexSerial() or setUpMicrocontrollerSerial() afterwards
     *
     * @param bufferSize receive buffer size in bytes. raised to MAX_FRAME_SIZE if smaller
     */
    SerialProtocol(int bufferSize = 2 * MAX_FRAME_SIZE);


    #ifdef __v5_brain__
    /**
     * @brief opens a smart port as a generic serial port for the link
     *
     * @param portNumber smart port number, 1–21
     * @param baud baud rate, e.g. 921600
     * @return bool false if the port number is bad or the port is in use
     */
    bool setUpVexSerial(uint8_t portNumber, int32_t baud);

    /**
     * @brief uses the brain's USB port (PROS stdin/stdout) for the link
     *
     * turns off PROS's COBS stream framing so frames go out as raw bytes
     *
     * @return bool false if PROS couldn't turn off COBS
     */
    bool setUpVexSerial();
    #endif

    #ifdef __jetson_nano__
    /**
     * @brief opens and configures the UART for the link
     *
     * no flow control. read() returns what is available, or waits at
     * most 100 ms if nothing is
     *
     * @param port UART device: "/dev/ttyTHS1" for RS-485, or "/dev/ttyACM1" for the brain's USB port
     * @param baud termios speed constant, e.g. B921600
     * @param deGpio sysfs GPIO number wired to the transceiver's DE and /RE pins.
     *        -1 = auto-direction transceiver or USB, no pin to drive
     * @return bool false if the port or the GPIO couldn't be opened
     */
    bool setUpMicrocontrollerSerial(const char* port, speed_t baud = DEFAULT_BAUD, int deGpio = -1);
    #endif


    /**
     * @brief sends one frame with header, payload and CRC-16
     *
     * @param type packets::PacketType
     * @param payload payload bytes
     * @param size payload size in bytes, at most packets::MAX_PAYLOAD
     * @return bool false if the payload is too large or the write failed
     */
    bool send(uint16_t type, const void* payload, uint16_t size);
    
    /**
     * @brief sends a payload built with packets::PayloadWriter
     *
     * @param type packets::PacketType
     * @param writer writer holding the payload. its byte count is the payload size
     * @return bool false if the payload is too large or the write failed
     */
    bool send(uint16_t type, const packets::PayloadWriter& writer) {
        return send(type, writer.buf, static_cast<uint16_t>(writer.n));
    }

    /**
     * @brief returns the next valid frame, if a full one has arrived
     *
     * parses m_rxBuffer first, and only reads the port if no full frame is
     * already buffered. frames with the wrong direction, a bad size or a bad
     * CRC are dropped and counted.
     * returns at most one frame per call, so call it in a loop until it
     * returns std::nullopt
     *
     * Brain: doesn't blocks. Nano: blocks up to 100 ms waiting for bytes
     *
     * @return std::optional<Frame> the frame, or std::nullopt if no full valid frame is buffered
     */
    std::optional<Frame> receive();

    /**
     * @brief whether a frame is partway received
     *
     * @return bool true if m_rxBuffer starts with a sync byte
     */
    bool midFrame() const { 
        return m_inFrame; 
    }

    /**
     * @brief sets the group count used for the exact COMMAND size check
     *
     * @param groups Group count of the applied config. 0 = not configured
     */
    void setCommandGroups(uint8_t groups) { 
        m_commandGroups = groups; 
    }

    int m_bufferSize;  ///< receive buffer size in bytes
    std::vector<uint8_t> m_rxBuffer; ///< persistent receive buffer
    size_t m_rxBufferPos; ///< bytes currently in m_rxBuffer

    std::atomic<bool> m_inFrame{false}; ///< see midFrame()
    uint8_t m_commandGroups = 0; ///< see setCommandGroups()

    /**
     * @brief whether this side accepts a packet type
     *
     * Brain: CONFIG and COMMAND. Nano: STATE and LOG
     *
     * @param type header `type` field
     * @return bool true if the type is sent by the other side
     */
    static bool isValidPacketType(uint16_t type);

    /**
     * @brief parses the next valid frame out of m_rxBuffer, without reading the port
     *
     * @return std::optional<Frame> the frame, or std::nullopt if no full valid frame is buffered
     */
    std::optional<Frame> parseFrame();

    /**
     * @brief moves the bytes available on the port into m_rxBuffer
     *
     * @return int number of bytes read. 0 if none, or if m_rxBuffer is full
     */
    int readBytes();

    /**
     * @brief writes all bytes to the port
     *
     * @param data bytes to write
     * @param n number of bytes
     * @return bool false if the port failed partway
     */
    bool writeBytes(const uint8_t* data, size_t n);

    /**
     * @brief removes bytes from the front of m_rxBuffer
     *
     * @param numBytes number of bytes to remove. clamped to m_rxBufferPos
     */
    void skipBytes(size_t numBytes);

    #ifdef __v5_brain__
    std::unique_ptr<pros::Serial> m_port; ///< link smart port. null over USB, or until setUpVexSerial()
    bool m_usb = false; ///< the link runs over USB (stdin/stdout) instead of m_port
    #endif

    #ifdef __jetson_nano__
    int m_fd = -1; ///< UART file descriptor. -1 until setUpMicrocontrollerSerial()
    int m_deFd = -1; ///< sysfs value file of the DE GPIO. -1 = none

    /**
     * @brief drives the transceiver direction pin
     *
     * @param on true = transmit, false = receive
     */
    void setDE(bool on);
    #endif
};


