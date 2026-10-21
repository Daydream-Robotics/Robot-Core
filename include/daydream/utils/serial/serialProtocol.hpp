#ifndef SERIAL_PROTOCOL_HPP
#define SERIAL_PROTOCOL_HPP

#include <string>
#include <vector>
#include <optional>
#include <cstdint>
#include <cctype>
#include <cstdio>
#include <chrono>

#define __vexbrain__

#ifdef __vexbrain__
    #include "pros/apix.h"
#endif

#ifdef __microcontroller__
    #include <termios.h>
    #include <fcntl.h>
    #include <unistd.h>
#endif

//mode for sending data
class SerialProtocol {
public:
    enum class Mode {
        ASCII,
        BINARY
    };

    /*
        Packet format:
        TYPE,field1,field2;field1,field2;field1,field2\n
        Example:
        ODOM,x,10.5;y,5.2;theta,1.57\n
        ------------------------------------------------
        TYPE            -> packet type / command name
        ,               -> field separator
        field1,field2   -> one message block
        ;               -> separator between message blocks
        \n              -> end of packet
        ------------------------------------------------
    */
    struct Packet {
         // name/type of packet
        std::string type;
        //2d vector storing message data
        /*
            Outer vector: individual message blocks

            Inner vector: fields inside each block

            Example:
            {
                {"x", "10.5"},
                {"y", "5.2"},
                {"theta", "1.57"}
            }
            Represents: x,10.5;y,5.2;theta,1.57
        */
        std::vector<std::vector<std::string>> message;
    };

    // binary packet header
    #pragma pack(push, 1)
    struct BinaryHeader {
        uint16_t sync; // frame start marker
        uint16_t type; // message ID (MPC, telemetry, ...)
        uint16_t size; // payload size in bytes
    };
    #pragma pack(pop)

    //enum for packet type header
    enum PacketType : uint16_t {
        GENERIC     = 0,
        MPC_UPDATE  = 1,
        MPC_CONTROL = 2
    };

    //persistent buffer for binary
    std::vector<uint8_t> rx_buffer;
    size_t rx_buffer_pos;

    //constructor with defaults (separators are now fixed)
    SerialProtocol(int buffer_size, Mode mode = Mode::ASCII);

    // T must have a fixed memory layout
    template <typename T>
    //sends packet as binary or serialized string over serial
    bool send(const T& data);
    template <typename T>
    bool send(uint16_t type, const T& data);

     //recieves either a fixed size binary struct or an ASCII line  over serial and returns either a packet struct or nulpptr if full packet isn't recieved 
    template <typename T>
    std::optional<T> receive();
    template <typename T>
    std::optional<T> receive(uint16_t expected_type);

    //send raw wakeup string to serial
    bool sendWakeup(const std::string& wakeup);

    #ifdef __microcontroller__
    static int setUpMicrocontrollerSerial(const char* port, speed_t baud = B115200);
    #endif

    #ifdef __vexbrain__
    static void setUpVexSerial();
    #endif

private:
    int buffer_size;
    Mode mode;

    //check if the packet is a valid type
    static bool isValidPacketType(const std::string& type);

    // T must be trivially copyable ie no ptrs in the struct
    template <typename T>
    //sends struct as raw binary over serial
    bool sendBinary(uint16_t type, const T& packet);

    template <typename T>
    // Receives a fixed-size binary struct from serial and returns nullptr if full packet isn't recieved
    std::optional<T> receiveBinary(uint16_t expectedType);

    //serializes packet into string then sends it over USB serial
    bool sendASCII(const Packet& packet);
    //reads one line from USB serial then parses into a packet object, returns std::nullopt if no packet is recieved
    std::optional<Packet> receiveASCII();

    uint8_t crc8(const uint8_t* data, size_t len);
    bool readBytes(void* dest, size_t n, int timeout_ms = 100);

    //converts packet struct to raw string packet to send
    static std::string serializePacket(const Packet& packet);
    //converts raw packet string (from serial) to a packet struct
    static std::optional<Packet> deserializePacket(const std::string& line);

    //function to skip bytes
    inline bool skipBytes(size_t num_bytes) {
        //buffer to hold discarded data
        char discard[256];
        //continue until all requested bytes have been discarded
        size_t remaining = num_bytes;
        //calc num bytes to read but not more than buffer size
        while (remaining > 0) {
            size_t to_read = std::min(remaining, sizeof(discard));
            //put bytes into discard buffer and are ignored
            if (!readBytes(discard, to_read, 100)) {
                //hit EOF or an error
                return false;
            }
            remaining -= to_read;
        }
        return true;
    }
};


//template functions
//unified send without a type (ie used more for ASCI or with the generic type 0)
template <typename T>
bool SerialProtocol::send(const T& data) {
    if (mode == Mode::ASCII) {
        if constexpr (std::is_same_v<T, Packet>) {
            return sendASCII(data);
        }
        return false;
    }
    if (mode == Mode::BINARY) {
        if constexpr (std::is_trivially_copyable_v<T>) {
            return sendBinary(0, data);
        }
        return false;
    }
    return false;
}

//unified send with a type (ie used more for Binary)
template <typename T>
bool SerialProtocol::send(uint16_t type, const T& data) {
    if (mode == Mode::ASCII) {
        if constexpr (std::is_same_v<T, Packet>) {
            return sendASCII(data);
        }
        return false;
    }
    if (mode == Mode::BINARY) {
        if constexpr (std::is_trivially_copyable_v<T>) {
            return sendBinary(type, data);
        }
        return false;
    }
    return false;
}

//unified recieve (used more for ASCII or looking for type 0)
template <typename T>
std::optional<T> SerialProtocol::receive() {
    if (mode == Mode::ASCII) {
        if constexpr (std::is_same_v<T, Packet>) {
            return receiveASCII();
        }
        return std::nullopt;
    }
    if (mode == Mode::BINARY) {
        if constexpr (std::is_trivially_copyable_v<T>) {
            return receiveBinary<T>(0);
        }
        return std::nullopt;
    }
    return std::nullopt;
}

//unified recieve (used more for Binary)
template <typename T>
std::optional<T> SerialProtocol::receive(uint16_t expected_type) {
    if (mode == Mode::ASCII) {
        if constexpr (std::is_same_v<T, Packet>) {
            return receiveASCII();
        }
        return std::nullopt;
    }
    if (mode == Mode::BINARY) {
        if constexpr (std::is_trivially_copyable_v<T>) {
            return receiveBinary<T>(expected_type);
        }
        return std::nullopt;
    }
    return std::nullopt;
}

//sends struct as raw binary over serial 
// Packet format: [0x55][0xAA][type_lo][type_hi][size_lo][size_hi][payload][CRC-8]
template <typename T>
bool SerialProtocol::sendBinary(uint16_t type, const T& packet) {
    //ensure type can be safley used as raw bytes
    static_assert(std::is_trivially_copyable_v<T>,
                  "Binary packet must be trivially copyable");
    //makes a header and sets:
    uint8_t header[6];
    //sync bytes
    header[0] = 0x55;
    header[1] = 0xAA;
    //type bytes
    header[2] = type & 0xFF;
    header[3] = (type >> 8) & 0xFF;
    //size bytes
    header[4] = sizeof(T) & 0xFF;
    header[5] = (sizeof(T) >> 8) & 0xFF;

    //writes header to USB serial and checks if it was properly written
    if (fwrite(header, 1, 6, stdout) != 6) {
        return false;
    }
    //Interpret struct as a raw byte array
    const uint8_t* raw = reinterpret_cast<const uint8_t*>(&packet);
    //writes message to USB serial and checks if it was properly written
    if (fwrite(raw, 1, sizeof(T), stdout) != sizeof(T)) {
        return false;
    }
    //computes CRC
    uint8_t crc = crc8(raw, sizeof(T));
    //writes crc byte to USB serial and checks if it was properly written
    if (fwrite(&crc, 1, 1, stdout) != 1) {
        return false;
    }
    //force immediate writing
    fflush(stdout);
    return true;
}

// Receives a fixed-size binary struct from serial and returns nullptr if full packet isn't recieved
template <typename T>
std::optional<T> SerialProtocol::receiveBinary(uint16_t expectedType) {
    static_assert(std::is_trivially_copyable_v<T>,
                  "Binary packet must be trivially copyable");
    
    const int timeout_ms = 2000;
    const auto deadline = std::chrono::steady_clock::now() 
                        + std::chrono::milliseconds(timeout_ms);

    BinaryHeader header;
    uint8_t byte;
    int sync_found = 0;
    int attempts   = 0;
    const int max_attempts = 10000;

    // Scan for sync bytes 0x55 0xAA
    while (sync_found < 2 && attempts < max_attempts) {
        if (std::chrono::steady_clock::now() > deadline) {
            return std::nullopt;
        }
        if (!readBytes(&byte, 1, 100)) {
            return std::nullopt;
        }
        if (sync_found == 0 && byte == 0x55) {
            sync_found = 1;
        } else if (sync_found == 1 && byte == 0xAA) {
            sync_found = 2;
        } else {
            sync_found = (byte == 0x55) ? 1 : 0;
        }
        attempts++;
    }

    if (attempts >= max_attempts) {
        return std::nullopt;
    }

    // Read type + size
    if (!readBytes(reinterpret_cast<uint8_t*>(&header) + 2, 
                   sizeof(BinaryHeader) - 2, 100)) {
        return std::nullopt;
    }

    // Validate packet type
    if (header.type != expectedType) {
        skipBytes(header.size + 1);
        return std::nullopt;
    }

    // Validate payload size
    if (header.size != sizeof(T)) {
        skipBytes(header.size + 1);
        return std::nullopt;
    }

    // Prevent buffer overflow
    if (header.size > (size_t)buffer_size || header.size == 0) {
        skipBytes(header.size + 1);
        return std::nullopt;
    }

    // Read payload
    T result;
    if (!readBytes(&result, sizeof(T), 100)) {
        return std::nullopt;
    }

    // Read and verify CRC
    uint8_t recv_crc;
    if (!readBytes(&recv_crc, 1, 100)) {
        return std::nullopt;
    }

    uint8_t computed_crc = crc8(reinterpret_cast<const uint8_t*>(&result), 
                                sizeof(T));
    if (computed_crc != recv_crc) {
        return std::nullopt;
    }

    return result;
}

//compute CRC-8 (0x07) over a byte buffer
//used to detect corruption over USB serial
inline uint8_t SerialProtocol::crc8(const uint8_t* data, size_t len) {
    uint8_t crc = 0;
    //process each byte in buffer
    for (size_t i = 0; i < len; i++) {
        //xor current byte into CRC register
        crc ^= data[i];
        //process all the current bytes
        for (int b = 0; b < 8; b++) {
            // If most sig bit (bit 7) is set, shift left and apply polynomial
            crc = (crc & 0x80) ? (crc << 1) ^ 0x07 : crc << 1;
        }
    }
    return crc;
}

#ifdef __microcontroller__
// Configures a serial port for microcontroller communication
inline int SerialProtocol::setUpMicrocontrollerSerial(const char* port, speed_t baud) {
    //open serial device (read/write, no controlling terminal, synchronous I/O)
    int fd = open(port, O_RDWR | O_NOCTTY | O_SYNC);
    //failed to open port
    if (fd < 0) {
        return -1;
    }
    struct termios tty;
    //retrieve current terminal attributes
    if(tcgetattr(fd, &tty) != 0) {
        return -1;
    }
    //set input/output baud rate
    cfsetospeed(&tty, baud);
    cfsetispeed(&tty, baud);
    // Configure 8 data bits, enable receiver, ignore modem control lines
    tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8 | CLOCAL | CREAD;
    //disable software flow control and special handling of input bytes
    tty.c_iflag &= ~(IXON | IXOFF | IXANY | IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR | ICRNL);
    //disable canonical mode, echo, signals, and extended input processing
    tty.c_lflag &= ~(ECHO | ECHONL | ICANON | ISIG | IEXTEN);
    //disable output processing (raw output mode)
    tty.c_oflag &= ~OPOST;
    //set read behavior: minimum 1 byte, timeout = 0.1s
    tty.c_cc[VMIN] = 1; 
    tty.c_cc[VTIME] = 1;
    //apply configuration immediately
    tcsetattr(fd, TCSANOW, &tty);
    //redirect standard input/output to the serial port
    dup2(fd, STDIN_FILENO);
    dup2(fd, STDOUT_FILENO);
    setbuf(stdin, NULL);
    setbuf(stdout, NULL);
    setvbuf(stdin, NULL, _IONBF, 0);
    setvbuf(stdout, NULL, _IONBF, 0);
    //return file descriptor for later use
    return fd;
}
#endif

#ifdef __vexbrain__
//configures serial settings for vex brain serial communication
inline void SerialProtocol::setUpVexSerial() {
    //disable COBS encoding in PROS serial layer (raw byte streaming mode)
    pros::c::serctl(SERCTL_DISABLE_COBS, nullptr);
    //disable buffering for stdout to ensure immediate transmission
    setbuf(stdout, NULL);
    //disable buffering for stdin to ensure immediate reception
    setbuf(stdin, NULL);
}
#endif



#endif