#ifndef SERIAL_PROTOCOL_HPP
#define SERIAL_PROTOCOL_HPP

#include <string>
#include <vector>
#include <optional>
#include <cstdint>
#include <cctype>


class SerialProtocol {
public:
    //mode for sending data
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
    struct BinaryHeader {
        uint16_t sync = 0xAA55;   // frame start marker
        uint16_t type;            // message ID (MPC, telemetry, ...)
        uint16_t size;            // payload size in bytes
    };

    //enum for packet type header
    enum PacketType : uint16_t {
        GENERIC = 0,
        MPC_UPDATE = 1,
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
    template<typename T>
    bool send(uint16_t type, const T& data);    
    
    //recieves either a fixed size binary struct or an ASCII line  over serial and returns either a packet struct or nulpptr if full packet isn't recieved 
    template <typename T>
    std::optional<T> receive();
    template<typename T>
    std::optional<T> receive(uint16_t expected_type);

    //send raw wakeup string to serial
    bool sendWakeup(const std::string& wakeup);

    // Wrapper for debug output that uses same mutex
    static void debugPrint(const char* msg);

private:
    int buffer_size;
    Mode mode;
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
    bool skipBytes(size_t num_bytes) {
        //buffer to hold discarded data 
        char discard[256];
        //continue until all requested bytes have been discarded
        while (num_bytes > 0) {
            //calc num bytes to read but not more than buffer size
            size_t to_read = std::min(num_bytes, sizeof(discard));
            //put bytes into discard buffer and are ignored
            size_t read = std::fread(discard, 1, to_read, stdin);
            //hit EOF or an error
            if (read == 0) {
                return false;
            }
            num_bytes -= read;
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
template<typename T>
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
template<typename T>
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
//Packet format: [BinaryHeader][payload bytes][CRC-8]
template <typename T>
bool SerialProtocol::sendBinary(uint16_t type, const T& packet) {
    //ensure type can be safley uses as raw bytes
    static_assert(std::is_trivially_copyable_v<T>, "Binary packet must be trivially copyable");
    //makes a header and sets type and size
    BinaryHeader header;
    header.type = type;
    header.size = sizeof(T);
    //Interpret struct as a raw byte array
    const uint8_t* raw = reinterpret_cast<const uint8_t*>(&packet);
    //compute CRC over raw binary
    uint8_t crc = crc8(raw, sizeof(T));
    //writes header to USB serial
    std::fwrite(&header, 1, sizeof(header), stdout);
    //write raw bytes to USB serial
    std::fwrite(raw, 1, sizeof(T), stdout);
    //send CRC byte after payload so reciever can validate
    std::fwrite(&crc, 1, 1, stdout);
    //force immediate writing
    std::fflush(stdout);
    return true;
}

// Receives a fixed-size binary struct from serial and returns nullptr if full packet isn't recieved
template <typename T>
std::optional<T> SerialProtocol::receiveBinary(uint16_t expectedType) {
    //ensure type can be safley uses as raw bytes
    static_assert(std::is_trivially_copyable_v<T>, "Binary packet must be trivially copyable");
    
    //declares header obj
    BinaryHeader header;
    uint8_t byte;

    //scan one byte at a time for sync bytes 0xAA 0x55
    while (true) {
        //read next byte with 1s timeout
        if (!readBytes(&byte, 1, 1000)) {
            return std::nullopt;
        }
        //look for first sync byte
        if (byte == 0xAA) {
            //read potential second sync bye
            uint8_t next_byte;
            if (!readBytes(&next_byte, 1, 100)) {
                return std::nullopt;
            }
            //check for second synce byte
            if (next_byte == 0x55) {
                //skip the 2 sync bytes and read the rest
                if (!readBytes(((uint8_t*)&header) + 2, sizeof(BinaryHeader) - 2, 100)) {
                    return std::nullopt;
                }
                break;
            } 
            else {
                continue;
            }
        }
    }
    //filter by type
    if (header.type != expectedType) {
        // skip payload + crc
        skipBytes(header.size + 1);
        return std::nullopt;
    }
    //validate size
    if (header.size != sizeof(T)) {
        skipBytes(header.size + 1);
        return std::nullopt;
    }

    //oversize protection
    if (header.size > buffer_size || header.size == 0) {
        skipBytes(header.size + 1);
        return std::nullopt;
    }

    //declares packet obj
    T packet;
    //Interpret struct memory as a writable byte buffer 
    uint8_t* raw = reinterpret_cast<uint8_t*>(&packet);
    // read exactly sizeof(T) bytes into the struct
    if (!readBytes(&packet, sizeof(T), 1000)) {
        return std::nullopt;
    }
    //read CRC byte
    uint8_t recv_crc;
    if (!readBytes(&recv_crc, 1, 100)) {
        return std::nullopt;
    }
    //compute crc and compare
    if (crc8(raw, sizeof(T)) != recv_crc) {
        return std::nullopt;
    }
    //return reconstructed struct
    return packet;
}   

//compute CRC-8 (0x07) over a byte buffer
//used to detect corruption over USB serial
inline uint8_t SerialProtocol::crc8(const uint8_t* data, size_t len) {
    //initialize CRC accumulator
    uint8_t crc = 0;
    //process each byte in buffer
    for (size_t i = 0; i < len; i++) {
        //xor current byte into CRC register
        crc ^= data[i];
        //process all the current byte
        for (int b = 0; b < 8; b++) {
            //if the most sig bit is set, shift left and apply polynomial
            if (crc & 0x80) {
                crc = (crc << 1) ^ 0x07;
            }
            //else shift left
            else {
                crc <<= 1;
            }
        }
    }
    return crc;
}
#endif