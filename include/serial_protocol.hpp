#ifndef SERIAL_PROTOCOL_HPP
#define SERIAL_PROTOCOL_HPP

#include <string>
#include <vector>
#include <optional>
#include <cstdint>


class SerialProtocol {
public:
    //mode for sending data
    enum class Mode {
        ASCII,
        BINARY
    };
    /*
        Packet format:
        TYPE,field1,field2|field1,field2|field1,field2\n
        Example:
        ODOM,x,10.5|y,5.2|theta,1.57\n
        ------------------------------------------------
        TYPE            -> packet type / command name
        ,               -> field separator
        field1,field2   -> one message block
        |               -> separator between message blocks
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
            Represents: x,10.5|y,5.2|theta,1.57
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

private:
    int buffer_size;
    Mode mode;

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

    //scan one byte at a time for sync byte 0xAA 0x55
    while (true) {
        if (byte == 0xAA) {
            //maybe start look at next byte
            int next = std::getc(stdin);
            if (next == EOF) {
                return std::nullopt;
            }
            if (next == 0x55) {
                //found sync so read rest of header
                if (std::fread(((uint8_t*)&header) + 2, 1, sizeof(BinaryHeader) - 2, stdin) != sizeof(BinaryHeader) - 2) {
                    return std::nullopt;
                }
                break;
            } else {
                //false start
                std::ungetc(next, stdin);
            }
        }
    }
    //read rest of header
    if (std::fread(((uint8_t*)&header) + sizeof(uint16_t), 1, sizeof(BinaryHeader) - sizeof(uint16_t),stdin) != sizeof(BinaryHeader) - sizeof(uint16_t)) {
        return std::nullopt;
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
    //tries to read sizeof(T) bytes from serial
    size_t read = std::fread(raw, 1, sizeof(T), stdin);
    // if we didn't get a full packet return nullptr
    if (read != sizeof(T)) {
        return std::nullopt;
    }
    //read CRC byte
    uint8_t recv_crc;
    std::fread(&recv_crc, 1, 1, stdin);
    //verify message integrity using CRC comparison
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