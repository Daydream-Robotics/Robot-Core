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

    //constructor with defaults
    SerialProtocol(int buffer_size, char field_seperator = ',', char message_seperator = '|', char end_char = '\n', Mode mode = Mode::ASCII);
    
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
    char field_seperator;
    char message_seperator;
    char end_char;
    Mode mode;

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
    static std::string serializePacket(const Packet& packet, char field_seperator, char message_seperator, char end_char);
    //converts raw packet string (from serial) to a packet struct
    static std::optional<Packet> deserializePacket(const std::string& line, char field_seperator, char message_seperator);
};

#endif