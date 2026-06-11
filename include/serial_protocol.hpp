#ifndef SERIAL_PROTOCOL_HPP
#define SERIAL_PROTOCOL_HPP

#include <string>
#include <vector>
#include <optional>

class SerialProtocol {
public:
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


    //constructor with defaults
    SerialProtocol(char field_seperator = ',', char message_seperator = '|', char end_char = '\n');
    
    
    //serializes packet into string then sends it over USB serial
    bool sendPacket(const Packet& packet);
    //reads one line from USB serial then parses into a packet object, returns std::nullopt if no packet is recieved
    std::optional<Packet> receivePacket();
    //send raw waakeup string to serial
    bool sendWakeup(const std::string& wakeup);

private:
    //holds special chars
    char field_seperator;
    char message_seperator;
    char end_char;
    //converts packet struct to raw string packet to send
    static std::string serializePacket(const Packet& packet, char field_seperator, char message_seperator, char end_char);
    //converts raw packet string (from serial) to a packet struct
    static std::optional<Packet> deserializePacket(const std::string& line, char field_seperator, char message_seperator);
};

#endif