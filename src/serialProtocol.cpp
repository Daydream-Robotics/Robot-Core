#include "serialProtocol.hpp"

#include <cstdio>
#include <cstring>
#include <sstream>

//constructor (separators are now fixed)
SerialProtocol::SerialProtocol(int bufferSize, Mode mode)
    : buffer_size(bufferSize),
      mode(mode) {}

//function to serialize packet into string then write to USB serial
bool SerialProtocol::sendASCII(const Packet& packet){
    //converts packet struct into raw string packet
    std::string data = serializePacket(packet);
    //write to serial
    std::fwrite(data.c_str(), 1, data.size(), stdout);
    //force serial output immediately
    std::fflush(stdout);

    return true;
}

//reads one line from USB serial and deserializes into a packet struct
std::optional<SerialProtocol::Packet> SerialProtocol::receiveASCII() {
    //temporary recieve buffer
    char buffer[buffer_size];

    /* reads line from USB serial until newline, buffer is full, or error
        returns nullptr on failure */
    if(!std::fgets(buffer, sizeof(buffer), stdin)) {
        return std::nullopt;
    }
    //converts C string into std::string
    std::string line(buffer);
    //parses a packet string into a packet struct and returns it
    return deserializePacket(line);
}

//converts packet struct into packet string
std::string SerialProtocol::serializePacket(const Packet& packet) {
    const char field_seperator = ',';
    const char message_seperator = ';';
    const char end_char = '\n';
    
    //declares final output string
    std::string out;

    //add packet type to begininning
    out += packet.type;

    //if no packet message just return packet type and newline
    if(packet.message.empty()) {
        out += end_char;
        return out;
    }

    //add seperator between type and body
    out += field_seperator;

    //loop through each message block
    for (std::size_t i = 0; i < packet.message.size(); ++i) {
        //stores current message block
        const std::vector<std::string>& message = packet.message[i];
        //loop through fields in message
        for (std::size_t j = 0; j < message.size(); ++j) {
            //add field contents
            out += message[j];
            //add field seperator between fields (except for final field)
            if (j+1 < message.size()) {
                out += field_seperator;
            }
        }
        //add message seperator between messages (except for final block)
        if (i+1 < packet.message.size()) {
            out += message_seperator;
        }
    }

    //add final packet terminator
    out += end_char;

    return out;
}

//convert packet string into packet struct
std::optional<SerialProtocol::Packet> SerialProtocol::deserializePacket(const std::string& line) {
    const char field_separator = ',';
    const char message_separator = ';';
    
    //if the packet is empty return nullptr
    if (line.empty()) {
        return std::nullopt;
    }

    //declares output packet object
    SerialProtocol::Packet packet;
    //makes a copy of line to safely modify
    std::string trimmed = line;

    //removes trailing newline
    if(!trimmed.empty() && trimmed.back() == '\n') {
        trimmed.pop_back();
    }
    //removes carriage return (Windows artifact)
    if(!trimmed.empty() && trimmed.back() == '\r') {
        trimmed.pop_back();
    }

    //find first field seperator
    std::size_t seperator_pos = trimmed.find(field_separator);
    //if there isn't a seperator, that means packet only contains type (and return)
    if (seperator_pos == std::string::npos) {
        packet.type = trimmed;
        return packet;
    }

    //extracts packet type
    packet.type = trimmed.substr(0, seperator_pos);
    //extracts packet body
    std::string body = trimmed.substr(seperator_pos+1);
    
    //store current parsing position
    std::size_t start = 0;
    //parse each message block
    while (start < body.size()) {
        //find next message seperator
        std::size_t msg_end = body.find(message_separator, start);
        //extract one message block
        std::string message = body.substr(start, msg_end - start);
        //store fields for current message block
        std::vector<std::string> fields;

        //store current field parsing position
        std::size_t field_start = 0;
        //parse fields inside message block
        while (field_start < message.size()) {
            //find next field seperator
            std::size_t field_end = message.find(field_separator, field_start);
            //extract field text
            fields.push_back(message.substr(field_start, field_end - field_start));
            
            //if no more field seperators it means that final field is reached and we break
            if (field_end == std::string::npos) {
                break;
            }

            //move parser after field seperator
            field_start = field_end + 1;
        }

        //store parsed fields into packet
        if (!fields.empty()) {
            packet.message.push_back(std::move(fields));
        }

        //if no more message seperators it means that final message block is reached and we break
        if (msg_end == std::string::npos) {
            break;
        }

        //mover parser after message separator
        start = msg_end + 1;
    }

    return packet;
}

//send wakeup string
bool SerialProtocol::sendWakeup(const std::string& wakeup) {
    //write wakeup string to serial
    std::fwrite(wakeup.c_str(), 1, wakeup.size(), stdout);
    //force immediate output
    std::fflush(stdout);
    return true;
}