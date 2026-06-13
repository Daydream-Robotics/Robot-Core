#include "serialProtocol.hpp"

#include <cstdio>
#include <cstring>
#include <sstream>
#include <chrono>
#include <thread>

//constructor (separators are now fixed)
SerialProtocol::SerialProtocol(int bufferSize, Mode mode)
    : buffer_size(bufferSize),
      mode(mode),
      rx_buffer(8192),
      rx_buffer_pos(0) {}

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

    //keep reading lines until we find a valid packet (skip debug messages)
     while (std::fgets(buffer, sizeof(buffer), stdin)) {
        std::string line(buffer);
        auto packet = deserializePacket(line);
        
        if (packet.has_value()) {
            //only return if it's a valid packet type (not debug output)
            if (isValidPacketType(packet->type)) {
                return packet;
            }
        }
    }
    return std::nullopt;
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

//reads n bytes from serial with timeout support
bool SerialProtocol::readBytes(void* dest, size_t n, int timeout_ms) {
    uint8_t* out = reinterpret_cast<uint8_t*>(dest);
    std::size_t remaining = n;

    auto start_time = std::chrono::steady_clock::now();
    while (remaining>0) {
        //use any buffered data before reading from serial
        if (rx_buffer_pos > 0) {
            std::size_t to_copy = std::min(remaining, rx_buffer_pos);
            memcpy(out, rx_buffer.data(), to_copy);
            if (rx_buffer_pos > to_copy) {
                //shifts remaining buffer data to the front
                memmove(rx_buffer.data(), rx_buffer.data() + to_copy, rx_buffer_pos - to_copy);
            }
            rx_buffer_pos -= to_copy;
            out+= to_copy;
            remaining -= to_copy;
            continue;
        }
        //check timeout before reading more data
        if (timeout_ms > 0) {
            auto now = std::chrono::steady_clock::now();
            if (std::chrono::duration_cast<std::chrono::milliseconds>(now - start_time).count() > timeout_ms) {
                return false;
            }
        }

        //read available data from serial into buffer
        int byte = std::getc(stdin);
        if (byte == EOF) {
            std::this_thread::sleep_for(std::chrono::milliseconds(5));
            continue;
        }
        
        rx_buffer[0] = static_cast<uint8_t>(byte);
        rx_buffer_pos = 1;
    }

    return true;
}



//send wakeup string
bool SerialProtocol::sendWakeup(const std::string& wakeup) {
    //write wakeup string to serial
    std::fwrite(wakeup.c_str(), 1, wakeup.size(), stdout);
    //force immediate output
    std::fflush(stdout);
    return true;
}

bool SerialProtocol::isValidPacketType(const std::string& type) {
    if (type.empty()) return true;
    for (char c : type) {
        if (!std::isalnum(static_cast<unsigned char>(c)) && c != '_') {
            return false;  
        }
    }
    return true;
}