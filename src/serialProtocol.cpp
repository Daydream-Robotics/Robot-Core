#include "serialProtocol.hpp"

#include <cstdio>
#include <cstring>
#include <chrono>
#include <thread>

//constructor
SerialProtocol::SerialProtocol(int bufferSize, Mode mode)
    : buffer_size(bufferSize),
      mode(mode),
      rx_buffer(8192),
      rx_buffer_pos(0) {}

//function to serialize packet into string then write to USB serial
bool SerialProtocol::sendASCII(const Packet& packet) {
    //converts packet struct into raw string packet
    std::string data = serializePacket(packet);
    //write to serial
    std::fwrite(data.c_str(), 1, data.size(), stdout);
    //force serial output immediately
    std::fflush(stdout);

    return true;
}

std::optional<SerialProtocol::Packet> SerialProtocol::receiveASCII() {
    //temporary recieve buffer
    char buffer[1024];
    //keep reading lines until we find a valid packet (skip debug messages)
     while (std::fgets(buffer, sizeof(buffer), stdin)) {
        std::string line(buffer);
        auto packet = deserializePacket(line);
        //only return if it's a valid packet type (not debug output)
        if (packet.has_value() && isValidPacketType(packet->type)) {
            return packet;
        }
    }
    return std::nullopt;
}

//converts packet struct into packet string
std::string SerialProtocol::serializePacket(const Packet& packet) {
    const char field_sep   = ',';
    const char message_sep = ';';
    const char end_char    = '\n';

    //declares final output string
    std::string out = packet.type;

    //if no packet message just return packet type and newline
    if (packet.message.empty()) {
        out += end_char;
        return out;
    }

    //add seperator between type and body
    out += field_sep;

    //loop through each message block
    for (size_t i = 0; i < packet.message.size(); ++i) {
        //stores current message block
        const auto& block = packet.message[i];
        //loop through fields in message
        for (size_t j = 0; j < block.size(); ++j) {
            //add field contents
            out += block[j];
            //add field seperator between fields (except for final field)
            if (j + 1 < block.size()) {
                out += field_sep;
            }
        }
        //add message seperator between messages (except for final block)
        if (i + 1 < packet.message.size()) {
            out += message_sep;
        }
    }

    //add final packet terminator
    out += end_char;

    return out;
}

//convert packet string into packet struct
std::optional<SerialProtocol::Packet> SerialProtocol::deserializePacket(
    const std::string& line) {
    const char field_sep   = ',';
    const char message_sep = ';';

    //if the packet is empty return nullptr
    if (line.empty()) {
        return std::nullopt;
    }

    //makes a copy of line to safely modify
    std::string trimmed = line;

    //removes trailing newline
    if(!trimmed.empty() && trimmed.back() == '\n') {
        trimmed.pop_back();
    }
    //removes carriage return (artifact)
    if(!trimmed.empty() && trimmed.back() == '\r') {
        trimmed.pop_back();
    }


    Packet packet;
    //find first field seperator
    size_t sep_pos = trimmed.find(field_sep);
    //if there isn't a seperator, that means packet only contains type (and return)
    if (sep_pos == std::string::npos) {
        packet.type = trimmed;
        return packet;
    }

    //extracts packet type
    packet.type = trimmed.substr(0, sep_pos);
    //extracts packet body
    std::string body = trimmed.substr(sep_pos + 1);

    //store current parsing position
    size_t start = 0;
    //parse each message block
    while (start < body.size()) {
        //find next message seperator
        size_t msg_end = body.find(message_sep, start);
        //extract one message block
        std::string block_str = body.substr(start, msg_end - start);

        //store fields for current message block
        std::vector<std::string> fields;
        //store current field parsing position
        size_t field_start = 0;
        //parse fields inside message block
        while (field_start < block_str.size()) {
            //find next field seperator
            size_t field_end = block_str.find(field_sep, field_start);
            //extract field text
            fields.push_back(block_str.substr(field_start, field_end - field_start));
            
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

        //read 1 byte so fread never blocks waiting to fill a large buffer
        int byte = std::fread(rx_buffer.data(), 1, 1, stdin);
        if (byte <= 0) {
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
            continue;
        }
        rx_buffer_pos = byte;
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

//returns true if the packet type is valid.
//its valid type if its either empty or is only alphanumeric chars and underscores.
bool SerialProtocol::isValidPacketType(const std::string& type) {
    if (type.empty()) {
        return true;
    }
    for (char c : type) {
        if (!std::isalnum(static_cast<unsigned char>(c)) && c != '_') {
            return false;
        }
    }
    return true;
}

