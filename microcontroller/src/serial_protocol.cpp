#include "serial_protocol.hpp"

#include <cstdio>
#include <cstring>
#include <sstream>

//constructor
SerialProtocol::SerialProtocol(int bufferSize, char fieldSep, char messageSep, char term, Mode mode)
    : buffer_size(bufferSize),
      field_seperator(fieldSep),
      message_seperator(messageSep),
      end_char(term),
      mode(mode) {}

//unified send
template <typename T>
bool SerialProtocol::send(const T& data) {
    if (mode == Mode::ASCII) {
        return sendASCII(data);
    }
    if (mode == Mode::BINARY) {
        return sendBinary(0, data);
    }
    return false;
}

template<typename T>
bool SerialProtocol::send(uint16_t type, const T& data) {

    if (mode == Mode::ASCII) {

        if constexpr (std::is_same_v<T, Packet>) {
            return sendASCII(data);
        }

        return false;
    }

    if (mode == Mode::BINARY) {
        return sendBinary(type, data);
    }

    return false;
}

//unified recieve
template <typename T>
std::optional<T> SerialProtocol::receive() {
    if (mode == Mode::ASCII) {
        return receiveASCII();
    }
    if (mode == Mode::BINARY) {
        return receiveBinary<T>(0);
    }
    return std::nullopt;
}

template<typename T>
std::optional<T> SerialProtocol::receive(uint16_t expected_type) {

    if (mode == Mode::ASCII) {

        if constexpr (std::is_same_v<T, Packet>) {
            return receiveASCII();
        }

        return std::nullopt;
    }

    if (mode == Mode::BINARY) {
        return receiveBinary<T>(expected_type);
    }

    return std::nullopt;
}


//function to serialize packet into string then write to USB serial
bool SerialProtocol::sendASCII(const Packet& packet){
    //converts packet struct into raw string packet
    std::string data = serializePacket(packet, field_seperator, message_seperator, end_char);
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
    return deserializePacket(line, field_seperator, message_seperator);
}

//sends struct as raw binary over serial
template <typename T>
bool SerialProtocol::sendBinary(uint16_t type, const T& packet) {
    //ensure type can be safley uses as raw bytes
    static_assert(std::is_trivially_copyable_v<T>, "Binary packet must be trivially copyable");
    //makes a header an sets type and size
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

    // find sync byte
    while (true) {
        if (std::fread(&header.sync, 1, sizeof(uint16_t), stdin) != sizeof(uint16_t)) {
            return std::nullopt;
        }
        if (header.sync == 0xAA55) {
            break;
        }
    }
    //read rest of header
    if (std::fread(((uint8_t*)&header) + sizeof(uint16_t), 1, sizeof(BinaryHeader) - sizeof(uint16_t),stdin) != sizeof(BinaryHeader) - sizeof(uint16_t)) {
        return std::nullopt;
    }
    //filter by type
    if (header.type != expectedType) {
        // skip payload + crc
        std::fseek(stdin, header.size + 1, SEEK_CUR);
        return std::nullopt;
    }
    //validate size
    if (header.size != sizeof(T)) {
        std::fseek(stdin, header.size + 1, SEEK_CUR);
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

//Compute CRC-8 (polynomial 0x07) over a byte buffer
//Used to detect corruption over USB serial
inline uint8_t SerialProtocol::crc8(const uint8_t* data, size_t len) {
    uint8_t crc = 0;

    for (size_t i = 0; i < len; i++) {
        crc ^= data[i];

        for (int b = 0; b < 8; b++) {
            if (crc & 0x80)
                crc = (crc << 1) ^ 0x07;
            else
                crc <<= 1;
        }
    }

    return crc;
}

//converts packet struct into packet string
std::string SerialProtocol::serializePacket(const Packet& packet, char field_seperator, char message_seperator, char end_char) {
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
std::optional<SerialProtocol::Packet> SerialProtocol::deserializePacket(const std::string& line, char field_separator, char message_separator) {
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
