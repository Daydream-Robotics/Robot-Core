#include "serial_protocol.hpp"
#include <iostream>
#include <cstring>
#include <cassert>
#include <cmath>
#include <thread>
#include <chrono>

#pragma pack(push, 1)
struct MPCUpdatePacket {
    float pose_x;
    float pose_y;
    float pose_theta;
    float omega_L;
    float omega_R;
    float V_battery;
    float I_total;
    float z_desired[120];
};
struct MPCControlPacket {
    float V_left;
    float V_right;
};
struct TestBinarySmall {
    int32_t a;
    int32_t b;
};
#pragma pack(pop)

int tests_run = 0;
int tests_passed = 0;

#define TEST(name) do { tests_run++; std::cout << "  TEST: " << name << " ... "; } while(0)
#define PASS() do { tests_passed++; std::cout << "PASSED" << std::endl; } while(0)
#define FAIL(msg) do { std::cout << "FAILED: " << msg << std::endl; } while(0)
#define ASSERT(cond, msg) do { if (!(cond)) { FAIL(msg); return false; } } while(0)

// Capture stdout to buffer, then restore
struct Capture {
    char buf[4096];
    long len;
    FILE* f;
    FILE* old;
    Capture() {
        f = tmpfile();
        old = stdout;
        stdout = f;
    }
    ~Capture() {
        fflush(stdout);
        len = ftell(f);
        rewind(f);
        fread(buf, 1, len, f);
        stdout = old;
        fclose(f);
    }
};

// Feed a buffer to stdin
struct StdinFeed {
    FILE* f;
    FILE* old;
    StdinFeed(const char* data, size_t len) {
        f = tmpfile();
        fwrite(data, 1, len, f);
        rewind(f);
        old = stdin;
        stdin = f;
    }
    StdinFeed(const std::string& s) : StdinFeed(s.c_str(), s.size()) {}
    ~StdinFeed() {
        stdin = old;
        fclose(f);
    }
};

// ===================== VEX TESTS =====================

// --- ASCII Mode: Send ---

bool test_vex_ascii_send_simple() {
    TEST("VEX ASCII send simple packet");
    Capture cap;
    SerialProtocol serial(256);
    SerialProtocol::Packet pkt;
    pkt.type = "ODOM";
    pkt.message = {{"x", "10.5"}, {"y", "5.2"}, {"theta", "1.57"}};
    bool ok = serial.send(pkt);
    ASSERT(ok, "send failed");
    std::string out(cap.buf, cap.len);
    ASSERT(out == "ODOM,x,10.5|y,5.2|theta,1.57\n", "wrong output: " + out);
    PASS();
    return true;
}

bool test_vex_ascii_send_type_only() {
    TEST("VEX ASCII send type-only (HEARTBEAT)");
    Capture cap;
    SerialProtocol serial(256);
    SerialProtocol::Packet pkt;
    pkt.type = "HEARTBEAT";
    bool ok = serial.send(pkt);
    ASSERT(ok, "send failed");
    std::string out(cap.buf, cap.len);
    ASSERT(out == "HEARTBEAT\n", "wrong output: " + out);
    PASS();
    return true;
}

bool test_vex_ascii_send_custom_sep() {
    TEST("VEX ASCII send with custom separators");
    Capture cap;
    SerialProtocol serial(256, ';', '#', '\r');
    SerialProtocol::Packet pkt;
    pkt.type = "DATA";
    pkt.message = {{"a", "1"}, {"b", "2"}};
    bool ok = serial.send(pkt);
    ASSERT(ok, "send failed");
    std::string out(cap.buf, cap.len);
    ASSERT(out == "DATA;a,1#b,2\r", "wrong output: " + out);
    PASS();
    return true;
}

// --- ASCII Mode: Receive ---

bool test_vex_ascii_recv_control() {
    TEST("VEX ASCII receive control-like packet");
    StdinFeed feed("CTRL,left,6.0|right,5.5\n");
    SerialProtocol serial(256);
    auto pkt = serial.receive<SerialProtocol::Packet>();
    ASSERT(pkt.has_value(), "no packet");
    ASSERT(pkt->type == "CTRL", "wrong type: " + pkt->type);
    ASSERT(pkt->message.size() == 2, "wrong msg count");
    ASSERT(pkt->message[0][0] == "left" && pkt->message[0][1] == "6.0", "wrong left");
    ASSERT(pkt->message[1][0] == "right" && pkt->message[1][1] == "5.5", "wrong right");
    PASS();
    return true;
}

bool test_vex_ascii_recv_crlf() {
    TEST("VEX ASCII receive with CRLF");
    StdinFeed feed("STATUS,ok\r\n");
    SerialProtocol serial(256);
    auto pkt = serial.receive<SerialProtocol::Packet>();
    ASSERT(pkt.has_value(), "no packet");
    ASSERT(pkt->type == "STATUS", "wrong type");
    ASSERT(pkt->message[0][0] == "ok", "wrong field");
    PASS();
    return true;
}

// --- Binary Mode: Send MPC Update ---

bool test_vex_binary_send_mpc_update() {
    TEST("VEX binary send MPCUpdatePacket");
    Capture cap;
    SerialProtocol serial(256, ',', '|', '\n', SerialProtocol::Mode::BINARY);
    MPCUpdatePacket pkt;
    pkt.pose_x = 12.0f;
    pkt.pose_y = 24.0f;
    pkt.pose_theta = 1.57f;
    pkt.omega_L = 100.0f;
    pkt.omega_R = 95.0f;
    pkt.V_battery = 12.6f;
    pkt.I_total = 2.5f;
    for (int i = 0; i < 120; i++) pkt.z_desired[i] = static_cast<float>(i);
    bool ok = serial.send(static_cast<uint16_t>(1), pkt);
    ASSERT(ok, "send failed");
    // Should be: 6 (header) + 512 (struct) + 1 (CRC) = 519 bytes
    ASSERT(cap.len == 6 + sizeof(MPCUpdatePacket) + 1, "wrong binary size: " + std::to_string(cap.len));
    PASS();
    return true;
}

// --- Binary Mode: Receive Control ---

bool test_vex_binary_recv_control() {
    TEST("VEX binary receive MPCControlPacket");
    // Build a valid binary packet for MPCControlPacket type=2
    MPCControlPacket ctrl;
    ctrl.V_left = 6.5f;
    ctrl.V_right = 5.5f;
    
    SerialProtocol::BinaryHeader hdr;
    hdr.type = 2;
    hdr.size = sizeof(MPCControlPacket);
    const uint8_t* raw = reinterpret_cast<const uint8_t*>(&ctrl);
    
    // Compute CRC manually
    uint8_t crc = 0;
    for (size_t i = 0; i < sizeof(ctrl); i++) {
        crc ^= raw[i];
        for (int b = 0; b < 8; b++)
            crc = (crc & 0x80) ? (crc << 1) ^ 0x07 : (crc << 1);
    }
    
    // Build the binary stream
    std::string stream;
    stream.append(reinterpret_cast<const char*>(&hdr), sizeof(hdr));
    stream.append(reinterpret_cast<const char*>(&ctrl), sizeof(ctrl));
    stream.push_back(static_cast<char>(crc));
    
    StdinFeed feed(stream);
    SerialProtocol serial(256, ',', '|', '\n', SerialProtocol::Mode::BINARY);
    auto result = serial.receive<MPCControlPacket>(2);
    ASSERT(result.has_value(), "no packet received");
    ASSERT(std::abs(result->V_left - 6.5f) < 0.01f, "V_left mismatch");
    ASSERT(std::abs(result->V_right - 5.5f) < 0.01f, "V_right mismatch");
    PASS();
    return true;
}

// --- Binary Mode: Type filtering ---

bool test_vex_binary_type_filter() {
    TEST("VEX binary ignores wrong type");
    MPCControlPacket ctrl;
    ctrl.V_left = 1.0f;
    ctrl.V_right = 2.0f;
    
    SerialProtocol::BinaryHeader hdr;
    hdr.type = 99;  // wrong type
    hdr.size = sizeof(MPCControlPacket);
    
    std::string stream;
    stream.append(reinterpret_cast<const char*>(&hdr), sizeof(hdr));
    stream.append(reinterpret_cast<const char*>(&ctrl), sizeof(ctrl));
    stream.push_back(0);  // dummy CRC
    
    StdinFeed feed(stream);
    SerialProtocol serial(256, ',', '|', '\n', SerialProtocol::Mode::BINARY);
    auto result = serial.receive<MPCControlPacket>(2);  // expects type 2
    ASSERT(!result.has_value(), "should reject wrong type");
    PASS();
    return true;
}

// --- Wakeup ---

bool test_vex_wakeup() {
    TEST("VEX send wakeup");
    Capture cap;
    SerialProtocol serial(256);
    bool ok = serial.sendWakeup("VEX_READY\n");
    ASSERT(ok, "wakeup failed");
    std::string out(cap.buf, cap.len);
    ASSERT(out == "VEX_READY\n", "wrong wakeup: " + out);
    PASS();
    return true;
}

// --- Mode: ASCII rejects non-Packet ---

bool test_vex_ascii_rejects_struct() {
    TEST("VEX ASCII mode rejects struct send");
    Capture cap;
    SerialProtocol serial(256);
    TestBinarySmall s{42, 99};
    bool ok = serial.send(s);
    ASSERT(!ok, "should reject non-Packet in ASCII");
    PASS();
    return true;
}

// --- Full MPC loop simulation ---

bool test_vex_mpc_loop() {
    TEST("VEX full MPC update-control loop");
    
    // Step 1: VEX sends MPC update
    Capture cap_send;
    {
        SerialProtocol serial(256, ',', '|', '\n', SerialProtocol::Mode::BINARY);
        MPCUpdatePacket update;
        update.pose_x = 0.0f;
        update.pose_y = 0.0f;
        update.pose_theta = 0.0f;
        update.omega_L = 0.0f;
        update.omega_R = 0.0f;
        update.V_battery = 12.6f;
        update.I_total = 0.5f;
        for (int i = 0; i < 120; i++) update.z_desired[i] = static_cast<float>(i % 3);
        bool ok = serial.send(static_cast<uint16_t>(1), update);
        ASSERT(ok, "send update failed");
    }
    
    // Step 2: VEX receives control from "Nano"
    MPCControlPacket ctrl;
    ctrl.V_left = 7.2f;
    ctrl.V_right = 6.8f;
    
    SerialProtocol::BinaryHeader hdr;
    hdr.type = 2;
    hdr.size = sizeof(MPCControlPacket);
    const uint8_t* raw = reinterpret_cast<const uint8_t*>(&ctrl);
    uint8_t crc = 0;
    for (size_t i = 0; i < sizeof(ctrl); i++) {
        crc ^= raw[i];
        for (int b = 0; b < 8; b++)
            crc = (crc & 0x80) ? (crc << 1) ^ 0x07 : (crc << 1);
    }
    
    std::string stream;
    stream.append(reinterpret_cast<const char*>(&hdr), sizeof(hdr));
    stream.append(reinterpret_cast<const char*>(&ctrl), sizeof(ctrl));
    stream.push_back(static_cast<char>(crc));
    
    StdinFeed feed(stream);
    {
        SerialProtocol serial(256, ',', '|', '\n', SerialProtocol::Mode::BINARY);
        auto result = serial.receive<MPCControlPacket>(2);
        ASSERT(result.has_value(), "no control received");
        ASSERT(std::abs(result->V_left - 7.2f) < 0.01f, "V_left bad");
        ASSERT(std::abs(result->V_right - 6.8f) < 0.01f, "V_right bad");
    }
    
    PASS();
    return true;
}

// ===================== MAIN =====================

int main() {
    std::cout << "============================================" << std::endl;
    std::cout << "  VEX Brain Side - Serial Protocol Tests" << std::endl;
    std::cout << "============================================" << std::endl;
    
    std::cout << std::endl << "--- ASCII Send ---" << std::endl;
    test_vex_ascii_send_simple();
    test_vex_ascii_send_type_only();
    test_vex_ascii_send_custom_sep();
    
    std::cout << std::endl << "--- ASCII Receive ---" << std::endl;
    test_vex_ascii_recv_control();
    test_vex_ascii_recv_crlf();
    
    std::cout << std::endl << "--- Binary Send ---" << std::endl;
    test_vex_binary_send_mpc_update();
    
    std::cout << std::endl << "--- Binary Receive ---" << std::endl;
    test_vex_binary_recv_control();
    test_vex_binary_type_filter();
    
    std::cout << std::endl << "--- Wakeup ---" << std::endl;
    test_vex_wakeup();
    
    std::cout << std::endl << "--- Mode Restrictions ---" << std::endl;
    test_vex_ascii_rejects_struct();
    
    std::cout << std::endl << "--- Full MPC Loop ---" << std::endl;
    test_vex_mpc_loop();
    
    std::cout << std::endl << "============================================" << std::endl;
    std::cout << "VEX Tests: " << tests_passed << "/" << tests_run << " passed" << std::endl;
    return (tests_passed == tests_run) ? 0 : 1;
}