#include "serial_protocol.hpp"
#include <iostream>
#include <cstring>
#include <cassert>
#include <cmath>

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

struct Capture {
    char* buf;
    long len;
    FILE* f;
    FILE* old;
    
    Capture() {
        buf = new char[8192];
        len = 0;
        f = tmpfile();
        old = stdout;
        stdout = f;
    }
    
    void stop() {
        fflush(stdout);
        len = ftell(f);
        rewind(f);
        fread(buf, 1, len, f);
        stdout = old;
        fclose(f);
    }
    
    ~Capture() {
        delete[] buf;
    }
};

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

// ===================== NANO TESTS =====================

// --- ASCII Mode: Send ---

bool test_nano_ascii_send_status() {
    TEST("Nano ASCII send status packet");
    Capture cap;
    SerialProtocol serial(256);
    SerialProtocol::Packet pkt;
    pkt.type = "STATUS";
    pkt.message = {{"mpc", "running"}, {"iter", "42"}};
    bool ok = serial.send(pkt);
    ASSERT(ok, "send failed");
    cap.stop();
    std::string out(cap.buf, cap.len);
    ASSERT(out == "STATUS,mpc,running|iter,42\n", "wrong output: " + out);
    PASS();
    return true;
}

bool test_nano_ascii_send_single_field() {
    TEST("Nano ASCII send single field");
    Capture cap;
    SerialProtocol serial(256);
    SerialProtocol::Packet pkt;
    pkt.type = "PONG";
    pkt.message = {{"alive"}};
    bool ok = serial.send(pkt);
    ASSERT(ok, "send failed");
    cap.stop();
    std::string out(cap.buf, cap.len);
    ASSERT(out == "PONG,alive\n", "wrong: " + out);
    PASS();
    return true;
}

// --- ASCII Mode: Receive ---

bool test_nano_ascii_recv_odom() {
    TEST("Nano ASCII receive odometry");
    StdinFeed feed("ODOM,x,10.5|y,5.2|theta,1.57\n");
    SerialProtocol serial(256);
    auto pkt = serial.receive<SerialProtocol::Packet>();
    ASSERT(pkt.has_value(), "no packet");
    ASSERT(pkt->type == "ODOM", "wrong type");
    ASSERT(pkt->message.size() == 3, "wrong msg count");
    ASSERT(pkt->message[0][1] == "10.5", "wrong x");
    ASSERT(pkt->message[1][1] == "5.2", "wrong y");
    ASSERT(pkt->message[2][1] == "1.57", "wrong theta");
    PASS();
    return true;
}

bool test_nano_ascii_recv_empty() {
    TEST("Nano ASCII receive empty line");
    StdinFeed feed("\n");
    SerialProtocol serial(256);
    auto pkt = serial.receive<SerialProtocol::Packet>();
    ASSERT(pkt.has_value(), "should get packet (empty type)");
    ASSERT(pkt->type.empty(), "type should be empty");
    PASS();
    return true;
}

// --- Binary Mode: Receive MPC Update ---

bool test_nano_binary_recv_mpc_update() {
    TEST("Nano binary receive MPCUpdatePacket");

    MPCUpdatePacket update;
    update.pose_x = 24.0f;
    update.pose_y = 36.0f;
    update.pose_theta = 0.785f;
    update.omega_L = 200.0f;
    update.omega_R = 190.0f;
    update.V_battery = 11.8f;
    update.I_total = 3.0f;
    for (int i = 0; i < 120; i++) update.z_desired[i] = static_cast<float>(i * 2);

    SerialProtocol::BinaryHeader hdr;
    hdr.type = 1;  // MPC_UPDATE
    hdr.size = sizeof(MPCUpdatePacket);
    const uint8_t* raw = reinterpret_cast<const uint8_t*>(&update);
    uint8_t crc = 0;
    for (size_t i = 0; i < sizeof(update); i++) {
        crc ^= raw[i];
        for (int b = 0; b < 8; b++)
            crc = (crc & 0x80) ? (crc << 1) ^ 0x07 : (crc << 1);
    }

    std::string stream;
    stream.append(reinterpret_cast<const char*>(&hdr), sizeof(hdr));
    stream.append(reinterpret_cast<const char*>(&update), sizeof(update));
    stream.push_back(static_cast<char>(crc));

    StdinFeed feed(stream);
    SerialProtocol serial(256, ',', '|', '\n', SerialProtocol::Mode::BINARY);
    auto result = serial.receive<MPCUpdatePacket>(1);
    ASSERT(result.has_value(), "no packet");
    ASSERT(std::abs(result->pose_x - 24.0f) < 0.01f, "pose_x wrong");
    ASSERT(std::abs(result->pose_y - 36.0f) < 0.01f, "pose_y wrong");
    ASSERT(std::abs(result->V_battery - 11.8f) < 0.01f, "battery wrong");
    PASS();
    return true;
}

// --- Binary Mode: Send Control ---

bool test_nano_binary_send_control() {
    TEST("Nano binary send MPCControlPacket");
    Capture cap;
    SerialProtocol serial(256, ',', '|', '\n', SerialProtocol::Mode::BINARY);
    MPCControlPacket ctrl;
    ctrl.V_left = 7.5f;
    ctrl.V_right = 6.3f;
    bool ok = serial.send(static_cast<uint16_t>(2), ctrl);
    ASSERT(ok, "send failed");
    cap.stop();
    ASSERT(cap.len == 6 + sizeof(MPCControlPacket) + 1, "wrong size: " + std::to_string(cap.len));
    PASS();
    return true;
}

// --- Binary roundtrip ---

bool test_nano_binary_roundtrip() {
    TEST("Nano binary send-receive roundtrip");

    TestBinarySmall send_data{12345, -67890};

    Capture cap;
    SerialProtocol serial_send(256, ',', '|', '\n', SerialProtocol::Mode::BINARY);
    serial_send.send(static_cast<uint16_t>(3), send_data);
    cap.stop();

    StdinFeed feed(std::string(cap.buf, cap.len));
    SerialProtocol serial_recv(256, ',', '|', '\n', SerialProtocol::Mode::BINARY);
    auto result = serial_recv.receive<TestBinarySmall>(3);

    ASSERT(result.has_value(), "no packet");
    ASSERT(result->a == 12345, "a wrong: " + std::to_string(result->a));
    ASSERT(result->b == -67890, "b wrong: " + std::to_string(result->b));
    PASS();
    return true;
}

// --- Wakeup receive ---

bool test_nano_recv_wakeup() {
    TEST("Nano receive wakeup string");
    StdinFeed feed("VEX_READY\n");
    SerialProtocol serial(256);
    char buf[64];
    ASSERT(std::fgets(buf, sizeof(buf), stdin) != nullptr, "fgets failed");
    std::string s(buf);
    ASSERT(s == "VEX_READY\n", "wrong: " + s);
    PASS();
    return true;
}

// --- Wakeup send ---

bool test_nano_send_wakeup_response() {
    TEST("Nano send wakeup response");
    Capture cap;
    SerialProtocol serial(256);
    bool ok = serial.sendWakeup("NANO_READY\n");
    ASSERT(ok, "wakeup failed");
    cap.stop();
    std::string out(cap.buf, cap.len);
    ASSERT(out == "NANO_READY\n", "wrong: " + out);
    PASS();
    return true;
}

// --- Full MPC loop (Nano perspective) ---

bool test_nano_mpc_loop() {
    TEST("Nano full MPC loop (recv update, send control)");

    // Step 1: Nano receives update from "VEX"
    MPCUpdatePacket update;
    update.pose_x = 12.0f;
    update.pose_y = 24.0f;
    update.pose_theta = 0.5f;
    update.omega_L = 50.0f;
    update.omega_R = 55.0f;
    update.V_battery = 12.6f;
    update.I_total = 1.0f;
    for (int i = 0; i < 120; i++) update.z_desired[i] = static_cast<float>(i % 3);

    SerialProtocol::BinaryHeader hdr;
    hdr.type = 1;
    hdr.size = sizeof(MPCUpdatePacket);
    const uint8_t* raw = reinterpret_cast<const uint8_t*>(&update);
    uint8_t crc = 0;
    for (size_t i = 0; i < sizeof(update); i++) {
        crc ^= raw[i];
        for (int b = 0; b < 8; b++)
            crc = (crc & 0x80) ? (crc << 1) ^ 0x07 : (crc << 1);
    }

    std::string stream;
    stream.append(reinterpret_cast<const char*>(&hdr), sizeof(hdr));
    stream.append(reinterpret_cast<const char*>(&update), sizeof(update));
    stream.push_back(static_cast<char>(crc));

    StdinFeed feed(stream);

    Capture cap_ctrl;
    {
        SerialProtocol recv(256, ',', '|', '\n', SerialProtocol::Mode::BINARY);
        auto pkt = recv.receive<MPCUpdatePacket>(1);
        ASSERT(pkt.has_value(), "no update received");

        // "Compute" control
        MPCControlPacket ctrl;
        ctrl.V_left = 6.0f + pkt->pose_x * 0.1f;
        ctrl.V_right = 6.0f - pkt->pose_x * 0.1f;

        SerialProtocol send(256, ',', '|', '\n', SerialProtocol::Mode::BINARY);
        bool ok = send.send(static_cast<uint16_t>(2), ctrl);
        ASSERT(ok, "send control failed");
    }
    cap_ctrl.stop();
    ASSERT(cap_ctrl.len == 6 + sizeof(MPCControlPacket) + 1, "wrong control size");
    PASS();
    return true;
}

// --- Mode switching ---

bool test_nano_ascii_to_binary() {
    TEST("Nano ASCII then Binary mode");

    // ASCII receive
    {
        StdinFeed feed("PING\n");
        SerialProtocol serial(256);
        auto pkt = serial.receive<SerialProtocol::Packet>();
        ASSERT(pkt.has_value(), "ascii recv failed");
        ASSERT(pkt->type == "PING", "wrong type");
    }

    // Binary send
    {
        Capture cap;
        SerialProtocol serial(256, ',', '|', '\n', SerialProtocol::Mode::BINARY);
        MPCControlPacket ctrl{1.0f, 2.0f};
        bool ok = serial.send(static_cast<uint16_t>(2), ctrl);
        ASSERT(ok, "binary send failed");
        cap.stop();
        ASSERT(cap.len == 6 + sizeof(MPCControlPacket) + 1, "wrong size");
    }

    PASS();
    return true;
}

// ===================== MAIN =====================

int main() {
    std::cout << "============================================" << std::endl;
    std::cout << "  Jetson Nano Side - Serial Protocol Tests" << std::endl;
    std::cout << "============================================" << std::endl;

    std::cout << std::endl << "--- ASCII Send ---" << std::endl;
    test_nano_ascii_send_status();
    test_nano_ascii_send_single_field();

    std::cout << std::endl << "--- ASCII Receive ---" << std::endl;
    test_nano_ascii_recv_odom();
    test_nano_ascii_recv_empty();

    std::cout << std::endl << "--- Binary Receive ---" << std::endl;
    test_nano_binary_recv_mpc_update();

    std::cout << std::endl << "--- Binary Send ---" << std::endl;
    test_nano_binary_send_control();

    std::cout << std::endl << "--- Binary Roundtrip ---" << std::endl;
    test_nano_binary_roundtrip();

    std::cout << std::endl << "--- Wakeup ---" << std::endl;
    test_nano_recv_wakeup();
    test_nano_send_wakeup_response();

    std::cout << std::endl << "--- Full MPC Loop ---" << std::endl;
    test_nano_mpc_loop();

    std::cout << std::endl << "--- Mode Switching ---" << std::endl;
    test_nano_ascii_to_binary();

    std::cout << std::endl << "============================================" << std::endl;
    std::cout << "Nano Tests: " << tests_passed << "/" << tests_run << " passed" << std::endl;
    return (tests_passed == tests_run) ? 0 : 1;
}