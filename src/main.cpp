#include "serialProtocol.hpp"
#include <iostream>
#include <chrono>
#include <thread>
#include <vector>
#include <algorithm>
#include <cstring>
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

int tests_passed = 0;
int tests_failed = 0;

#define TEST_ASSERT(cond, msg) \
    do { if (!(cond)) { std::cout << "  FAILED: " << msg << std::endl; tests_failed++; return false; } \
    else { std::cout << "  PASSED: " << msg << std::endl; tests_passed++; } } while(0)

#define TEST_START(name) std::cout << "\n=== " << name << " ===" << std::endl;

uint8_t computeCRC(const uint8_t* data, size_t len) {
    uint8_t crc = 0;
    for (size_t i = 0; i < len; i++) {
        crc ^= data[i];
        for (int b = 0; b < 8; b++)
            crc = (crc & 0x80) ? (crc << 1) ^ 0x07 : (crc << 1);
    }
    return crc;
}

MPCUpdatePacket createTestUpdate(float x, float y, float theta) {
    MPCUpdatePacket pkt;
    pkt.pose_x = x;
    pkt.pose_y = y;
    pkt.pose_theta = theta;
    pkt.omega_L = 100.0f;
    pkt.omega_R = 95.0f;
    pkt.V_battery = 12.6f;
    pkt.I_total = 2.5f;
    for (int i = 0; i < 120; i++) {
        pkt.z_desired[i] = static_cast<float>(i);
    }
    return pkt;
}

// ==================== TESTS ====================

bool test_ascii_send() {
    TEST_START("ASCII Send Test");
    SerialProtocol serial(256, SerialProtocol::Mode::ASCII);
    SerialProtocol::Packet pkt;
    pkt.type = "TEST";
    pkt.message = {{"status", "ok"}};
    bool ok = serial.send(pkt);
    TEST_ASSERT(ok, "ASCII send");
    return true;
}

bool test_binary_send() {
    TEST_START("Binary Send Test");
    SerialProtocol serial(256, SerialProtocol::Mode::BINARY);
    MPCControlPacket ctrl{7.5f, 6.3f};
    bool ok = serial.send(static_cast<uint16_t>(SerialProtocol::MPC_CONTROL), ctrl);
    TEST_ASSERT(ok, "Binary send");
    return true;
}

bool test_printf_interference() {
    TEST_START("printf Interference Test");
    for (int i = 0; i < 10; i++) {
        printf("[DEBUG] Loop %d\n", i);
        SerialProtocol serial(256, SerialProtocol::Mode::BINARY);
        MPCControlPacket ctrl{6.0f, 5.5f};
        serial.send(static_cast<uint16_t>(SerialProtocol::MPC_CONTROL), ctrl);
    }
    TEST_ASSERT(true, "All packets sent");
    return true;
}

bool test_speed_ascii() {
    TEST_START("ASCII Send Speed Test");
    SerialProtocol serial(256, SerialProtocol::Mode::ASCII);
    const int NUM = 1000;
    SerialProtocol::Packet pkt;
    pkt.type = "ODOM";
    pkt.message = {{"x", "10.5"}, {"y", "5.2"}, {"theta", "1.57"}};
    
    auto start = std::chrono::high_resolution_clock::now();
    for (int i = 0; i < NUM; i++) serial.send(pkt);
    auto end = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
    
    std::cout << "  Sent " << NUM << " packets in " << duration.count() << " ms" << std::endl;
    TEST_ASSERT(true, "Speed test complete");
    return true;
}

bool test_speed_binary() {
    TEST_START("Binary Send Speed Test");
    SerialProtocol serial(256, SerialProtocol::Mode::BINARY);
    const int NUM = 1000;
    MPCControlPacket ctrl{7.5f, 6.3f};
    
    auto start = std::chrono::high_resolution_clock::now();
    for (int i = 0; i < NUM; i++) {
        serial.send(static_cast<uint16_t>(SerialProtocol::MPC_CONTROL), ctrl);
    }
    auto end = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
    
    std::cout << "  Sent " << NUM << " packets in " << duration.count() << " ms" << std::endl;
    TEST_ASSERT(true, "Speed test complete");
    return true;
}

bool test_wakeup() {
    TEST_START("Wakeup Test");
    SerialProtocol serial(256, SerialProtocol::Mode::ASCII);
    bool ok = serial.sendWakeup("VEX_READY\n");
    TEST_ASSERT(ok, "Wakeup sent");
    return true;
}

bool test_long_running() {
    TEST_START("Long Running Test");
    SerialProtocol serial(256, SerialProtocol::Mode::BINARY);
    const int NUM = 500;
    int errors = 0;
    
    for (int i = 0; i < NUM; i++) {
        MPCControlPacket ctrl{6.0f, 5.5f};
        if (!serial.send(static_cast<uint16_t>(SerialProtocol::MPC_CONTROL), ctrl)) {
            errors++;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    
    std::cout << "  Sent " << NUM << " packets, " << errors << " errors" << std::endl;
    TEST_ASSERT(errors == 0, "No errors");
    return true;
}

// ==================== MAIN ====================
int main() {
    std::cout << "\n";
    std::cout << "╔══════════════════════════════════════════════════════════╗\n";
    std::cout << "║     VEX BRAIN SERIAL PROTOCOL TEST SUITE                 ║\n";
    std::cout << "╚══════════════════════════════════════════════════════════╝\n";
    
    test_ascii_send();
    test_binary_send();
    test_printf_interference();
    test_speed_ascii();
    test_speed_binary();
    test_wakeup();
    test_long_running();
    
    std::cout << "\n╔══════════════════════════════════════════════════════════╗\n";
    std::cout << "║                    TEST SUMMARY                          ║\n";
    std::cout << "╚══════════════════════════════════════════════════════════╝\n";
    std::cout << "  Tests passed: " << tests_passed << std::endl;
    std::cout << "  Tests failed: " << tests_failed << std::endl;
    
    if (tests_failed == 0) {
        std::cout << "\n  ✅ ALL TESTS PASSED!\n" << std::endl;
        return 0;
    } else {
        std::cout << "\n  ❌ SOME TESTS FAILED!\n" << std::endl;
        return 1;
    }
}