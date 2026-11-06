// rplidar_lite.hpp
// A small, dependency-free driver for Slamtec RPLIDAR sensors (written for the C1).
// It only needs a SerialIO implementation, so the same code runs on a PC
// (WinSerialIO) and on a VEX V5 Brain under PROS (ProsSerialIO).
//
// Uses the standard SCAN command (0xA5 0x20), which streams 5-byte measurement packets.
#pragma once
#include <cstdint>

// ---- What the driver needs from the hardware -------------------------------
class SerialIO {
public:
    virtual ~SerialIO() {}
    // Send bytes. Returns how many were written.
    virtual int write(const uint8_t* data, int len) = 0;
    // Read up to maxLen bytes WITHOUT waiting. Returns how many were read (0 if none).
    virtual int read(uint8_t* buffer, int maxLen) = 0;
    // Milliseconds since some fixed start point.
    virtual uint32_t millis() = 0;
    // Microseconds since the same start point. Used for timestamps.
    virtual uint64_t micros() { return (uint64_t)millis() * 1000; }
    virtual void sleepMs(uint32_t ms) = 0;
    // Only used by the A1, whose motor is switched by the DTR line. No-op elsewhere.
    virtual void setDTR(bool /*asserted*/) {}
};

// ---- Data types ---------------------------------------------------------------
struct LidarPoint {
    float angleDeg;     // 0..360, clockwise when looking down at the lidar
    float distanceMm;   // always > 0 (invalid readings are dropped)
    uint8_t quality;    // 0..63, higher is a stronger return
    uint64_t timeUs;    // estimated time this point was measured (SerialIO::micros() clock)
};

struct LidarScanTiming {
    uint64_t startUs;   // time the revolution started (its first sample arrived)
    uint64_t endUs;     // time the next revolution started
    int samples;        // samples in the revolution, including dropped invalid ones
};

struct LidarDeviceInfo {
    uint8_t model;      // upper 4 bits = model family
    uint8_t firmwareMajor;
    uint8_t firmwareMinor;
    uint8_t hardware;
    uint8_t serial[16];
};

// ---- The driver ---------------------------------------------------------------
class RPLidarLite {
public:
    static const int kMaxPoints = 2048;   // points stored per revolution

    explicit RPLidarLite(SerialIO& io);

    // Ask the lidar who it is. Good first test that wiring and baud rate are right.
    bool getDeviceInfo(LidarDeviceInfo& info, uint32_t timeoutMs = 1000);

    // Spin up the motor and start streaming measurements.
    bool startScan(uint32_t timeoutMs = 2000);

    // Stop streaming and stop the motor. Always call this before your program exits.
    void stop();

    // Call this often (every 5-10 ms). It reads whatever bytes have arrived and
    // returns true when a new full revolution has just been completed.
    bool update();

    // Copy the most recent full revolution into `out`. Returns the number of points.
    // If `timing` is given, it receives the revolution's start/end time.
    int getScan(LidarPoint* out, int maxPoints, LidarScanTiming* timing = nullptr) const;

    uint32_t scanCount() const { return completedScans_; }  // revolutions completed so far
    bool isScanning() const { return scanning_; }
    uint32_t badPacketCount() const { return badPackets_; } // nonzero means noise/lost bytes

    // Motor speed for lidars with built-in speed control (C1, S-series).
    void setTargetRpm(uint16_t rpm) { targetRpm_ = rpm; }

private:
    enum MotorType { MOTOR_UNKNOWN, MOTOR_DTR, MOTOR_PWM, MOTOR_RPM };

    void sendCommand(uint8_t cmd, const uint8_t* payload = nullptr, uint8_t payloadLen = 0);
    bool waitForDescriptor(uint8_t expectedType, uint32_t& responseLen, uint32_t timeoutMs);
    bool readExact(uint8_t* out, int len, uint32_t timeoutMs);
    void drainInput(uint32_t quietMs);
    void setMotor(bool on);
    void handleNode(const uint8_t* n, uint64_t arrivalUs);

    SerialIO& io_;
    MotorType motorType_ = MOTOR_UNKNOWN;
    uint16_t targetRpm_ = 600;            // 600 rpm = 10 revolutions per second
    bool scanning_ = false;

    // Incoming byte buffer for packet parsing
    uint8_t rx_[512];
    int rxLen_ = 0;

    // Revolution being filled, and the last completed one
    LidarPoint building_[kMaxPoints];
    uint16_t buildingIndex_[kMaxPoints];  // sample number of each stored point
    int buildingCount_ = 0;
    int buildingSamples_ = 0;
    uint64_t buildingStartUs_ = 0;
    bool sawFirstStart_ = false;
    LidarPoint latest_[kMaxPoints];
    int latestCount_ = 0;
    LidarScanTiming latestTiming_ = {0, 0, 0};
    uint32_t completedScans_ = 0;
    uint32_t badPackets_ = 0;
};