// rplidar_lite.cpp
#include "rplidar_lite.hpp"
#include <cstring>

namespace {
const uint8_t CMD_SYNC        = 0xA5;
const uint8_t CMD_STOP        = 0x25;
const uint8_t CMD_SCAN        = 0x20;
const uint8_t CMD_GET_INFO    = 0x50;
const uint8_t CMD_MOTOR_RPM   = 0xA8;   // C1 / S-series: built-in speed control
const uint8_t CMD_MOTOR_PWM   = 0xF0;   // A2 / A3: PWM motor control
const uint8_t ANS_SYNC1       = 0xA5;
const uint8_t ANS_SYNC2       = 0x5A;
const uint8_t ANS_TYPE_INFO   = 0x04;
const uint8_t ANS_TYPE_SCAN   = 0x81;
const uint16_t DEFAULT_PWM    = 660;
}

RPLidarLite::RPLidarLite(SerialIO& io) : io_(io) {}

// Request format: A5 <cmd> [<size> <payload...> <checksum>]
// checksum = XOR of every byte before it.
void RPLidarLite::sendCommand(uint8_t cmd, const uint8_t* payload, uint8_t payloadLen) {
    uint8_t buf[3 + 255 + 1];
    int len = 0;
    buf[len++] = CMD_SYNC;
    buf[len++] = cmd;
    if (payload && payloadLen > 0) {
        buf[len++] = payloadLen;
        std::memcpy(&buf[len], payload, payloadLen);
        len += payloadLen;
        uint8_t checksum = 0;
        for (int i = 0; i < len; ++i) checksum ^= buf[i];
        buf[len++] = checksum;
    }
    io_.write(buf, len);
}

bool RPLidarLite::readExact(uint8_t* out, int len, uint32_t timeoutMs) {
    uint32_t start = io_.millis();
    int got = 0;
    while (got < len) {
        int n = io_.read(out + got, len - got);
        if (n > 0) { got += n; continue; }
        if (io_.millis() - start > timeoutMs) return false;
        io_.sleepMs(1);
    }
    return true;
}

// Response descriptor: A5 5A <4 bytes: length (30 bits) + mode (2 bits)> <type>
bool RPLidarLite::waitForDescriptor(uint8_t expectedType, uint32_t& responseLen, uint32_t timeoutMs) {
    uint32_t start = io_.millis();
    uint8_t prev = 0, b = 0;
    while (true) {
        if (io_.millis() - start > timeoutMs) return false;
        if (io_.read(&b, 1) != 1) { io_.sleepMs(1); continue; }
        if (prev == ANS_SYNC1 && b == ANS_SYNC2) break;
        prev = b;
    }
    uint8_t rest[5];
    uint32_t elapsed = io_.millis() - start;
    if (!readExact(rest, 5, timeoutMs > elapsed ? timeoutMs - elapsed : 1)) return false;
    uint32_t sizeAndMode = (uint32_t)rest[0] | ((uint32_t)rest[1] << 8) |
                           ((uint32_t)rest[2] << 16) | ((uint32_t)rest[3] << 24);
    responseLen = sizeAndMode & 0x3FFFFFFF;
    return rest[4] == expectedType;
}

void RPLidarLite::drainInput(uint32_t quietMs) {
    uint8_t junk[256];
    uint32_t lastData = io_.millis();
    while (io_.millis() - lastData < quietMs) {
        if (io_.read(junk, sizeof(junk)) > 0) lastData = io_.millis();
        else io_.sleepMs(1);
    }
    rxLen_ = 0;
}

bool RPLidarLite::getDeviceInfo(LidarDeviceInfo& info, uint32_t timeoutMs) {
    if (scanning_) stop();
    drainInput(5);
    sendCommand(CMD_GET_INFO);

    uint32_t len = 0;
    if (!waitForDescriptor(ANS_TYPE_INFO, len, timeoutMs) || len < 20) return false;
    uint8_t raw[20];
    if (!readExact(raw, 20, timeoutMs)) return false;

    info.model = raw[0];
    info.firmwareMinor = raw[1];
    info.firmwareMajor = raw[2];
    info.hardware = raw[3];
    std::memcpy(info.serial, &raw[4], 16);

    // Same rule the Slamtec SDK uses to pick how to drive the motor.
    uint8_t family = info.model >> 4;
    if (family >= 6)      motorType_ = MOTOR_RPM;   // C1, S-series
    else if (family >= 2) motorType_ = MOTOR_PWM;   // A2, A3
    else                  motorType_ = MOTOR_DTR;   // A1
    return true;
}

void RPLidarLite::setMotor(bool on) {
    uint8_t payload[2];
    switch (motorType_) {
    case MOTOR_RPM: {
        uint16_t rpm = on ? targetRpm_ : 0;
        payload[0] = rpm & 0xFF; payload[1] = rpm >> 8;
        sendCommand(CMD_MOTOR_RPM, payload, 2);
        break;
    }
    case MOTOR_PWM: {
        uint16_t pwm = on ? DEFAULT_PWM : 0;
        payload[0] = pwm & 0xFF; payload[1] = pwm >> 8;
        sendCommand(CMD_MOTOR_PWM, payload, 2);
        break;
    }
    case MOTOR_DTR:
        io_.setDTR(!on);   // A1: DTR asserted = motor off
        break;
    default:
        break;
    }
    io_.sleepMs(10);
}

bool RPLidarLite::startScan(uint32_t timeoutMs) {
    // Make sure the lidar is idle, then identify it (this also picks the motor type).
    sendCommand(CMD_STOP);
    io_.sleepMs(20);
    LidarDeviceInfo info;
    if (!getDeviceInfo(info, timeoutMs)) return false;

    setMotor(true);
    sendCommand(CMD_SCAN);

    uint32_t len = 0;
    if (!waitForDescriptor(ANS_TYPE_SCAN, len, timeoutMs) || len != 5) {
        sendCommand(CMD_STOP);
        setMotor(false);
        return false;
    }

    rxLen_ = 0;
    buildingCount_ = 0;
    sawFirstStart_ = false;
    scanning_ = true;
    return true;
}

void RPLidarLite::stop() {
    sendCommand(CMD_STOP);
    io_.sleepMs(20);
    setMotor(false);
    scanning_ = false;
}

// One measurement packet (5 bytes):
//   byte0: bit0 = start of new revolution (S), bit1 = inverse of S, bits2-7 = quality
//   byte1-2: bit0 of byte1 = check bit (always 1), remaining 15 bits = angle * 64
//   byte3-4: distance * 4 (mm)
//
// Timestamps: the lidar samples at a steady rate, so each point's time is placed
// between the revolution's start and end according to its sample number.
// Times are when the PC/Brain received the data, so they include a small
// transmission delay (a few ms over USB, less over a direct UART).
void RPLidarLite::handleNode(const uint8_t* n, uint64_t arrivalUs) {
    bool newRevolution = (n[0] & 0x01) != 0;
    uint16_t angleQ6 = (uint16_t)(((uint16_t)n[1] | ((uint16_t)n[2] << 8)) >> 1);
    uint16_t distQ2 = (uint16_t)n[3] | ((uint16_t)n[4] << 8);

    if (newRevolution) {
        if (sawFirstStart_ && buildingCount_ > 0 && buildingSamples_ > 0) {
            uint64_t duration = arrivalUs - buildingStartUs_;
            for (int i = 0; i < buildingCount_; ++i) {
                latest_[i] = building_[i];
                latest_[i].timeUs = buildingStartUs_ +
                    duration * (uint64_t)buildingIndex_[i] / (uint64_t)buildingSamples_;
            }
            latestCount_ = buildingCount_;
            latestTiming_.startUs = buildingStartUs_;
            latestTiming_.endUs = arrivalUs;
            latestTiming_.samples = buildingSamples_;
            ++completedScans_;
        }
        sawFirstStart_ = true;
        buildingCount_ = 0;
        buildingSamples_ = 0;
        buildingStartUs_ = arrivalUs;
    }
    if (!sawFirstStart_) return;                 // skip the partial first turn

    int sampleIndex = buildingSamples_++;
    if (distQ2 == 0) return;                     // invalid reading
    if (buildingCount_ >= kMaxPoints) return;

    buildingIndex_[buildingCount_] = (uint16_t)sampleIndex;
    LidarPoint& p = building_[buildingCount_++];
    p.angleDeg = angleQ6 / 64.0f;
    p.distanceMm = distQ2 / 4.0f;
    p.quality = n[0] >> 2;
    p.timeUs = 0;                                // filled in when the revolution completes
}

bool RPLidarLite::update() {
    if (!scanning_) return false;
    uint32_t before = completedScans_;

    // Bounded loop so one call never blocks the caller for long.
    for (int iter = 0; iter < 16; ++iter) {
        int space = (int)sizeof(rx_) - rxLen_;
        int n = io_.read(rx_ + rxLen_, space);
        if (n <= 0) break;
        uint64_t arrivalUs = io_.micros();
        rxLen_ += n;

        int pos = 0;
        while (rxLen_ - pos >= 5) {
            const uint8_t* p = rx_ + pos;
            bool s = p[0] & 0x01, notS = (p[0] >> 1) & 0x01, check = p[1] & 0x01;
            if (s != notS && check) {
                handleNode(p, arrivalUs);
                pos += 5;
            } else {
                ++badPackets_;   // out of step with the stream: slide by one byte
                pos += 1;
            }
        }
        std::memmove(rx_, rx_ + pos, rxLen_ - pos);
        rxLen_ -= pos;
    }
    return completedScans_ != before;
}

int RPLidarLite::getScan(LidarPoint* out, int maxPoints, LidarScanTiming* timing) const {
    if (timing) *timing = latestTiming_;
    int n = latestCount_ < maxPoints ? latestCount_ : maxPoints;
    std::memcpy(out, latest_, sizeof(LidarPoint) * n);
    return n;
}