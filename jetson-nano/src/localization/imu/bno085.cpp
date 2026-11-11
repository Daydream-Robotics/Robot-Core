#include "localization/imu/bno085.hpp"

#include "config/constants.h"
#include "utils/mathHelpers.hpp"

#include <chrono>
#include <cmath>
#include <cstdio>
#include <functional>
#include <utility>

#include <fcntl.h>
#include <linux/i2c-dev.h>
#include <sys/ioctl.h>
#include <unistd.h>

namespace {

// SHTP and SH-2 values
// SHTP channels
constexpr uint8_t CH_EXECUTABLE = 1; // reset in, reset complete out
constexpr uint8_t CH_CONTROL = 2; // commands, feature settings, product ID
constexpr uint8_t CH_REPORTS = 3; // sensor reports
constexpr uint8_t CH_WAKE_REPORTS = 4; // sensor reports enabled as wake-up sensors

// executable channel
constexpr uint8_t EXEC_RESET = 0x01; // host -> chip: soft reset
constexpr uint8_t EXEC_RESET_COMPLETE = 0x01; // chip -> host: finished booting

// control channel report IDs
constexpr uint8_t COMMAND_RESPONSE = 0xF1;
constexpr uint8_t COMMAND_REQUEST = 0xF2;
constexpr uint8_t PRODUCT_ID_RESPONSE = 0xF8;
constexpr uint8_t PRODUCT_ID_REQUEST = 0xF9;
constexpr uint8_t GET_FEATURE_RESPONSE = 0xFC;
constexpr uint8_t SET_FEATURE = 0xFD;

// command IDs
constexpr uint8_t CMD_SAVE_DCD = 0x06; // save calibration (dynamic calibration data) to the chip's flash
constexpr uint8_t CMD_ME_CALIBRATION = 0x07; // pick which sensors the chip calibrates while running

// input report IDs
constexpr uint8_t TIMESTAMP_REBASE = 0xFA;
constexpr uint8_t BASE_TIMESTAMP = 0xFB;
constexpr uint8_t GAME_ROTATION_VECTOR = 0x08; // gyro + accel quaternion, no magnetometer
constexpr uint8_t STABILITY_CLASSIFIER = 0x13;

constexpr int STABILITY_RATE_HZ = 10;
constexpr double Q14 = 1.0 / (1 << 14); // quaternion fixed point scale
constexpr size_t MAX_PACKET = 4096; // larger lengths mean a bad read (a floating bus reads 0x7FFF)


/**
 * @brief Current steady_clock time.
 *
 * @return int64_t Microseconds since a fixed point.
 */
int64_t nowUs() {
    return std::chrono::duration_cast<std::chrono::microseconds>(
               std::chrono::steady_clock::now().time_since_epoch()).count();
}

/**
 * @brief Reads a little-endian int16.
 *
 * @param p First byte.
 * @return int16_t The value.
 */
int16_t le16(const uint8_t* p) {
    return static_cast<int16_t>(p[0] | (p[1] << 8));
}

/**
 * @brief Reads a little-endian int32.
 *
 * @param p First byte.
 * @return int32_t The value.
 */
int32_t le32(const uint8_t* p) {
    return static_cast<int32_t>(static_cast<uint32_t>(p[0]) | (static_cast<uint32_t>(p[1]) << 8) |
                                (static_cast<uint32_t>(p[2]) << 16) | (static_cast<uint32_t>(p[3]) << 24));
}

/**
 * @brief Length of an input report, so the parser can find the next one in a packet.
 *
 * @param id Report ID.
 * @return size_t Bytes including the ID, 0 for a report this driver doesn't enable.
 */
size_t reportLength(uint8_t id) {
    switch (id) {
        case TIMESTAMP_REBASE:
        case BASE_TIMESTAMP:
            return 5;
        case GAME_ROTATION_VECTOR:
            return 12;
        case STABILITY_CLASSIFIER:
            return 6;
        default:
            return 0;
    }
}


} 

Bno085::Bno085(Config cfg) : m_cfg(std::move(cfg)) {}

Bno085::~Bno085() {
    // join the read thread first so it can't touch the bus after it's closed
    stop();
    // -1 means the bus was never opened (or is closed)
    if (m_fd >= 0) {
        close(m_fd);
        m_fd = -1;
    }
}

bool Bno085::start() {
    // already running
    if (m_running) {
        return true;
    }

    // the static asserts only cover the NANO_IMU defaults, so check a caller's own Config here
    if (m_cfg.rateHz <= 0 || m_cfg.rateHz > 400) {
        std::fprintf(stderr, "[bno085] rateHz %d out of range (1-400)\n", m_cfg.rateHz);
        return false;
    }
    if (m_cfg.pollHz < m_cfg.rateHz || m_cfg.pollHz > 2000) {
        std::fprintf(stderr, "[bno085] pollHz %d must be between rateHz and 2000\n", m_cfg.pollHz);
        return false;
    }
    if (m_cfg.historySec * m_cfg.rateHz < 2) {
        std::fprintf(stderr, "[bno085] historySec %.3f too short, need at least 2 samples\n", m_cfg.historySec);
        return false;
    }
    if (m_cfg.maxReadErrors < 1 || m_cfg.reportTimeoutMs * m_cfg.rateHz <= 1000) {
        std::fprintf(stderr, "[bno085] maxReadErrors must be at least 1 and reportTimeoutMs longer than one report\n");
        return false;
    }


    // open the I2C bus
    m_fd = open(m_cfg.device.c_str(), O_RDWR);
    if (m_fd < 0) {
        std::fprintf(stderr, "[bno085] can't open %s\n", m_cfg.device.c_str());
        return false;
    }
    // point later reads/writes on this handle at the BNO085's address
    if (ioctl(m_fd, I2C_SLAVE, m_cfg.address) < 0) {
        std::fprintf(stderr, "[bno085] no device at 0x%02X\n", m_cfg.address);
        close(m_fd);
        m_fd = -1;
        return false;
    }

    // one slot per sample over historySec, allocated now so the read thread doesn't resize it
    m_history.assign(static_cast<size_t>(m_cfg.historySec * m_cfg.rateHz), ThetaSample{});
    
    invalidate();
    m_calibrationSaved = false;

    m_running = true;

    if (!resetAndConfigure()) {
        // the chip isn't answering, so close the bus
        m_running = false;
        close(m_fd);
        m_fd = -1;
        return false;
    }

    // run() on this object in its own thread
    m_thread = std::thread(&Bno085::run, this);
    return true;
}

void Bno085::stop() {
    // tell run() to exit its loop
    m_running = false;
    // wait for the thread to finish. joinable() is false if it was never started or is already joined
    if (m_thread.joinable()) {
        m_thread.join();
    }
    // close the bus so a later start() opens a fresh handle instead of leaking this one
    if (m_fd >= 0) {
        close(m_fd);
        m_fd = -1;
    }
}

std::optional<double> Bno085::headingIncrement(int64_t t0Us, int64_t t1Us) const {
    {
        std::lock_guard<std::mutex> lock(m_mutex);
        if (!m_health.valid){
            return std::nullopt;
        }
    }
    // integrated heading at both times. nothing if either is outside the history
    std::optional<double> theta0 = thetaAt(t0Us);
    std::optional<double> theta1 = thetaAt(t1Us);
    if (!theta0 || !theta1) {
        return std::nullopt;
    }

    // change in heading between the two times, CCW+
    return *theta1 - *theta0;
}

Bno085::Health Bno085::health() const {
    std::lock_guard<std::mutex> lock(m_mutex);
    return m_health;
}

bool Bno085::resetAndConfigure() {
    using namespace std::chrono_literals;

    // soft reset, retried for about 1s in case the chip is still booting and NAKs the write
    bool sent = false;
    for (int i = 0; i < 10 && !sent && m_running; ++i) {
        sent = writePacket(CH_EXECUTABLE, {EXEC_RESET});
        if (!sent) {
            std::this_thread::sleep_for(100ms);
        }
    }
    // stop() was called mid-retry
    if (!m_running) {
        return false;
    }
    if (!sent) {
        std::fprintf(stderr, "[bno085] no answer at 0x%02X, check wiring and that PS0/PS1 are low (I2C mode)\n", m_cfg.address);
        return false;
    }

    // after booting, the chip sends its advertisement and then reset complete on the executable channel
    Packet p;
    auto resetComplete = [](const Packet& q) {
        return q.channel == CH_EXECUTABLE && q.payload[0] == EXEC_RESET_COMPLETE;
    };
    if (!waitFor(1000, resetComplete, p)) {
        std::fprintf(stderr, "[bno085] chip didn't finish booting after reset\n");
        return false;
    }

    // product ID: proves an SH-2 chip is answering, and says why it last booted
    auto productId = [](const Packet& q) {
        return q.channel == CH_CONTROL && q.payload[0] == PRODUCT_ID_RESPONSE && q.payload.size() >= 16;
    };
    if (!writePacket(CH_CONTROL, {PRODUCT_ID_REQUEST, 0x00}) || !waitFor(500, productId, p)) {
        std::fprintf(stderr, "[bno085] no product ID response\n");
        return false;
    }
    {
        std::lock_guard<std::mutex> lock(m_mutex);
        m_health.resetCause = p.payload[1];
    }
    std::fprintf(stderr, "[bno085] firmware %u.%u.%u, reset cause %u\n", p.payload[2], p.payload[3],
                 static_cast<unsigned>(le16(&p.payload[12])), p.payload[1]);

    m_reconfigure = false;
    return configure();
}

bool Bno085::configure() {
    // calibrate accel and gyro while running.
    // params: P0 accel, P1 gyro, P2 mag, P3 subcommand (0 = configure), P4 planar accel
    if (!sendCommand(CMD_ME_CALIBRATION, {1, 1, 0, 0, 1})) {
        return false;
    }
    Packet p;
    auto meResponse = [](const Packet& q) {
        return q.channel == CH_CONTROL && q.payload[0] == COMMAND_RESPONSE && q.payload.size() >= 6 &&
               q.payload[2] == CMD_ME_CALIBRATION;
    };
    if (!waitFor(300, meResponse, p)) {
        std::fprintf(stderr, "[bno085] no response to the calibration setup\n");
        return false;
    }
    // R0 (byte 5) is the status, 0 = success. Keep going on failure, the chip still runs with its default calibration
    if (p.payload[5] != 0) {
        std::fprintf(stderr, "[bno085] calibration setup returned status %u, using the chip's defaults\n", p.payload[5]);
    }

    // heading source, and whether the robot is still (for logs and later zero-velocity updates)
    return enableReport(GAME_ROTATION_VECTOR, 1'000'000 / m_cfg.rateHz) &&
           enableReport(STABILITY_CLASSIFIER, 1'000'000 / STABILITY_RATE_HZ);
}


void Bno085::run() {
    // monotonic clock for scheduling
    using clock = std::chrono::steady_clock;

    // time between polls: 1 s / 400 Hz = 2.5 ms
    const auto period = std::chrono::microseconds(1'000'000 / m_cfg.pollHz);
    // when the next poll is due
    auto nextTick = clock::now();

    // for the once-a-second health update
    auto lastHealthCheck = clock::now();
    m_reportsSinceCheck = 0;
    // the timeout counts from now, not from the last report before start() returned
    m_lastReportUs = nowUs();

    // runs until stop() sets m_running to false
    while (m_running) {
        // wait until this poll's slot, then schedule the next one
        nextTick += period;
        std::this_thread::sleep_until(nextTick);

        // read everything queued
        for (int i = 0; i < 16 && m_running; ++i) {
            Packet p;
            if (!readPacket(p)) {
                m_errorStreak++;
                std::lock_guard<std::mutex> lock(m_mutex);
                m_health.readErrors++;
                break;
            }
            // good read ends any error streak
            m_errorStreak = 0;
            // empty packet means the chip has nothing more to send
            if (p.payload.empty()) {
                break;
            }
            handlePacket(p);
        }

        // the chip reset itself
        if (m_reconfigure) {
            m_reconfigure = false;
            if (!configure()) {
                recover();
            }
            m_lastReportUs = nowUs();
            nextTick = clock::now();
            continue;
        }

        // too many read errors, or the chip went quiet then reset it (takes about 1 s)
        const bool timedOut = nowUs() - m_lastReportUs > static_cast<int64_t>(m_cfg.reportTimeoutMs) * 1000;
        if (m_errorStreak >= m_cfg.maxReadErrors || timedOut) {
            recover();
            m_errorStreak = 0;
            m_lastReportUs = nowUs();
            // restart the schedule from now
            nextTick = clock::now();
            continue;
        }

        // about once a second: report rate into health, and save calibration once it's trusted
        const auto now = clock::now();
        if (now - lastHealthCheck >= std::chrono::seconds(1)) {
            const double seconds = std::chrono::duration<double>(now - lastHealthCheck).count();
            bool valid = false;
            {
                std::lock_guard<std::mutex> lock(m_mutex);
                m_health.lastRateHz = m_reportsSinceCheck / seconds;
                valid = m_health.valid;
            }
            lastHealthCheck = now;
            m_reportsSinceCheck = 0;

            // the chip keeps calibration in its own flash, so the next boot starts calibrated
            if (valid && m_cfg.saveCalibration && !m_calibrationSaved) {
                m_calibrationSaved = true;
                sendCommand(CMD_SAVE_DCD, {});
            }
        }
    }
}


void Bno085::recover() {
    {
        std::lock_guard<std::mutex> lock(m_mutex);
        m_health.resets++;
    }
    // headingIncrement() can't span the reset gap
    invalidate();

    // reset and reconfigure, retrying up to NANO_IMU_RESET_ATTEMPTS times
    bool configured = false;
    for (int attempt = 0; attempt < NANO_IMU_RESET_ATTEMPTS && m_running && !configured; ++attempt) {
        configured = resetAndConfigure();
    }
    if (!configured && m_running) {
        std::fprintf(stderr, "[bno085] reconfigure after reset failed\n");
    }
}


void Bno085::handlePacket(const Packet& p) {
    switch (p.channel) {
        case CH_EXECUTABLE:
            // a reset complete we didn't ask for: brownout, watchdog, or the reset pin. Every report is off now
            if (p.payload[0] == EXEC_RESET_COMPLETE) {
                {
                    std::lock_guard<std::mutex> lock(m_mutex);
                    m_health.chipResets++;
                }
                invalidate();
                m_reconfigure = true;
                std::fprintf(stderr, "[bno085] chip reset itself, reconfiguring\n");
            }
            break;
        case CH_CONTROL:
            // the only command run() sends is the calibration save, R0 (byte 5) is its status
            if (p.payload[0] == COMMAND_RESPONSE && p.payload.size() >= 6 && p.payload[2] == CMD_SAVE_DCD) {
                if (p.payload[5] != 0) {
                    std::fprintf(stderr, "[bno085] calibration save failed, status %u\n", p.payload[5]);
                }
            }
            break;
        case CH_REPORTS:
        case CH_WAKE_REPORTS:
            handleReports(p);
            break;
        default:
            // advertisement and anything else isn't needed
            break;
    }
}

void Bno085::handleReports(const Packet& p) {
    // without the INT pin the chip raised its interrupt sometime in the last poll period, so take the middle
    const int64_t hostIntUs = p.readUs - 500'000 / m_cfg.pollHz;
    // offset from hostIntUs to the batch's base time, in 100 us ticks. Same math as CEVA's sh2.c:
    // sample time = interrupt time - base delta + rebase + report delay
    int64_t referenceTicks = 0;

    const std::vector<uint8_t>& b = p.payload;
    size_t i = 0;
    while (i < b.size()) {
        const uint8_t id = b[i];
        const size_t len = reportLength(id);
        if (len == 0 || i + len > b.size()) {
            break;
        }
        const uint8_t* r = &b[i];

        switch (id) {
            case BASE_TIMESTAMP:
                referenceTicks = -static_cast<int64_t>(le32(r + 1));
                break;
            case TIMESTAMP_REBASE:
                referenceTicks += le32(r + 1);
                break;
            case GAME_ROTATION_VECTOR: {
                // status byte: bits 1:0 accuracy, bits 7:2 the top 6 bits of the 14-bit delay
                const uint8_t accuracy = r[2] & 0x03;
                const int64_t delayTicks = ((r[2] & 0xFC) << 6) | r[3];
                int64_t tUs = hostIntUs + (referenceTicks + delayTicks) * 100;
                // a sample can't be newer than the read that brought it
                if (tUs > p.readUs) {
                    tUs = p.readUs;
                }

                // unit quaternion i, j, k, real in Q14
                const double qi = le16(r + 4) * Q14;
                const double qj = le16(r + 6) * Q14;
                const double qk = le16(r + 8) * Q14;
                const double qr = le16(r + 10) * Q14;
                // yaw about the chip's Z axis, CCW+, (-pi, pi]
                const double yaw = std::atan2(2.0 * (qr * qk + qi * qj), 1.0 - 2.0 * (qj * qj + qk * qk));
                addHeading(tUs, yaw, accuracy);
                break;
            }
            case STABILITY_CLASSIFIER: {
                std::lock_guard<std::mutex> lock(m_mutex);
                m_health.stability = r[4];
                break;
            }
            default:
                break;
        }
        i += len;
    }
}

void Bno085::addHeading(int64_t tUs, double yaw, uint8_t accuracy) {
    yaw *= m_cfg.yawSign;
    // unwrap and add the short way around from the last yaw, so theta keeps counting past +-pi
    if (m_haveYaw) {
        m_theta += utils::wrapAngle(yaw - m_lastYaw);
    }
    m_lastYaw = yaw;
    m_haveYaw = true;

    // check if strictly increasing
    if (tUs <= m_lastSampleUs) {
        tUs = m_lastSampleUs + 1;
    }
    m_lastSampleUs = tUs;
    m_lastReportUs = nowUs();
    m_reportsSinceCheck++;

    // store (time, heading) in the ring buffer, overwriting the oldest sample when full
    std::lock_guard<std::mutex> lock(m_mutex);
    m_history[m_historyHead] = {tUs, m_theta};
    m_historyHead = (m_historyHead + 1) % m_history.size();
    if (m_historyCount < m_history.size()) {
        m_historyCount++;
    }
    // the chip's own accuracy estimate decides validity, updated every report
    m_health.accuracy = accuracy;
    m_health.valid = accuracy >= NANO_IMU_ACCURACY_TRUSTED;
}

void Bno085::invalidate() {
    // the next report starts a new unwrap, theta carries on from where it was
    m_haveYaw = false;
    std::lock_guard<std::mutex> lock(m_mutex);
    m_health.valid = false;
    m_health.accuracy = 0;
    m_historyHead = 0;
    m_historyCount = 0;
}


std::optional<double> Bno085::thetaAt(int64_t tUs) const {
    std::lock_guard<std::mutex> lock(m_mutex);

    // no samples yet
    if (m_historyCount == 0) {
        return std::nullopt;
    }

    const size_t size = m_history.size();
    // newest sample is just before the head, oldest is count slots before that
    const size_t newestIdx = (m_historyHead + size - 1) % size;
    const size_t oldestIdx = (m_historyHead + size - m_historyCount) % size;

    // outside the stored time range
    if (tUs < m_history[oldestIdx].tUs || tUs > m_history[newestIdx].tUs) {
        return std::nullopt;
    }

    // walk back from the newest sample to the first one at or before tUs
    size_t after = newestIdx;
    for (size_t k = 0; k < m_historyCount; ++k) {
        const size_t idx = (newestIdx + size - k) % size;
        const ThetaSample& s = m_history[idx];

        // exact match or the bracketing pair is found
        if (s.tUs <= tUs) {
            if (s.tUs == tUs || idx == after) {
                return s.theta;
            }
            const ThetaSample& next = m_history[after];
            // fraction of the way from s to next
            const double frac = static_cast<double>(tUs - s.tUs) / (next.tUs - s.tUs);
            return s.theta + frac * (next.theta - s.theta);
        }
        after = idx;
    }

    return std::nullopt;
}

bool Bno085::enableReport(uint8_t reportId, uint32_t intervalUs) {
    // Set Feature: ID, report, flags, change sensitivity (2), report interval (4), batch interval (4), sensor config (4)
    std::vector<uint8_t> cmd(17, 0);
    cmd[0] = SET_FEATURE;
    cmd[1] = reportId;
    for (int b = 0; b < 4; ++b) {
        cmd[5 + b] = static_cast<uint8_t>(intervalUs >> (8 * b));
    }
    if (!writePacket(CH_CONTROL, cmd)) {
        return false;
    }

    // the chip answers with the settings it actually used
    Packet p;
    auto featureResponse = [reportId](const Packet& q) {
        return q.channel == CH_CONTROL && q.payload[0] == GET_FEATURE_RESPONSE && q.payload.size() >= 9 &&
               q.payload[1] == reportId;
    };
    if (!waitFor(300, featureResponse, p)) {
        std::fprintf(stderr, "[bno085] report 0x%02X not confirmed\n", reportId);
        return false;
    }
    const uint32_t actualUs = static_cast<uint32_t>(le32(&p.payload[5]));
    if (actualUs != intervalUs) {
        std::fprintf(stderr, "[bno085] report 0x%02X runs every %u us, asked for %u us\n", reportId, actualUs, intervalUs);
    }
    return true;
}

bool Bno085::sendCommand(uint8_t command, const std::vector<uint8_t>& params) {
    // Command Request: ID, sequence, command, P0-P8
    std::vector<uint8_t> cmd(12, 0);
    cmd[0] = COMMAND_REQUEST;
    cmd[1] = m_commandSeq++;
    cmd[2] = command;
    for (size_t i = 0; i < params.size() && i < 9; ++i) {
        cmd[3 + i] = params[i];
    }
    return writePacket(CH_CONTROL, cmd);
}

bool Bno085::waitFor(int timeoutMs, const std::function<bool(const Packet&)>& match, Packet& out) {
    using namespace std::chrono_literals;
    const int64_t deadline = nowUs() + static_cast<int64_t>(timeoutMs) * 1000;

    while (m_running && nowUs() < deadline) {
        // read errors are expected while the chip boots, so keep trying until the deadline
        if (!readPacket(out) || out.payload.empty()) {
            std::this_thread::sleep_for(1ms);
            continue;
        }
        if (match(out)) {
            return true;
        }
    }
    return false;
}


bool Bno085::writePacket(uint8_t channel, const std::vector<uint8_t>& payload) {
    // bus not open
    if (m_fd < 0) {
        return false;
    }

    // header: total length (LSB, MSB), channel, this channel's sequence number
    const size_t len = payload.size() + 4;
    std::vector<uint8_t> buf;
    buf.reserve(len);
    buf.push_back(static_cast<uint8_t>(len & 0xFF));
    buf.push_back(static_cast<uint8_t>(len >> 8));
    buf.push_back(channel);
    buf.push_back(m_writeSeq[channel]++);
    buf.insert(buf.end(), payload.begin(), payload.end());

    // whole packet as one I2C write
    return write(m_fd, buf.data(), buf.size()) == static_cast<ssize_t>(buf.size());
}

bool Bno085::readPacket(Packet& out) {
    out.payload.clear();
    out.readUs = nowUs();

    // bus not open
    if (m_fd < 0) {
        return false;
    }

    // the header says how long the waiting packet is, 0 when there's nothing
    uint8_t header[4];
    if (read(m_fd, header, sizeof(header)) != static_cast<ssize_t>(sizeof(header))) {
        return false;
    }
    // bit 15 is the continuation flag, not part of the length
    const size_t len = (header[0] | (header[1] << 8)) & 0x7FFF;
    out.channel = header[2];
    if (len <= sizeof(header)) {
        return true;
    }
    if (len > MAX_PACKET) {
        return false;
    }

    // every I2C read starts at the header again, so read the whole packet and skip it
    std::vector<uint8_t> buf(len);
    if (read(m_fd, buf.data(), len) != static_cast<ssize_t>(len)) {
        return false;
    }
    out.channel = buf[2];
    out.payload.assign(buf.begin() + 4, buf.end());
    return true;
}

