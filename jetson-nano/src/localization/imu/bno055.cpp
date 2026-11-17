#include "localization/imu/bno055.hpp"

#include "config/constants.h"

#include "utils/mathHelpers.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <utility>

#include <fcntl.h>
#include <linux/i2c-dev.h>
#include <sys/ioctl.h>
#include <unistd.h>

namespace {

// BNO055 registers. Check every value against the Bosch BNO055 datasheet.
constexpr uint8_t REG_CHIP_ID = 0x00; // reads CHIP_ID_VALUE
constexpr uint8_t REG_PAGE_ID = 0x07;
constexpr uint8_t REG_GYR_DATA_Z_LSB = 0x18; // 0x18 LSB, 0x19 MSB
constexpr uint8_t REG_CALIB_STAT = 0x35; // gyro calibration is bits 5:4
constexpr uint8_t REG_UNIT_SEL = 0x3B;
constexpr uint8_t REG_OPR_MODE = 0x3D;
constexpr uint8_t REG_PWR_MODE = 0x3E;
constexpr uint8_t REG_SYS_TRIGGER = 0x3F; // bit 5 resets the chip
constexpr uint8_t REG_AXIS_MAP_CONFIG = 0x41;
constexpr uint8_t REG_AXIS_MAP_SIGN = 0x42;
constexpr uint8_t REG_GYR_OFFSET_X_LSB = 0x61; // gyro calibration offsets, 0x61–0x66

constexpr uint8_t CHIP_ID_VALUE = 0xA0;
constexpr uint8_t MODE_CONFIG = 0x00;
constexpr uint8_t MODE_IMU = 0x08; // accelerometer + gyro fusion, no magnetometer

constexpr double GYRO_LSB_PER_DPS = 16.0; // default scale, in dps units
constexpr double DEG_TO_RAD = utils::PI / 180.0;

/**
 * @brief Current steady_clock time, on the same scale as the serial link's nowUs().
 *
 * @return int64_t Microseconds since a fixed point.
 */
int64_t nowUs() {
    return std::chrono::duration_cast<std::chrono::microseconds>(
               std::chrono::steady_clock::now().time_since_epoch()).count();
}

} // namespace

Bno055::Bno055(Config cfg) : m_cfg(std::move(cfg)) {}

Bno055::~Bno055() {
    // join the read thread first so it can't touch the bus after it's closed
    stop();
    // -1 means the bus was never opened (or is already closed)
    if (m_fd >= 0) {
        close(m_fd);
        m_fd = -1;
    }
}

bool Bno055::start() {
    // already running, nothing to do
    if (m_running) {
        return true;
    }

    // the static asserts only cover the NANO_IMU defaults, so check a caller's own Config here
    if (m_cfg.rateHz <= 0 || m_cfg.rateHz > 1000) {
        std::fprintf(stderr, "[bno055] rateHz %d out of range (1-1000)\n", m_cfg.rateHz);
        return false;
    }
    if (m_cfg.historySec * m_cfg.rateHz < 2) {
        std::fprintf(stderr, "[bno055] historySec %.3f too short, need at least 2 samples\n", m_cfg.historySec);
        return false;
    }
    if (m_cfg.maxReadErrors < 1) {
        std::fprintf(stderr, "[bno055] maxReadErrors must be at least 1\n");
        return false;
    }

    // open the I2C bus, where m_fd is the handle every later read/write uses
    m_fd = open(m_cfg.device.c_str(), O_RDWR);
    if (m_fd < 0) {
        std::fprintf(stderr, "[bno055] can't open %s\n", m_cfg.device.c_str());
        return false;
    }
    // point later reads/writes on this handle at the BNO055's address
    if (ioctl(m_fd, I2C_SLAVE, m_cfg.address) < 0) {
        std::fprintf(stderr, "[bno055] no device at 0x%02X\n", m_cfg.address);
        close(m_fd);
        m_fd = -1;
        return false;
    }

    // one slot per sample over historySec, allocated now so the read thread never resizes it
    m_history.assign(static_cast<size_t>(m_cfg.historySec * m_cfg.rateHz), ThetaSample{});
    // empty ring buffer
    m_historyHead = 0;
    m_historyCount = 0;

    // set before configure() so stop() can cut its chip ID retries short, and before the thread since run() loops while m_running
    m_running = true;

    // set IMU mode, then measure gyro bias (robot must be still)
    const BiasResult bias = configure() ? estimateBias() : BiasResult::READ_ERRORS;
    if (bias == BiasResult::READ_ERRORS) {
        // the chip isn't answering, so close the bus so a retry starts clean
        m_running = false;
        close(m_fd);
        m_fd = -1;
        return false;
    }

    // a moving robot gives a wrong bias, but run() re-measures it once the chip calibrates,
    // so keep going and flag it for whoever reads health()
    {
        std::lock_guard<std::mutex> lock(m_mutex);
        m_health.movedAtStart = bias == BiasResult::MOVING;
    }
    if (bias == BiasResult::MOVING) {
        std::fprintf(stderr,
                     "[bno055] ******** WARNING: ROBOT MOVED DURING IMU STARTUP ********\n"
                     "[bno055] heading stays invalid until the robot sits still after the gyro calibrates\n");
    }

    // run() on this object in its own thread
    m_thread = std::thread(&Bno055::run, this);
    return true;
}

void Bno055::stop() {
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

std::optional<double> Bno055::headingIncrement(int64_t t0Us, int64_t t1Us) const {
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

Bno055::Health Bno055::health() const {
    std::lock_guard<std::mutex> lock(m_mutex);
    return m_health;
}

bool Bno055::configure() {
    using namespace std::chrono_literals;

    // select register page 0 (the chip ID and all config registers are on page 0)
    if (!writeReg(REG_PAGE_ID, 0x00)) {
        return false;
    }

    // check we're talking to a BNO055 and retry briefly in case it's still booting
    uint8_t id = 0;
    for (int i = 0; i < 10 && id != CHIP_ID_VALUE && m_running; ++i) {
        // a failed read is seen as a wrong ID, so the loop retries
        if (!readRegs(REG_CHIP_ID, &id, 1)) {
            id = 0;
        }
        // only wait if this attempt didn't succeed
        if (id != CHIP_ID_VALUE) {
            std::this_thread::sleep_for(100ms);
        }
    }
    // stop() was called mid-retry, so give up without blaming the wiring
    if (!m_running) {
        return false;
    }
    // still wrong after ~1 s: wrong address, or wiring problem
    if (id != CHIP_ID_VALUE) {
        std::fprintf(stderr, "[bno055] chip ID 0x%02X, expected 0x%02X\n", id, CHIP_ID_VALUE);
        return false;
    }

    // config registers can only be written in CONFIG mode, writes in other modes are ignored
    if (!writeReg(REG_OPR_MODE, MODE_CONFIG)) {
        return false;
    }
    // switching any mode -> CONFIG takes 19 ms
    std::this_thread::sleep_for(25ms);

    // normal power mode (0x00): all sensors always on, no sleeping between samples
    if (!writeReg(REG_PWR_MODE, 0x00)) {
        return false;
    }

    // units: bit 1 = 0 -> gyro in dps (readGyroZ() converts to rad/s), bit 7 = 0 -> Windows orientation
    // the other bits (accel m/s², Euler degrees, temp °C) are 0 too; only the gyro unit matters
    if (!writeReg(REG_UNIT_SEL, 0x00)) {
        return false;
    }

    // axis remap: datasheet defaults (0x24 / 0x00) keep the chip's own axes
    // AXIS_MAP_CONFIG picks which chip axis becomes X/Y/Z; AXIS_MAP_SIGN flips their signs
    // TODO: change once the board's mounting is known, so gyro Z is up and CCW+
    if (!writeReg(REG_AXIS_MAP_CONFIG, 0x24) || !writeReg(REG_AXIS_MAP_SIGN, 0x00)) {
        return false;
    }

    // restore saved gyro offsets, which can only be written in CONFIG mode
    if (loadGyroOffsets()) {
        m_offsetsSaved = true;
    }

    // IMU fusion mode (accel + gyro, 100 Hz, no magnetometer); this also starts the sensors
    if (!writeReg(REG_OPR_MODE, MODE_IMU)) {
        return false;
    }
    // switching CONFIG -> any mode takes 7 ms; 10 ms adds margin before the first read
    std::this_thread::sleep_for(10ms);

    return true;
}

void Bno055::run() {
    // monotonic clock for scheduling
    using clock = std::chrono::steady_clock;

    // time between reads: 1 s / 100 Hz = 10 ms
    const auto period = std::chrono::microseconds(1000000 / m_cfg.rateHz);
    // when the next read is due
    auto nextTick = clock::now();

    // for the once-a-second health update
    auto lastHealthCheck = clock::now();
    // good reads since the last health update, used to measure the actual rate
    int samplesSinceCheck = 0;

    // first sample has nothing to integrate from (0 = no previous sample)
    m_lastTUs = 0;

    // runs until stop() sets m_running to false
    while (m_running) {
        // wait until this read's slot, then schedule the next one
        nextTick += period;
        std::this_thread::sleep_until(nextTick);

        // read turn rate (rad/s, CCW+, bias already subtracted)
        std::optional<double> rate = readGyroZ();
        if (!rate) {
            // count the error; reset the chip after maxReadErrors in a row
            m_errorStreak++;
            // lock while changing shared health
            {
                std::lock_guard<std::mutex> lock(m_mutex);
                m_health.readErrors++;
            }
            if (m_errorStreak >= m_cfg.maxReadErrors) {
                // reset and reconfigure the chip (takes ~650 ms)
                recover();
                m_errorStreak = 0;
                // integration restarts after a reset
                m_lastTUs = 0;
                // restart the schedule from now
                nextTick = clock::now();
            }
            // skip to the next slot
            continue;
        }
        // good read ends any error streak
        m_errorStreak = 0;

        // integrate rate into heading with the trapezoid rule: average of this and last rate * time step
        const int64_t t = nowUs();
        // integrate only if there's a previous sample
        if (m_lastTUs != 0) {
            // time since the last good read, µs -> s
            const double dt = (t - m_lastTUs) * 1e-6;
            m_theta += 0.5 * (*rate + m_lastRateRadS) * dt;
        }
        // remember this read for the next step
        m_lastTUs = t;
        m_lastRateRadS = *rate;
        samplesSinceCheck++;

        // store (time, heading) in the ring buffer, overwriting the oldest sample when full
        {
            // lock while writing shared history
            std::lock_guard<std::mutex> lock(m_mutex);
            m_history[m_historyHead] = {t, m_theta};
            // move to the next slot, wrapping to 0 at the end
            m_historyHead = (m_historyHead + 1) % m_history.size();
            // count grows until the buffer is full, then stays at its size
            if (m_historyCount < m_history.size()) {
                m_historyCount++;
            }
        }

        // re-measure the bias from the normal reads, so integration never pauses for it
        if (m_biasWindowActive) {
            m_biasWindowSum += *rate;
            m_biasWindowSumSq += *rate * *rate;
            m_biasWindowCount++;

            if (m_biasWindowCount >= static_cast<int>(NANO_IMU_BIAS_SEC * m_cfg.rateHz)) {
                // no log on failure, since this retries every window while the robot moves
                if (applyBiasStats(m_biasWindowSum, m_biasWindowSumSq, m_biasWindowCount, false)) {
                    m_biasSettled = true;
                    m_biasWindowActive = false;
                    std::lock_guard<std::mutex> lock(m_mutex);
                    // older samples were integrated with the stale bias, so headingIncrement() can't span them
                    m_historyHead = 0;
                    m_historyCount = 0;
                    m_health.valid = m_health.calibGyro >= NANO_IMU_GYRO_CALIB_TRUSTED;
                }
                // robot moved during the window: start another and stay invalid until one is still
                m_biasWindowSum = 0.0;
                m_biasWindowSumSq = 0.0;
                m_biasWindowCount = 0;
            }
        }

        // about once a second: calibration and status into health
        const auto now = clock::now();
        if (now - lastHealthCheck >= std::chrono::seconds(1)) {
            // read calibration status
            uint8_t calib = 0;
            const bool calibOk = readRegs(REG_CALIB_STAT, &calib, 1);
            // time since the last health update, in seconds
            const double seconds = std::chrono::duration<double>(now - lastHealthCheck).count();

            bool calibrated = false;
            {
                std::lock_guard<std::mutex> lock(m_mutex);
                if (calibOk) {
                    // gyro calibration is bits 5:4 of CALIB_STAT: shift down, keep 2 bits (0-3)
                    m_health.calibGyro = (calib >> 4) & 0x03;
                }
                calibrated = m_health.calibGyro >= NANO_IMU_GYRO_CALIB_TRUSTED;
                // valid once the gyro reports full calibration and the bias was re-measured after it
                m_health.valid = calibrated && m_biasSettled;
                m_health.lastRateHz = samplesSinceCheck / seconds;
            }

            // start the next one-second window
            lastHealthCheck = now;
            samplesSinceCheck = 0;

            if (!calibrated) {
                // the chip's offset may change again before it's trusted, so measure the bias after that
                m_biasSettled = false;
                m_biasWindowActive = false;
            } else if (!m_biasSettled && !m_biasWindowActive) {
                // the chip now removes its own offset, so the bias measured before is stale
                m_biasWindowActive = true;
                m_biasWindowSum = 0.0;
                m_biasWindowSumSq = 0.0;
                m_biasWindowCount = 0;
            }

            // save the offsets the first time the gyro is fully calibrated
            if (calibrated && !m_offsetsSaved && !m_cfg.calibFile.empty()) {
                // reset the chip if it couldn't go back to IMU mode
                if (!saveGyroOffsets()) {
                    recover();
                    m_lastTUs = 0;
                }
                // the save paused the loop, so restart the schedule instead of catching up with back-to-back reads
                nextTick = clock::now();
            }
        }
    }
}

void Bno055::recover() {
    // mark the IMU invalid and count the reset
    {
        std::lock_guard<std::mutex> lock(m_mutex);
        m_health.valid = false;
        m_health.resets++;
        // the reset clears the chip's calibration, so don't trust the level read before it
        m_health.calibGyro = 0;
        // clear the history so headingIncrement() can't span the reset gap
        m_historyHead = 0;
        m_historyCount = 0;
    }
    // the chip recalibrates after the reset, so the bias must be re-measured once it's trusted again
    m_biasSettled = false;
    m_biasWindowActive = false;

    // reset and reconfigure, retrying up to NANO_IMU_RESET_ATTEMPTS times
    bool configured = false;
    for (int attempt = 0; attempt < NANO_IMU_RESET_ATTEMPTS && m_running && !configured; ++attempt) {
        // reset the chip with the RST_SYS bit (bit 5 of SYS_TRIGGER)
        // the write may fail if the chip is stuck, so wait and reconfigure either way
        writeReg(REG_SYS_TRIGGER, 0x20);

        // the chip takes 650 ms to boot after a reset
        std::this_thread::sleep_for(std::chrono::milliseconds(650));

        // set IMU mode again
        configured = configure();
    }
    if (!configured) {
        std::fprintf(stderr, "[bno055] reconfigure after reset failed\n");
        return;
    }

    // re-measure gyro bias, keep the old bias if the robot is moving
    if (estimateBias() != BiasResult::OK) {
        std::fprintf(stderr, "[bno055] bias not re-estimated, keeping previous bias\n");
    }

    // integration restarts from the next good read
    m_lastRateRadS = 0.0;
}

Bno055::BiasResult Bno055::estimateBias() {
    // one sample per 1 / rateHz for NANO_IMU_BIAS_SEC
    const auto period = std::chrono::microseconds(1'000'000 / m_cfg.rateHz);
    const int wanted = static_cast<int>(NANO_IMU_BIAS_SEC * m_cfg.rateHz);

    double sum = 0.0;
    double sumSq = 0.0;
    int count = 0;

    for (int i = 0; i < wanted; ++i) {
        std::this_thread::sleep_for(period);

        // readGyroZ() already subtracts the current bias, so this is the leftover error
        std::optional<double> rate = readGyroZ();
        if (!rate) {
            continue;
        }
        sum += *rate;
        sumSq += *rate * *rate;
        count++;
    }
    // need most of the samples to trust the average
    if (count < wanted * 8 / 10) {
        std::fprintf(stderr, "[bno055] bias: only %d of %d reads succeeded\n", count, wanted);
        return BiasResult::READ_ERRORS;
    }

    return applyBiasStats(sum, sumSq, count, true) ? BiasResult::OK : BiasResult::MOVING;
}

bool Bno055::applyBiasStats(double sum, double sumSq, int count, bool logIfMoving) {
    // mean and standard deviation of the leftover rate
    const double mean = sum / count;
    const double stdDev = std::sqrt(std::max(0.0, sumSq / count - mean * mean));

    // a still robot reads near-constant rate, so a big spread or a big mean means it was moving
    if (stdDev > NANO_IMU_BIAS_MAX_STDDEV || std::abs(mean) > NANO_IMU_BIAS_MAX_MEAN) {
        if (logIfMoving) {
            std::fprintf(stderr, "[bno055] bias: robot moving (mean %.4f, std %.4f rad/s)\n", mean, stdDev);
        }
        return false;
    }

    // add the leftover error to the bias already being subtracted
    m_biasRadS += mean;
    return true;
}

std::optional<double> Bno055::readGyroZ() {
    // GYR_DATA_Z is two bytes, LSB at 0x18 and MSB at 0x19
    uint8_t buf[2];
    if (!readRegs(REG_GYR_DATA_Z_LSB, buf, 2)) {
        return std::nullopt;
    }

    // combine little-endian bytes into a signed 16-bit value
    const int16_t raw = static_cast<int16_t>((buf[1] << 8) | buf[0]);

    // raw counts -> dps (16 counts per dps) -> rad/s, minus the bias
    return raw / GYRO_LSB_PER_DPS * DEG_TO_RAD - m_biasRadS;
}

std::optional<double> Bno055::thetaAt(int64_t tUs) const {
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

bool Bno055::loadGyroOffsets() {
    if (m_cfg.calibFile.empty()) {
        return false;
    }

    // 6 bytes: gyro X, Y, Z offsets, each LSB then MSB
    uint8_t buf[6];
    std::FILE* f = std::fopen(m_cfg.calibFile.c_str(), "rb");
    if (!f) {
        return false;
    }
    const size_t got = std::fread(buf, 1, sizeof(buf), f);
    std::fclose(f);
    if (got != sizeof(buf)) {
        std::fprintf(stderr, "[bno055] %s is not a gyro offset file, ignoring it\n", m_cfg.calibFile.c_str());
        return false;
    }

    // write the offsets one register at a time, starting at GYR_OFFSET_X_LSB
    for (size_t i = 0; i < sizeof(buf); ++i) {
        if (!writeReg(static_cast<uint8_t>(REG_GYR_OFFSET_X_LSB + i), buf[i])) {
            return false;
        }
    }
    return true;
}

bool Bno055::saveGyroOffsets() {
    using namespace std::chrono_literals;

    // try once per run, so a failed save doesn't pause the gyro every second
    m_offsetsSaved = true;

    // offsets can only be read in CONFIG mode
    if (!writeReg(REG_OPR_MODE, MODE_CONFIG)) {
        return false;
    }
    std::this_thread::sleep_for(25ms);

    // 6 bytes: gyro X, Y, Z offsets, each LSB then MSB
    uint8_t buf[6];
    const bool readOk = readRegs(REG_GYR_OFFSET_X_LSB, buf, sizeof(buf));

    // back to IMU mode
    if (!writeReg(REG_OPR_MODE, MODE_IMU)) {
        return false;
    }
    std::this_thread::sleep_for(10ms);

    if (!readOk) {
        std::fprintf(stderr, "[bno055] couldn't read gyro offsets\n");
        return true;
    }

    std::FILE* f = std::fopen(m_cfg.calibFile.c_str(), "wb");
    if (!f || std::fwrite(buf, 1, sizeof(buf), f) != sizeof(buf)) {
        std::fprintf(stderr, "[bno055] couldn't write %s\n", m_cfg.calibFile.c_str());
    }
    if (f) {
        std::fclose(f);
    }
    return true;
}

bool Bno055::writeReg(uint8_t reg, uint8_t value) {
    //bus not open
    if (m_fd < 0) {
        return false;
    }

    // register address first, then the value, sent as one I2C write
    const uint8_t buf[2] = {reg, value};
    // write() returns how many bytes were sent, anything other than 2 is a failure
    return write(m_fd, buf, 2) == 2;
}

bool Bno055::readRegs(uint8_t reg, uint8_t* out, size_t len) {
    // bus not open
    if (m_fd < 0) {
        return false;
    }

    // tell the chip which register to start reading from
    if (write(m_fd, &reg, 1) != 1) {
        return false;
    }

    // read len bytes, the chip moves to the next register after each byte
    const ssize_t got = read(m_fd, out, len);
    return got == static_cast<ssize_t>(len);
}
