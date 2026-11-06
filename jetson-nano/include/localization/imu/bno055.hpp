#pragma once

#include <atomic>
#include <cstdint>
#include <mutex>
#include <optional>
#include <string>
#include <thread>
#include <vector>

#include "config/constants.h"

// catch bad NANO_IMU constants at compile time instead of a divide by zero or out-of-bounds write at runtime
static_assert(NANO_IMU_RATE_HZ > 0 && NANO_IMU_RATE_HZ <= 1000, "NANO_IMU_RATE_HZ must be 1-1000 (period is 1 s / rate in us)");
static_assert(NANO_IMU_HISTORY_SEC * NANO_IMU_RATE_HZ >= 2, "heading history needs at least 2 samples to interpolate");
static_assert(NANO_IMU_BIAS_SEC * NANO_IMU_RATE_HZ >= 10, "bias estimate needs at least 10 samples");
static_assert(NANO_IMU_BIAS_MAX_STDDEV > 0 && NANO_IMU_BIAS_MAX_MEAN > 0, "bias limits must be positive");
static_assert(NANO_IMU_GYRO_CALIB_TRUSTED <= 3, "CALIB_STAT gyro level is 0-3");
static_assert(NANO_IMU_MAX_READ_ERRORS >= 1, "NANO_IMU_MAX_READ_ERRORS must be at least 1");
static_assert(NANO_IMU_RESET_ATTEMPTS >= 1, "recover() needs at least one reset attempt");

/**
 * @brief Adafruit BNO055 driver on the Nano's I2C bus. Integrates gyro Z into heading.
 *
 * Runs its own read thread so an I2C stall doesn't block the EKF loop. Heading is CCW+ in radians,
 * stamped on the Nano's steady_clock.
 */
class Bno055 {
public:
    /**
     * @brief Bus and timing settings.
     */
    struct Config {
        std::string device = "/dev/i2c-1";
        uint8_t address = 0x28; // 0x29 with ADR high
        int rateHz = NANO_IMU_RATE_HZ; // sensor output rate in fusion mode
        double historySec = NANO_IMU_HISTORY_SEC; // how far back headingIncrement() can look
        int maxReadErrors = NANO_IMU_MAX_READ_ERRORS; // consecutive read errors before recover()
        std::string calibFile = NANO_IMU_CALIB_FILE; // saved gyro offsets, empty = don't save or load
    };

    /**
     * @brief Health counters for logging.
     */
    struct Health {
        uint32_t readErrors = 0;
        uint32_t resets = 0;
        uint8_t calibGyro = 0; // CALIB_STAT gyro bits, 0–3
        bool valid = false; // false until configured and the gyro is calibrated
        double lastRateHz = 0.0;
    };

    /**
     * @brief Constructs the driver. Nothing touches the bus until start().
     *
     * @param cfg Bus, address, and rate.
     */
    explicit Bno055(Config cfg);

    /**
     * @brief Stops the read thread and closes the bus.
     */
    ~Bno055();

    // owns a thread and a file descriptor, so it can't be copied or moved
    Bno055(const Bno055&) = delete;
    Bno055& operator=(const Bno055&) = delete;

    /**
     * @brief Opens the bus, checks the chip ID, sets IMU mode, and starts the read thread.
     *
     * @return bool True if the sensor answered and was configured.
     */
    bool start();

    /**
     * @brief Stops the read thread.
     */
    void stop();

    /**
     * @brief Heading change between two times from the integrated gyro (CCW+, rad).
     *
     * @param t0Us Start time on the Nano's steady_clock.
     * @param t1Us End time on the Nano's steady_clock.
     * @return std::optional<double> Nothing if either time is outside the history or the IMU is invalid.
     */
    std::optional<double> headingIncrement(int64_t t0Us, int64_t t1Us) const;

    /**
     * @brief Thread-safe copy of the health counters.
     *
     * @return Health Current counters.
     */
    Health health() const;

private:
    /// @brief One integrated heading sample.
    struct ThetaSample {
        int64_t tUs = 0;
        double theta = 0.0; // integrated heading since start(), CCW+, rad
    };

    /**
     * @brief Chip ID check, CONFIG mode, units, axis remap, IMU mode, restore calibration offsets.
     *
     * @return bool True if every step succeeded.
     */
    bool configure();

    /**
     * @brief Read thread: read gyro Z at rateHz, stamp it, subtract bias, integrate into heading.
     */
    void run();

    /**
     * @brief Resets the chip and reconfigures it after repeated read errors.
     */
    void recover();

    /**
     * @brief Averages gyro Z while the robot is still, to estimate the bias.
     *
     * @return bool True if enough samples were collected.
     */
    bool estimateBias();

    /**
     * @brief Writes saved gyro offsets from calibFile to the chip. Call only in CONFIG mode.
     *
     * @return bool True if offsets were loaded and written.
     */
    bool loadGyroOffsets();

    /**
     * @brief Reads the chip's gyro offsets in CONFIG mode, saves them to calibFile, and returns to IMU mode.
     *
     * @return bool False if the chip couldn't be put back in IMU mode.
     */
    bool saveGyroOffsets();

    /**
     * @brief Reads gyro Z (GYR_DATA_Z).
     *
     * @return std::optional<double> Turn rate, CCW+, rad/s. Nothing on a read error.
     */
    std::optional<double> readGyroZ();

    /**
     * @brief Heading at time t, interpolated from the history.
     *
     * @param tUs Time on the Nano's steady_clock.
     * @return std::optional<double> Nothing if t is outside the history.
     */
    std::optional<double> thetaAt(int64_t tUs) const;

    /**
     * @brief Writes one register.
     *
     * @param reg Register address.
     * @param value Byte to write.
     * @return bool True on success.
     */
    bool writeReg(uint8_t reg, uint8_t value);

    /**
     * @brief Reads consecutive registers starting at reg.
     *
     * @param reg First register address.
     * @param out Buffer to fill.
     * @param len Number of bytes.
     * @return bool True on success.
     */
    bool readRegs(uint8_t reg, uint8_t* out, size_t len);

    Config m_cfg;
    int m_fd{-1}; // I2C device, -1 when closed

    std::thread m_thread;
    std::atomic<bool> m_running{false};

    // read thread only
    double m_biasRadS{0.0}; // gyro Z bias, subtracted from every read
    double m_theta{0.0}; // running integral of gyro Z
    double m_lastRateRadS{0.0}; // previous reading, for trapezoidal integration
    int64_t m_lastTUs{0};
    int m_errorStreak{0};
    bool m_offsetsSaved{false}; // true once offsets were loaded or a save was tried this run

    // shared with headingIncrement() and health(), guarded by m_mutex
    mutable std::mutex m_mutex;
    std::vector<ThetaSample> m_history; // ring buffer of the last historySec
    size_t m_historyHead{0}; // next slot to write
    size_t m_historyCount{0};
    Health m_health;
};
