#pragma once

#include <atomic>
#include <cstddef>
#include <cstdint>
#include <functional>
#include <mutex>
#include <optional>
#include <string>
#include <thread>
#include <vector>

#include "config/constants.h"

// catch bad NANO_IMU constants at compile time instead of at runtime
static_assert(NANO_IMU_RATE_HZ > 0 && NANO_IMU_RATE_HZ <= 400, "NANO_IMU_RATE_HZ must be 1-400 (game rotation vector max)");
static_assert(NANO_IMU_POLL_HZ >= NANO_IMU_RATE_HZ && NANO_IMU_POLL_HZ <= 2000, "NANO_IMU_POLL_HZ must be at least the report rate and at most 2000");
static_assert(NANO_IMU_HISTORY_SEC * NANO_IMU_RATE_HZ >= 2, "heading history needs at least 2 samples to interpolate");
static_assert(NANO_IMU_ACCURACY_TRUSTED <= 3, "report accuracy is 0-3");
static_assert(NANO_IMU_MAX_READ_ERRORS >= 1, "NANO_IMU_MAX_READ_ERRORS must be at least 1");
static_assert(NANO_IMU_REPORT_TIMEOUT_MS > 1000 / NANO_IMU_RATE_HZ, "report timeout must be longer than one report period");
static_assert(NANO_IMU_RESET_ATTEMPTS >= 1, "recover() needs at least one reset attempt");
static_assert(NANO_IMU_YAW_SIGN == 1.0 || NANO_IMU_YAW_SIGN == -1.0, "NANO_IMU_YAW_SIGN must be 1 or -1");


/**
 * @brief Adafruit BNO085 driver on the Nano's I2C bus. Tracks heading from the chip's game rotation vector.
 *
 * The chip fuses gyro and accelerometer on board (no magnetometer), calibrates its own gyro bias, and
 * stamps every report with when it was sampled. Runs its own read thread so an I2C stall doesn't block
 * the EKF loop. Heading is CCW+ in radians, stamped on the Nano's steady_clock.
 */
class Bno085 {
public:
    /**
     * @brief Bus and timing settings.
     */
    struct Config {
        std::string device = "/dev/i2c-1";
        uint8_t address = 0x4A; // 0x4B with DI pulled high
        int rateHz = NANO_IMU_RATE_HZ; // game rotation vector report rate
        int pollHz = NANO_IMU_POLL_HZ; // how often the read thread checks for a packet
        double historySec = NANO_IMU_HISTORY_SEC; // how far back headingIncrement() can look
        int maxReadErrors = NANO_IMU_MAX_READ_ERRORS; // consecutive read errors before recover()
        int reportTimeoutMs = NANO_IMU_REPORT_TIMEOUT_MS; // time without a heading report before recover()
        double yawSign = NANO_IMU_YAW_SIGN; // 1 with the chip's Z up, -1 upside down
        bool saveCalibration = NANO_IMU_SAVE_CALIBRATION; // save calibration to the chip's flash once trusted

    };

    /**
     * @brief Health counters for logging.
     */
    struct Health {
        uint32_t readErrors = 0;
        uint32_t resets = 0; // resets done by recover()
        uint32_t chipResets = 0; // resets the chip reported on its own (brownout, watchdog), each one is reconfigured
        uint8_t accuracy = 0; // game rotation vector accuracy, 0 unreliable to 3 high
        uint8_t stability = 0; // 0 unknown, 1 on table, 2 stationary, 3 stable, 4 moving
        uint8_t resetCause = 0; // last boot: 1 power on, 2 internal, 3 watchdog, 4 external, 5 other
        bool valid = false; // false until configured and accuracy reaches NANO_IMU_ACCURACY_TRUSTED
        double lastRateHz = 0.0; // heading reports per second

    };

    /**
     * @brief Constructs the driver. Nothing touches the bus until start().
     *
     * @param cfg Bus, address, and rates.
     */
    explicit Bno085(Config cfg);

    /**
     * @brief Stops the read thread and closes the bus.
     */
    ~Bno085();

    // owns a thread and a file descriptor, so it can't be copied or moved
    Bno085(const Bno085&) = delete;
    Bno085& operator=(const Bno085&) = delete;

    /**
     * @brief Opens the bus, resets the chip, checks it answers, enables its reports, and starts the read thread.
     *
     * @return bool true if the sensor answered and was configured.
     */
    bool start();

    /**
     * @brief Stops the read thread.
     */
    void stop();

    /**
     * @brief Heading change between two times from the game rotation vector (CCW+, rad).
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
    /// @brief One heading sample.
    struct ThetaSample {
        int64_t tUs = 0;
        double theta = 0.0; // unwrapped heading since start(), CCW+, rad
    };

    /// @brief One SHTP packet, header stripped.
    struct Packet {
        uint8_t channel = 0;
        int64_t readUs = 0; // steady_clock time the read started
        std::vector<uint8_t> payload; // empty when the chip had nothing to send
    };

    /**
     * @brief Soft-resets the chip, waits for it to boot, checks its product ID, and enables reports.
     *
     * @return bool true if every step succeeded.
     */
    bool resetAndConfigure();

    /**
     * @brief Turns on the chip's gyro and planar accelerometer calibration and enables the
     * game rotation vector and stability classifier reports.
     *
     * @return bool true if the chip confirmed every step.
     */
    bool configure();

    /**
     * @brief Read thread: drain packets at pollHz, handle them, and recover if the chip goes quiet.
     */
    void run();

    /**
     * @brief Resets the chip and reconfigures it after repeated read errors or a report timeout.
     */
    void recover();

    /**
     * @brief Routes one packet to its channel's handler.
     *
     * @param p Packet from readPacket().
     */
    void handlePacket(const Packet& p);

    /**
     * @brief Goes through the sensor reports in one input packet and stamps each with its sample time.
     *
     * @param p Packet on the input report channel.
     */
    void handleReports(const Packet& p);

    /**
     * @brief Adds one heading sample to the history.
     *
     * @param tUs Sample time on the Nano's steady_clock.
     * @param yaw Heading from the quaternion, wrapped to (-pi, pi].
     * @param accuracy Report accuracy, 0-3.
     */
    void addHeading(int64_t tUs, double yaw, uint8_t accuracy);

    /**
     * @brief Clears the history and marks the IMU invalid, for after any chip reset.
     */
    void invalidate();

    /**
     * @brief Sends a Set Feature command that turns on one report at a fixed interval.
     *
     * @param reportId Sensor report ID.
     * @param intervalUs Time between reports (us).
     * @return bool true if the chip confirmed the feature.
     */
    bool enableReport(uint8_t reportId, uint32_t intervalUs);

    /**
     * @brief Sends a command request on the control channel.
     *
     * @param command Command ID.
     * @param params Up to 9 parameter bytes, the rest are sent as 0.
     * @return bool true if the write succeeded.
     */
    bool sendCommand(uint8_t command, const std::vector<uint8_t>& params);

    /**
     * @brief Reads packets until one matches, dropping the rest.
     *
     * @param timeoutMs How long to wait.
     * @param match Returns true for the packet being waited for (payload is never empty).
     * @param out Filled with the matching packet.
     * @return bool false on timeout or stop().
     */
    bool waitFor(int timeoutMs, const std::function<bool(const Packet&)>& match, Packet& out);

    /**
     * @brief Heading at time t, interpolated from the history.
     *
     * @param tUs Time on the Nano's steady_clock.
     * @return std::optional<double> Nothing if t is outside the history.
     */
    std::optional<double> thetaAt(int64_t tUs) const;

    /**
     * @brief Sends one SHTP packet.
     *
     * @param channel SHTP channel.
     * @param payload Bytes after the 4-byte header.
     * @return bool true on success.
     */
    bool writePacket(uint8_t channel, const std::vector<uint8_t>& payload);

    /**
     * @brief Reads one SHTP packet: the 4-byte header for the length, then the whole packet.
     *
     * @param out Filled with the packet, payload empty if the chip had nothing to send.
     * @return bool false on an I2C error.
     */
    bool readPacket(Packet& out);


    Config m_cfg;
    int m_fd{-1}; // I2C device, -1 when closed

    std::thread m_thread;
    std::atomic<bool> m_running{false}; // true from start() until stop(), including while configuring

    // bus and read thread only
    uint8_t m_writeSeq[6]{}; // SHTP sequence number per channel
    uint8_t m_commandSeq{0}; // command request sequence number
    double m_theta{0.0}; // unwrapped heading
    double m_lastYaw{0.0}; // previous wrapped yaw, for unwrapping
    bool m_haveYaw{false}; // false until the first report after a reset
    int64_t m_lastSampleUs{0}; // last stored sample time, keeps the history increasing
    int64_t m_lastReportUs{0}; // when the last heading report was read, for the timeout
    int m_reportsSinceCheck{0};
    int m_errorStreak{0};
    bool m_calibrationSaved{false}; // true once a save was sent this run
    bool m_reconfigure{false}; // the chip reset itself and needs its reports enabled again


    // shared with headingIncrement() and health(), guarded by m_mutex
    mutable std::mutex m_mutex;
    std::vector<ThetaSample> m_history; // ring buffer of the last historySec
    size_t m_historyHead{0}; // next slot to write
    size_t m_historyCount{0};
    Health m_health;
};
