/**
 * @file serial.hpp
 * @brief Link layer for the V5 Brain ⇄ Jetson Nano serial link.
 *
 * Runs the protocol's behavior on top of SerialProtocol
 * Both sides compile the same file. The platform define in serialProtocol.hpp 
 * picks the brain half or the Nano half.
 *
 * Each side runs one global reader and one global writer:
 * - Brain: the writer sends STATE every tick, the reader applies CONFIG and COMMAND.
 * - Nano: the reader takes STATE and LOG, the writer replies to each STATE.
 */
#pragma once

#include <array>
#include <atomic>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <optional>
#include <vector>

#include "daydream/utils/serial/packets.hpp"
#include "daydream/utils/serial/serialProtocol.hpp"

#ifdef __vexbrain__
    #include "pros/adi.hpp"
    #include "pros/misc.hpp"
    #include "pros/motor_group.hpp"
    #include "pros/rotation.hpp"
    #include "pros/rtos.hpp"
#endif

#ifdef __microcontroller__
    #include <chrono>
    #include <condition_variable>
    #include <deque>
    #include <functional>
    #include <mutex>
    #include <string>
    #include <thread>
#endif

/**
 * @brief Runs one side of the brain ⇄ Nano link.
 *
 * Create one global instance per side and call start() once.
 */
class SerialLink {
public:
    #ifdef __microcontroller__
    /// @brief One decoded STATE, with positions rebuilt from the deltas.
    struct Sample {
        int64_t t_nano_us = 0; ///< when the brain sampled, on the Nano's steady_clock, in µs
        uint16_t state_seq = 0; ///< StateFixed::state_seq
        std::vector<int64_t> ime_cdeg; ///< per group: IME position since the config was applied, centidegrees
        std::vector<int64_t> wheel_cdeg; ///< per tracking wheel: position since the config was applied, centidegrees
        std::vector<int64_t> ime_step_cdeg; ///< per group: change since the previous Sample
        std::vector<int64_t> wheel_step_cdeg; ///< per tracking wheel: change since the previous Sample
        std::vector<float> omega_rad_s; ///< per group: output shaft speed, rad/s
        std::vector<uint8_t> group_flags; ///< per group: packets::fault bits
        std::vector<uint8_t> wheel_flags; ///< per tracking wheel: packets::fault bits
        uint8_t status = 0; ///< packets::status bits
        uint8_t last_request_id = 0; ///< last one-shot request the brain ran
        uint8_t last_error = 0; ///< packets::Error
        uint16_t battery_mV = 0; ///< battery voltage
        int8_t ctrl_axes[4] = {}; ///< master controller LX, LY, RX, RY, −127..127
        uint16_t ctrl_buttons = 0; ///< master controller buttons, see StateFixed::ctrl_buttons
        uint16_t rx_errors = 0; ///< frames the brain dropped. wraps
    };

    std::atomic<uint32_t> skipped_replies{0}; ///< ticks where the reply couldn't start within REPLY_START_MAX_US
    std::atomic<uint32_t> unknown_bases{0}; ///< STATEs whose delta base wasn't in the history
    #endif

    #ifdef __vexbrain__
    /**
     * @brief Creates the link with nothing open until start().
     *
     * @param port Smart port wired to the RS-485 transceiver.
     * @param baud Link baud rate, e.g. 921600.
     */
    SerialLink(uint8_t port, int32_t baud);
    #endif

    #ifdef __microcontroller__
    /**
     * @brief Creates the link. Nothing is opened until start().
     *
     * @param port UART device, e.g. "/dev/ttyTHS1".
     * @param baud Link baud rate, e.g. 921600.
     * @param de_gpio sysfs GPIO wired to the transceiver's DE and /RE pins. -1 = auto-direction transceiver.
     * @param config CONFIG payload to apply on the brain, built with packets::PayloadWriter.
     *        Its config_id must not be 0.
     */
    SerialLink(const char* port, int32_t baud, int de_gpio, std::vector<uint8_t> config);
    #endif

    /// @brief Stops the reader and writer.
    ~SerialLink();

    SerialLink(const SerialLink&) = delete;
    SerialLink& operator=(const SerialLink&) = delete;

    /**
     * @brief Opens the port and starts the reader and writer.
     *
     * Brain: call it from initialize(). STATE starts going out right away with
     * config_id = 0 until the Nano configures the brain.
     *
     * @return bool false if the port couldn't be opened (Nano: or the baud or config is invalid).
     */
    bool start();

    #ifdef __vexbrain__
    /**
     * @brief Queues a line of text for the Nano. Sent as LOG, at most one per tick.
     *
     * Safe to call from any task. Text past MAX_LOG_TEXT is cut off, and the
     * line is dropped if LOG_QUEUE_SIZE lines are already waiting.
     *
     * @param level Log level, passed through to the Nano as-is.
     * @param format printf-style format string.
     */
    void log(uint8_t level, const char* format, ...);
    #endif

    #ifdef __microcontroller__
    /**
     * @brief Sets a group's command. Sent in every COMMAND until changed.
     *
     * @param slot Group slot, the group's index in CONFIG.
     * @param mode How `value` and `aux` are read.
     * @param value See packets::GroupMode.
     * @param aux Max rpm in POSITION mode. Unused otherwise.
     */
    void setGroup(uint8_t slot, packets::GroupMode mode, float value, int16_t aux = 0);

    /**
     * @brief Sets an ADI output. Re-sent in every COMMAND from then on.
     *
     * @param slot ADI slot, 0–7 brain A–H, 8–15 expander A–H.
     * @param on Logical state. The brain applies adi_active_low.
     */
    void setAdi(uint8_t slot, bool on);

    /**
     * @brief Starts a one-shot request. Replaces any request still in flight.
     *
     * @param type What to run.
     * @param target Group slot for GROUP_TARE. Unused for CONTROLLER_RUMBLE.
     * @param value Pattern index for CONTROLLER_RUMBLE. Unused for GROUP_TARE.
     * @return uint8_t The request_id. Pass it to requestDone().
     */
    uint8_t request(packets::Request type, uint8_t target, float value = 0);

    /**
     * @brief Whether the brain has run a request.
     *
     * @param id ID returned by request().
     * @return bool true once the brain echoes the ID in StateFixed::last_request_id.
     */
    bool requestDone(uint8_t id) const;

    /**
     * @brief Registers a callback that runs on every new Sample.
     *
     * It runs on the reader thread, so keep it short or hand off to another
     * thread. Calling any other SerialLink function from it is fine.
     *
     * @param callback Called with each Sample.
     */
    void onSample(std::function<void(const Sample&)> callback);

    /**
     * @brief Returns the most recent Sample.
     *
     * @return std::optional<Sample> The Sample, or std::nullopt before the first one.
     */
    std::optional<Sample> latestSample() const;

    /**
     * @brief Whether STATE is arriving.
     *
     * @return bool false if no STATE arrived in the last 5 state periods.
     */
    bool linkUp() const;
    #endif

private:
    #ifdef __vexbrain__
    static constexpr size_t HISTORY_SIZE = 8; ///< STATE snapshots kept for moving the delta base
    static constexpr size_t LOG_QUEUE_SIZE = 8; ///< LOG lines waiting to be sent
    static constexpr size_t MAX_CHANNELS = packets::MAX_GROUPS + packets::MAX_ROTATIONS; ///< delta channels: groups, then rotations

    /// @brief One configured motor group.
    struct Group {
        std::unique_ptr<pros::MotorGroup> motors; ///< the group's motors
        std::vector<double> last_pos; ///< per motor: last good position, degrees
        std::vector<bool> last_ok; ///< per motor: whether the last read worked
        double accum_cdeg = 0; ///< averaged IME position since config, centidegrees
        bool missing = false; ///< a port had no motor when the config was applied
    };

    /// @brief One configured tracking wheel.
    struct Wheel {
        std::unique_ptr<pros::Rotation> sensor; ///< the rotation sensor
        int32_t last = 0; ///< last good reading, centidegrees
        bool read_failed = false; ///< the last read failed and `last` was held
    };

    /// @brief Every channel's position at one STATE.
    struct Snapshot {
        uint16_t seq = 0; ///< state_seq. 0 = config origin, or an empty slot
        int32_t ch[MAX_CHANNELS] = {}; ///< groups, then rotations, centidegrees
    };

    /// @brief One queued LOG line.
    struct LogEntry {
        uint8_t level = 0; ///< log level
        uint8_t length = 0; ///< text bytes, 1–MAX_LOG_TEXT
        char text[packets::MAX_LOG_TEXT + 1] = {}; ///< text, +1 for vsnprintf's terminator
    };

    /// @brief Reader task: applies every CONFIG and COMMAND as it arrives.
    void readerLoop();

    /// @brief Writer task: every tick, checks the watchdog, samples, sends one LOG if queued, then STATE.
    void writerLoop();

    /**
     * @brief Validates a CONFIG and, if valid, rebuilds every device.
     *
     * @param payload CONFIG payload. Its size was already checked by SerialProtocol.
     * @param size Payload size in bytes.
     * @return packets::Error NONE if applied or already applied, otherwise why it was rejected.
     */
    packets::Error applyConfig(const uint8_t* payload, uint16_t size);

    /**
     * @brief Validates a COMMAND and applies it to the groups, the ADI outputs and the delta base.
     *
     * @param payload COMMAND payload. Its size was already checked by SerialProtocol.
     * @param size Payload size in bytes.
     * @return packets::Error NONE if applied, otherwise why it was rejected.
     */
    packets::Error applyCommand(const uint8_t* payload, uint16_t size);

    /**
     * @brief Runs a one-shot request.
     *
     * @param command COMMAND carrying the request.
     * @return packets::Error NONE, or REQ_TARGET / REQ_TYPE / REQ_DEVICE / RANGE.
     */
    packets::Error runRequest(const packets::CommandFixed& command);

    /**
     * @brief Reads every channel's current position.
     *
     * @param channels Filled with groups then rotations, centidegrees. Must hold MAX_CHANNELS.
     */
    void sampleChannels(int32_t* channels);

    /**
     * @brief Trips when no valid COMMAND has arrived for watchdog_ms.
     *
     * Sets packets::status::WATCHDOG in any mode, but only brakes every group
     * (once) during autonomous. In driver control the groups are left alone.
     */
    void checkWatchdog();

    /**
     * @brief Builds StateFixed::status.
     *
     * @return uint8_t packets::status bits.
     */
    uint8_t statusBits();


    /**
     * @brief Copies the master controller's axes and buttons into STATE.
     *
     * @param fixed STATE fixed part to fill.
     */
    void readController(packets::StateFixed& fixed);

    /// @brief Sends the oldest queued LOG line, if any.
    void flushOneLog();

    SerialProtocol serial; ///< framing and the smart port
    uint8_t link_port; ///< smart port of the link. no device may use it
    int32_t baud; ///< link baud rate
    pros::Controller controller; ///< master controller, read into every STATE
    std::unique_ptr<pros::Task> reader_task; ///< runs readerLoop()
    std::unique_ptr<pros::Task> writer_task; ///< runs writerLoop()

    pros::Mutex mutex; ///< guards the device and delta state below
    std::vector<Group> groups; ///< configured groups, by slot
    std::vector<Wheel> wheels; ///< configured tracking wheels, by slot
    std::array<std::unique_ptr<pros::adi::DigitalOut>, packets::MAX_ADI> adi; ///< ADI outputs by slot. null = not enabled
    uint16_t adi_enable = 0; ///< ConfigFixed::adi_out_enable
    uint16_t adi_active_low = 0; ///< ConfigFixed::adi_active_low
    bool expander_missing = false; ///< the configured ADI expander wasn't detected
    std::array<Snapshot, HISTORY_SIZE> history; ///< recent STATE snapshots, by state_seq % HISTORY_SIZE
    Snapshot base; ///< what the deltas are measured from
    uint8_t config_id = 0; ///< applied config. 0 = not configured
    uint8_t state_period_ms = 10; ///< STATE interval
    uint16_t watchdog_ms = 100; ///< watchdog timeout
    uint16_t state_seq = 1; ///< state_seq of the next STATE. never 0
    uint16_t last_cmd_seq = 0; ///< cmd_seq of the last COMMAND applied
    uint8_t last_request_id = 0; ///< ID of the last request run
    packets::Error last_error = packets::Error::NONE; ///< latest error since the previous STATE
    bool have_cmd = false; ///< a valid COMMAND arrived since the config was applied
    bool watchdog_tripped = false; ///< the groups were braked by the watchdog
    uint32_t cmd_rx_us = 0; ///< when the last valid COMMAND arrived, pros::c::micros()
    uint32_t cmd_rx_ms = 0; ///< when the last valid COMMAND arrived, pros::millis()

    pros::Mutex log_mutex; ///< guards the log queue
    std::array<LogEntry, LOG_QUEUE_SIZE> log_queue; ///< ring buffer of queued lines
    size_t log_head = 0; ///< index of the oldest queued line
    size_t log_count = 0; ///< lines queued
    #endif

    #ifdef __microcontroller__
    using Clock = std::chrono::steady_clock;

    static constexpr size_t POS_HISTORY_SIZE = 16; ///< received positions kept, by state_seq
    static constexpr size_t CMD_TIMES_SIZE = 64; ///< COMMAND send times kept, by cmd_seq
    static constexpr int CONFIG_RETRY_TICKS = 20; ///< ticks between CONFIG resends
    static constexpr int LINK_DOWN_PERIODS = 5; ///< state periods without STATE before the link is down
    static constexpr int64_t SYNC_WINDOW_US = 1000000; ///< how long time-sync samples are kept

    /// @brief Rebuilt positions at one received STATE.
    struct PosEntry {
        uint16_t seq = 0; ///< state_seq. 0 = empty slot
        std::vector<int64_t> pos; ///< groups, then rotations, centidegrees
    };

    /// @brief One NTP-style time-sync measurement.
    struct SyncSample {
        int64_t t3; ///< when the STATE arrived, Nano µs
        int64_t rtt; ///< round trip minus the brain's hold time, µs
        int64_t offset; ///< brain clock − Nano clock, µs
    };

    /// @brief Reader thread: takes STATE and LOG, wakes the writer on each STATE.
    void readerLoop();

    /// @brief Writer thread: after each STATE, waits GUARD_US and sends CONFIG or the latest COMMAND.
    void writerLoop();

    /**
     * @brief Decodes a STATE: session tracking, request echo, time sync, positions, then publishes a Sample.
     *
     * @param frame The STATE frame.
     * @param state_end When the reader got the end of the frame.
     */
    void handleState(const SerialProtocol::Frame& frame, Clock::time_point state_end);

    /**
     * @brief Prints a LOG line to stderr.
     *
     * @param frame The LOG frame.
     */
    void handleLog(const SerialProtocol::Frame& frame);

    /**
     * @brief Adds a time-sync measurement if this STATE closes a new round trip.
     *
     * @param fixed STATE fixed part.
     * @param state_size STATE payload size in bytes.
     * @param t3 When the STATE arrived, Nano µs.
     * @return int64_t When the brain sampled, on the Nano clock, µs.
     */
    int64_t syncTime(const packets::StateFixed& fixed, uint16_t state_size, int64_t t3);

    /// @brief Forgets everything tied to the brain's previous program run.
    void resetBrainSession();

    /// @brief Restarts positions from the config origin.
    void resetPositions();

    SerialProtocol serial; ///< framing and the UART
    std::string device; ///< UART device path
    int32_t baud; ///< link baud rate
    int de_gpio; ///< DE GPIO, -1 = none
    std::vector<uint8_t> config; ///< CONFIG payload
    uint8_t my_config_id = 0; ///< config_id inside `config`
    uint8_t n_groups = 0; ///< group count inside `config`
    uint8_t state_period_ms = 10; ///< state period inside `config`

    std::atomic<bool> running{false}; ///< cleared to stop both threads
    std::thread reader; ///< runs readerLoop()
    std::thread writer; ///< runs writerLoop()

    std::mutex reply_mutex; ///< guards the reply handoff
    std::condition_variable reply_cv; ///< wakes the writer on each STATE
    Clock::time_point state_end; ///< end of the latest STATE
    bool state_pending = false; ///< a STATE arrived that the writer hasn't replied to

    std::mutex command_mutex; ///< guards the command mailbox
    packets::CommandFixed command_fixed{}; ///< latest COMMAND fixed part, minus the sequence fields
    std::vector<packets::GroupCmd> group_cmds; ///< latest command per group
    uint8_t next_request_id = 0; ///< last request_id handed out
    uint16_t cmd_seq = 0; ///< cmd_seq of the last COMMAND sent. writer only

    std::atomic<uint8_t> brain_config_id{0}; ///< StateFixed::config_id of the latest STATE
    std::atomic<uint8_t> brain_request_id{0}; ///< StateFixed::last_request_id of the latest STATE
    std::atomic<uint16_t> ack_state_seq{0}; ///< newest state_seq recorded, sent as COMMAND::ack_state_seq
    std::atomic<int64_t> last_state_us{0}; ///< when the latest STATE arrived, Nano µs. 0 = never
    std::array<std::atomic<int64_t>, CMD_TIMES_SIZE> cmd_sent_us{}; ///< COMMAND send times by cmd_seq % CMD_TIMES_SIZE

    // reader thread only
    std::array<PosEntry, POS_HISTORY_SIZE> pos_history; ///< rebuilt positions by state_seq % POS_HISTORY_SIZE
    std::vector<int64_t> last_pos; ///< positions at the previous Sample. empty = none yet
    std::deque<SyncSample> sync_window; ///< time-sync samples from the last SYNC_WINDOW_US
    int64_t best_offset = 0; ///< offset of the best recent sample
    bool have_offset = false; ///< best_offset is valid
    uint16_t last_synced_cmd = 0; ///< cmd_seq of the last COMMAND used for time sync
    uint32_t last_brain_t_us = 0; ///< previous StateFixed::t_us, for unwrapping
    int64_t brain_time_high = 0; ///< wraps of StateFixed::t_us so far, × 2^32
    bool have_brain_time = false; ///< last_brain_t_us is valid

    mutable std::mutex sample_mutex; ///< guards latest and callbacks
    std::optional<Sample> latest; ///< most recent Sample
    std::vector<std::function<void(const Sample&)>> callbacks; ///< onSample() callbacks
    #endif
};