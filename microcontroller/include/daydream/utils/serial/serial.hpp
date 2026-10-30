/**
 * @file serial.hpp
 * @brief link layer for the V5 Brain ⇄ Jetson Nano serial link
 *
 * runs the protocol's behavior on top of SerialProtocol
 * both sides compile the same file. the platform define in serialProtocol.hpp 
 * picks the brain half or the Nano half
 *
 * each side runs one global reader and one global writer:
 * - Brain: the writer sends STATE every tick, the reader applies CONFIG and COMMAND.
 * - Nano: the reader takes STATE and LOG, the writer replies to each STATE
 */
#pragma once

#include <array>
#include <atomic>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <optional>
#include <vector>

#include "daydream/config/constants.h"
#include "daydream/utils/serial/packets.hpp"
#include "daydream/utils/serial/serialProtocol.hpp"

#ifdef __v5_brain__
    #include "pros/adi.hpp"
    #include "pros/misc.hpp"
    #include "pros/motor_group.hpp"
    #include "pros/rotation.hpp"
    #include "pros/rtos.hpp"
#endif

#ifdef __jetson_nano__
    #include <chrono>
    #include <condition_variable>
    #include <deque>
    #include <functional>
    #include <mutex>
    #include <string>
    #include <thread>
#endif

/*
 * built-in CONFIG, shared by the brain and the Nano
 *
 * the brain applies it at boot, and the Nano passes the same bytes to
 * SerialLink. A CONFIG only goes out when the Nano's copy
 * differs from what the brain is running
 *
 * its configId is computed from the config's bytes so IDs are 
 * automatically generated,
 */
namespace packets {
    constexpr uint8_t LEFT_DRIVE_SLOT = 0; ///< group packet slot of the left drive
    constexpr uint8_t RIGHT_DRIVE_SLOT = 1; ///< group packet slot of the right drive
    constexpr uint8_t PARALLEL_WHEEL_SLOT = 0; ///< rotation packet slot of the parallel tracking wheel
    constexpr uint8_t PERPENDICULAR_WHEEL_SLOT = 1; ///< rotation packet slot of the perpendicular tracking wheel

    /**
     * @brief ADI mask bit for one of the brain's own ports
     *
     * @param port Brain ADI port, 'A'–'H'
     * @return uint16_t the bit for that slot
     */
    constexpr uint16_t adiBit(char port) {
        return static_cast<uint16_t>(1u << (port - 'A'));
    }

    /**
     * @brief builds the robot's built-in CONFIG payload
     *
     * the configId comes from the rest of the payload, so it changes
     * whenever the config does
     *
     * @return std::vector<uint8_t> CONFIG payload: fixed part, groups, then rotations
     */
    inline std::vector<uint8_t> buildLinkConfig() {
        constexpr uint8_t GROUPS = 2;
        constexpr uint8_t ROTATIONS = 2;
        std::vector<uint8_t> config(configSize(GROUPS, ROTATIONS));
        PayloadWriter writer{config.data()};

        ConfigFixed fixed{};
        fixed.statePeriodMs = 10;
        fixed.watchdogMs = 100;
        fixed.nGroups = GROUPS;
        fixed.nRotations = ROTATIONS;
        // no expander, so only the brain's ports A–H can be used
        fixed.adiExpanderPort = 0;
        // pneumatics on brain ports A–D, all starting off
        fixed.adiOutEnable = adiBit(BALL_BLOCKER_PORT) | adiBit(SCORING_LIFTER) |
                             adiBit(MATCHLOADER_PORT) | adiBit(DESCORE_PORT);
        writer.put(fixed);

        // groups, in slot order
        const uint8_t blue = static_cast<uint8_t>(Gearset::BLUE);
        const uint8_t coast = static_cast<uint8_t>(BrakeMode::COAST);
        writer.put(GroupCfg{3, blue, coast, LEFT_DRIVE_WHEEL_PORTS});
        writer.put(GroupCfg{3, blue, coast, RIGHT_DRIVE_WHEEL_PORTS});

        // tracking wheels, in slot order
        writer.put(RotationCfg{PARALLEL_TRACKING_WHEEL_PORT, 0});
        writer.put(RotationCfg{PERPENDICULAR_TRACKING_WHEEL_PORT, 0});

        // configId (byte 0) = CRC of everything after it
        uint16_t crc = SerialProtocol::crc16(config.data() + 1, config.size() - 1);
        config[0] = static_cast<uint8_t>(crc % 255 + 1);
        return config;
    }
}

/**
 * @brief runs one side of the brain ⇄ Nano link
 *
 * create one global instance per side and call start() once
 */
class SerialLink {
public:
    #ifdef __jetson_nano__
    /// @brief one decoded STATE, with positions rebuilt from the deltas
    struct Sample {
        int64_t tNanoUs = 0; ///< when the brain sampled, on the Nano's steady_clock, in µs
        uint16_t stateSeq = 0; ///< StateFixed::stateSeq
        std::vector<double> imeRad; ///< per group: IME position since the config was applied, radians
        std::vector<double> wheelRad; ///< per tracking wheel: position since the config was applied, radians
        std::vector<double> imeStepRad; ///< per group: change since the previous Sample, radians
        std::vector<double> wheelStepRad; ///< per tracking wheel: change since the previous Sample, radians
        std::vector<float> omegaRadS; ///< per group: output shaft speed, rad/s
        std::vector<uint8_t> groupFlags; ///< per group: packets::fault bits
        std::vector<uint8_t> wheelFlags; ///< per tracking wheel: packets::fault bits
        uint8_t status = 0; ///< packets::status bits
        uint8_t lastRequestId = 0; ///< last one-shot request the brain ran
        uint8_t lastError = 0; ///< packets::Error
        uint16_t batteryMv = 0; ///< battery voltage
        int8_t ctrlAxes[4] = {}; ///< master controller LX, LY, RX, RY
        uint16_t ctrlButtons = 0; ///< master controller buttons, see StateFixed::ctrlButtons
        uint16_t rxErrors = 0; ///< frames the brain dropped. wraps
    };

    std::atomic<uint32_t> skippedReplies{0}; ///< ticks where the reply couldn't start within REPLY_START_MAX_US
    std::atomic<uint32_t> unknownBases{0}; ///< STATEs whose delta base wasn't in the history
    #endif

    #ifdef __v5_brain__
    /**
     * @brief creates the link with nothing open until start()
     *
     * @param port smart port wired to the RS-485 transceiver
     * @param baud link baud rate, e.g. 921600
     * @param config built-in CONFIG payload, applied by start(). normally packets::buildLinkConfig()
     * @param transport RS485 (the default) or USB. over USB, `port` and `baud` are ignored
     */
    SerialLink(uint8_t port, int32_t baud, std::vector<uint8_t> config,
               SerialProtocol::Transport transport = SerialProtocol::Transport::RS485);
    #endif

    #ifdef __jetson_nano__
    /**
     * @brief creates the link. nothing is opened until start()
     *
     * @param port UART device: "/dev/ttyTHS1" for RS-485, or "/dev/ttyACM1" for the brain's USB port
     * @param baud link baud rate, e.g. 921600. over USB any supported rate works
     * @param deGpio sysfs GPIO wired to the transceiver's DE and /RE pins. -1 = auto-direction transceiver or USB
     * @param config CONFIG payload to apply on the brain, built with packets::PayloadWriter.
     *        its configId must not be 0
     * @param transport RS485 (the default) or USB. over USB, time sync skips the airtime correction
     */
    SerialLink(const char* port, int32_t baud, int deGpio, std::vector<uint8_t> config,
               SerialProtocol::Transport transport = SerialProtocol::Transport::RS485);
    #endif

    /// @brief stops the reader and writer
    ~SerialLink();

    SerialLink(const SerialLink&) = delete;
    SerialLink& operator=(const SerialLink&) = delete;

    /**
     * @brief opens the port and starts the reader and writer
     *
     * Brain: call it from initialize(). it applies the built-in config, then
     * starts sending STATE. the Nano only sends CONFIG if its config differs
     *
     * @return bool false if the port couldn't be opened (Nano: or the baud or config is invalid)
     */
    bool start();

    #ifdef __v5_brain__
    /**
     * @brief queues a line of text for the Nano. sent as LOG, at most one per tick
     *
     * safe to call from any task. text past MAX_LOG_TEXT is cut off, and the
     * line is dropped if LOG_QUEUE_SIZE lines are already waiting
     *
     * @param level log level, passed through to the Nano as-is
     * @param format printf-style format string
     */
    void log(uint8_t level, const char* format, ...);
    #endif

    #ifdef __jetson_nano__
    /**
     * @brief sets a group's command. sent in every COMMAND until changed
     *
     * @param slot group slot, the group's index in CONFIG
     * @param mode how `value` and `aux` are read
     * @param value see packets::GroupMode
     * @param aux max rpm in POSITION mode. unused otherwise
     */
    void setGroup(uint8_t slot, packets::GroupMode mode, float value, int16_t aux = 0);

    /**
     * @brief sets an ADI output. re-sent in every COMMAND from then on
     *
     * @param slot ADI slot, 0–7 brain A–H, 8–15 expander A–H
     * @param on logical state. the brain applies adiActiveLow
     */
    void setAdi(uint8_t slot, bool on);

    /**
     * @brief starts a one-shot request. replaces any request still in flight
     *
     * @param type what to run
     * @param target group slot for GROUP_TARE. unused for CONTROLLER_RUMBLE
     * @param value pattern index for CONTROLLER_RUMBLE. unused for GROUP_TARE
     * @return uint8_t the requestId. pass it to requestDone()
     */
    uint8_t request(packets::Request type, uint8_t target, float value = 0);

    /**
     * @brief whether the brain has run a request
     *
     * @param id ID returned by request()
     * @return bool true once the brain echoes the ID in StateFixed::lastRequestId
     */
    bool requestDone(uint8_t id) const;

    /**
     * @brief registers a callback that runs on every new Sample
     *
     * it runs on the reader thread, so keep it short or hand off to another
     * thread. calling any other SerialLink function from it is fine
     *
     * @param callback called with each Sample
     */
    void onSample(std::function<void(const Sample&)> callback);

    /**
     * @brief returns the most recent Sample
     *
     * @return std::optional<Sample> the Sample, or std::nullopt before the first one
     */
    std::optional<Sample> latestSample() const;

    /**
     * @brief whether STATE is arriving
     *
     * @return bool false if no STATE arrived in the last 5 state periods
     */
    bool linkUp() const;
    #endif

private:
    #ifdef __v5_brain__
    static constexpr size_t HISTORY_SIZE = 8; ///< STATE snapshots kept for moving the delta base
    static constexpr size_t LOG_QUEUE_SIZE = 8; ///< LOG lines waiting to be sent
    static constexpr size_t MAX_CHANNELS = packets::MAX_GROUPS + packets::MAX_ROTATIONS; ///< delta channels: groups, then rotations

    /// @brief one configured motor group
    struct Group {
        std::unique_ptr<pros::MotorGroup> motors; ///< the group's motors
        std::vector<double> lastPos; ///< per motor: last good position, degrees
        std::vector<bool> lastValid; ///< per motor: whether the last read worked
        double accumRad = 0; ///< averaged IME position since config, radians
        bool missing = false; ///< a port had no motor when the config was applied
    };

    /// @brief one configured tracking wheel
    struct Wheel {
        std::unique_ptr<pros::Rotation> sensor; ///< the rotation sensor
        double last = 0; ///< last good reading, radians
        bool readFailed = false; ///< the last read failed and `last` was held
    };

    /// @brief every channel's position at one STATE
    struct Snapshot {
        uint16_t seq = 0; ///< stateSeq. 0 = config origin, or an empty slot
        double ch[MAX_CHANNELS] = {}; ///< groups, then rotations, radians
    };

    /// @brief one queued LOG line
    struct LogEntry {
        uint8_t level = 0; ///< log level
        uint8_t length = 0; ///< text bytes, 1–MAX_LOG_TEXT
        char text[packets::MAX_LOG_TEXT + 1] = {}; ///< text, +1 for vsnprintf's terminator
    };

    /// @brief reader task: applies every CONFIG and COMMAND as it arrives
    void readerLoop();

    /// @brief writer task: every tick, checks the watchdog, samples, sends one LOG if queued, then STATE
    void writerLoop();

    /**
     * @brief validates a CONFIG and, if valid, rebuilds every device
     *
     * @param payload CONFIG payload. its size was already checked by SerialProtocol
     * @param size payload size in bytes
     * @return packets::Error NONE if applied or already applied, otherwise why it was rejected
     */
    packets::Error applyConfig(const uint8_t* payload, uint16_t size);

    /**
     * @brief validates a COMMAND and applies it to the groups, the ADI outputs and the delta base
     *
     * @param payload COMMAND payload. its size was already checked by SerialProtocol
     * @param size payload size in bytes
     * @return packets::Error NONE if applied, otherwise why it was rejected
     */
    packets::Error applyCommand(const uint8_t* payload, uint16_t size);

    /**
     * @brief runs a one-shot request
     *
     * @param command COMMAND carrying the request
     * @return packets::Error NONE, or REQ_TARGET / REQ_TYPE / REQ_DEVICE / RANGE
     */
    packets::Error runRequest(const packets::CommandFixed& command);

    /**
     * @brief reads every channel's current position
     *
     * @param channels filled with groups then rotations, radians. must hold MAX_CHANNELS
     */
    void sampleChannels(double* channels);

    /**
     * @brief trips when no valid COMMAND has arrived for watchdogMs
     *
     * sets packets::status::WATCHDOG in any mode, but only brakes every group
     * (once) during autonomous. in driver control the groups are left alone
     */
    void checkWatchdog();

    /**
     * @brief builds StateFixed::status
     *
     * @return uint8_t packets::status bits
     */
    uint8_t statusBits();


    /**
     * @brief copies the master controller's axes and buttons into STATE
     *
     * @param fixed STATE fixed part to fill
     */
    void readController(packets::StateFixed& fixed);

    /// @brief sends the oldest queued LOG line, if any
    void flushOneLog();

    SerialProtocol m_serial; ///< framing and the smart port
    uint8_t m_linkPort; ///< smart port of the link. no device may use it
    int32_t m_baud; ///< link baud rate
    SerialProtocol::Transport m_transport; ///< what the link runs over
    pros::Controller m_controller; ///< master controller, read into every STATE
    std::unique_ptr<pros::Task> m_readerTask; ///< runs readerLoop()
    std::unique_ptr<pros::Task> m_writerTask; ///< runs writerLoop()

    pros::Mutex m_mutex; ///< guards the device and delta state below
    std::vector<Group> m_groups; ///< configured groups, by slot
    std::vector<Wheel> m_wheels; ///< configured tracking wheels, by slot
    std::array<std::unique_ptr<pros::adi::DigitalOut>, packets::MAX_ADI> m_adi; ///< ADI outputs by slot. null = not enabled
    uint16_t m_adiEnable = 0; ///< ConfigFixed::adiOutEnable
    uint16_t m_adiActiveLow = 0; ///< ConfigFixed::adiActiveLow
    bool m_expanderMissing = false; ///< the configured ADI expander wasn't detected
    std::array<Snapshot, HISTORY_SIZE> m_history; ///< recent STATE snapshots, by stateSeq % HISTORY_SIZE
    Snapshot m_base; ///< what the deltas are measured from
    uint8_t m_configId = 0; ///< applied config. 0 = not configured
    uint8_t m_statePeriodMs = 10; ///< STATE interval
    uint16_t m_watchdogMs = 100; ///< watchdog timeout
    uint16_t m_stateSeq = 1; ///< stateSeq of the next STATE. never 0
    uint16_t m_lastCmdSeq = 0; ///< cmdSeq of the last COMMAND applied
    uint8_t m_lastRequestId = 0; ///< ID of the last request run
    packets::Error m_lastError = packets::Error::NONE; ///< latest error since the previous STATE
    bool m_haveCmd = false; ///< a valid COMMAND arrived since the config was applied
    bool m_watchdogTripped = false; ///< no valid COMMAND for watchdogMs
    bool m_watchdogBraked = false; ///< the watchdog has braked the groups since it tripped
    bool m_booted = true; ///< no valid COMMAND since the program started. reported as status::BOOTED
    std::vector<uint8_t> m_bootConfig; ///< built-in CONFIG payload, applied by start()
    uint32_t m_cmdRxUs = 0; ///< when the last valid COMMAND arrived, pros::c::micros()
    uint32_t m_cmdRxMs = 0; ///< when the last valid COMMAND arrived, pros::millis()

    pros::Mutex m_logMutex; ///< guards the log queue
    std::array<LogEntry, LOG_QUEUE_SIZE> m_logQueue; ///< ring buffer of queued lines
    size_t m_logHead = 0; ///< index of the oldest queued line
    size_t m_logCount = 0; ///< lines queued
    #endif

    #ifdef __jetson_nano__
    using Clock = std::chrono::steady_clock;

    static constexpr size_t POS_HISTORY_SIZE = 16; ///< received positions kept, by stateSeq
    static constexpr size_t CMD_TIMES_SIZE = 64; ///< COMMAND send times kept, by cmdSeq
    static constexpr int CONFIG_RETRY_TICKS = 20; ///< ticks between CONFIG resends
    static constexpr int LINK_DOWN_PERIODS = 5; ///< state periods without STATE before the link is down
    static constexpr int64_t SYNC_WINDOW_US = 1000000; ///< how long time-sync samples are kept

    /// @brief rebuilt positions at one received STATE
    struct PosEntry {
        uint16_t seq = 0; ///< stateSeq. 0 = empty slot
        std::vector<double> pos; ///< groups, then rotations, radians
    };

    /// @brief one NTP-style time-sync measurement
    struct SyncSample {
        int64_t t3; ///< when the STATE arrived, Nano µs
        int64_t rtt; ///< round trip minus the brain's hold time, µs
        int64_t offset; ///< brain clock − Nano clock, µs
    };

    /// @brief reader thread: takes STATE and LOG, wakes the writer on each STATE
    void readerLoop();

    /// @brief writer thread: after each STATE, waits GUARD_US and sends CONFIG or the latest COMMAND
    void writerLoop();

    /**
     * @brief decodes a STATE: session tracking, request echo, time sync, positions, then publishes a Sample
     *
     * @param frame the STATE frame
     * @param end when the reader got the end of the frame
     */
    void handleState(const SerialProtocol::Frame& frame, Clock::time_point end);

    /**
     * @brief prints a LOG line to stderr
     *
     * @param frame the LOG frame
     */
    void handleLog(const SerialProtocol::Frame& frame);

    /**
     * @brief adds a time-sync measurement if this STATE closes a new round trip
     *
     * @param fixed STATE fixed part
     * @param stateBytes STATE payload size in bytes
     * @param t3 when the STATE arrived, Nano µs
     * @return int64_t when the brain sampled, on the Nano clock, µs
     */
    int64_t syncTime(const packets::StateFixed& fixed, uint16_t stateBytes, int64_t t3);

    /// @brief forgets everything tied to the brain's previous program run
    void resetBrainSession();

    /// @brief restarts positions from the config origin
    void resetPositions();

    SerialProtocol m_serial; ///< framing and the UART
    std::string m_device; ///< UART device path
    int32_t m_baud; ///< link baud rate
    SerialProtocol::Transport m_transport; ///< what the link runs over
    int m_deGpio; ///< DE GPIO, -1 = none
    std::vector<uint8_t> m_config; ///< CONFIG payload
    uint8_t m_myConfigId = 0; ///< configId inside `m_config`
    uint8_t m_nGroups = 0; ///< group count inside `m_config`
    uint8_t m_statePeriodMs = 10; ///< state period inside `m_config`

    std::atomic<bool> m_running{false}; ///< cleared to stop both threads
    std::thread m_reader; ///< runs readerLoop()
    std::thread m_writer; ///< runs writerLoop()

    std::mutex m_replyMutex; ///< guards the reply handoff
    std::condition_variable m_replyCv; ///< wakes the writer on each STATE
    Clock::time_point m_stateEnd; ///< end of the latest STATE
    bool m_statePending = false; ///< a STATE arrived that the writer hasn't replied to

    std::mutex m_commandMutex; ///< guards the command mailbox
    packets::CommandFixed m_commandFixed{}; ///< latest COMMAND fixed part, minus the sequence fields
    std::vector<packets::GroupCmd> m_groupCmds; ///< latest command per group
    uint8_t m_nextRequestId = 0; ///< last requestId handed out
    uint16_t m_cmdSeq = 0; ///< cmdSeq of the last COMMAND sent. writer only

    std::atomic<uint8_t> m_brainConfigId{0}; ///< StateFixed::configId of the latest STATE
    bool m_brainBooted = false; ///< status::BOOTED of the latest STATE. reader thread only
    std::atomic<uint8_t> m_brainRequestId{0}; ///< StateFixed::lastRequestId of the latest STATE
    std::atomic<uint16_t> m_ackStateSeq{0}; ///< newest stateSeq recorded, sent as COMMAND::ackStateSeq
    std::atomic<int64_t> m_lastStateUs{0}; ///< when the latest STATE arrived, Nano µs. 0 = never
    std::array<std::atomic<int64_t>, CMD_TIMES_SIZE> m_cmdSentUs{}; ///< COMMAND send times by cmdSeq % CMD_TIMES_SIZE

    // reader thread only
    std::array<PosEntry, POS_HISTORY_SIZE> m_posHistory; ///< rebuilt positions by stateSeq % POS_HISTORY_SIZE
    std::vector<double> m_lastPos; ///< positions at the previous Sample. empty = none yet
    std::deque<SyncSample> m_syncWindow; ///< time-sync samples from the last SYNC_WINDOW_US
    int64_t m_bestOffset = 0; ///< offset of the best recent sample
    bool m_haveOffset = false; ///< m_bestOffset is valid
    uint16_t m_lastSyncedCmd = 0; ///< cmdSeq of the last COMMAND used for time sync
    uint32_t m_lastBrainTUs = 0; ///< previous StateFixed::tUs, for unwrapping
    int64_t m_brainTimeHigh = 0; ///< wraps of StateFixed::tUs so far, × 2^32
    bool m_haveBrainTime = false; ///< m_lastBrainTUs is valid

    mutable std::mutex m_sampleMutex; ///< guards m_latest and m_callbacks
    std::optional<Sample> m_latest; ///< most recent Sample
    std::vector<std::function<void(const Sample&)>> m_callbacks; ///< onSample() callbacks
    #endif
};