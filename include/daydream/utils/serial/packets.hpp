/**
 * @file packets.hpp
 * @brief Wire format for the V5 Brain ⇄ Jetson Nano serial protocol.
 *
 * Both the brain and the Nano compile it, so it must 
 * stay byte-for-byte identical on both sides.
 * It has no I/O and no platform #ifdefs.
 *
 * Every multi-byte field is little-endian, and every struct is packed with no
 * padding. Fields are often unaligned, so read and write them with memcpy,
 * never reinterpret_cast.
 */

#pragma once
#include <cstdint>
#include <cstddef>
#include <cstring>

namespace packets {
    constexpr uint8_t PROTO_VERSION = 1; ///< sent in CONFIG. the brain rejects any other value (Error::CFG_VERSION)
    constexpr uint16_t MAX_PAYLOAD = 1024; ///< largest legal frame `size`, in bytes
    constexpr uint8_t MAX_GROUPS = 21; ///< max motor groups in one CONFIG
    constexpr uint8_t MAX_MOTORS = 8; ///< max motors per group (length of GroupCfg::ports)
    constexpr uint8_t MAX_ROTATIONS = 21; ///< max tracking-wheel rotation sensors
    constexpr uint8_t MAX_ADI = 16; ///< ADI output slots: 0–7 brain A–H, 8–15 expander A–H
    constexpr uint8_t MAX_LOG_TEXT = 62; ///< max LOG text bytes (payload = 1 level byte + text)
    constexpr uint32_t GUARD_US = 200; ///< minimum wait after STATE ends before the nano may transmit
    constexpr uint32_t REPLY_START_MAX_US = 2000; ///< latest the nano may start its reply. if it's later, it skips the tick
    constexpr uint32_t NO_COMMAND = 0xFFFFFFFF; ///< StateFixed::cmd_age_us value meaning "no COMMAND since config"


    /**
    * @brief Value of the frame header's `type` field.
    *
    * Unscoped so it compares directly against the raw uint16_t
    * read from the header. A receiver drops any type meant for the other side.
    */
    enum PacketType : uint16_t {
        CONFIG = 0, ///< nano → brain. sent instead of COMMAND until the config is applied
        STATE = 1, ///< brain → nano. sent every tick. once it finishes sending, the nano may reply
        COMMAND = 2, ///< nano → brain. the reply to every STATE
        LOG = 3 ///< brain → nano. logging text only, at most one per tick, before STATE
    };

    /// @brief GroupCmd::mode. The meaning of GroupCmd::value and ::aux depends on the mode.
    /// An unknown mode is treated as STOP.
    enum class GroupMode : uint8_t {
        STOP = 0, ///< stop using the group's brake_mode. value and aux are ignored
        VOLTAGE = 1, ///< value = mV, −12000..12000
        VELOCITY = 2, ///< value = rpm at the output shaft (motor's internal PID)
        POSITION = 3 ///< value = absolute degrees, aux = max rpm
    };

    /// @brief GroupCfg::gearset, the motor cartridge.
    enum class Gearset : uint8_t { 
        RED = 0, ///< 100 rpm
        GREEN = 1, ///< 200 rpm
        BLUE = 2 ///< 600 rpm
    };

    /// @brief GroupCfg::brake_mode. Used by GroupMode::STOP and when the watchdog trips.
    enum class BrakeMode : uint8_t { 
        COAST = 0, 
        BRAKE = 1, 
        HOLD = 2 
    };

    /// @brief CommandFixed::request_type for one-shot requests.
    enum class Request : uint8_t { 
        NONE = 0, 
        GROUP_TARE = 1, ///< target = group slot. zeroes each motor's absolute position for POSITION mode
        CONTROLLER_RUMBLE = 2 ///< value = pattern index from the shared rumble table
    };

    /// @brief StateFixed::last_error. Holds the most recent error.
    enum class Error : uint8_t {
        NONE = 0, 
        NOT_CONFIGURED = 1, ///< COMMAND arrived before any CONFIG
        SIZE = 2, ///< payload size didn't match what its counts imply
        CONFIG_ID = 3, ///< COMMAND config_id didn't match the applied config
        GROUP_COUNT = 4, ///< COMMAND n_groups didn't match the applied config
        RANGE = 5, ///< a field was out of range
        UNKNOWN_TYPE = 6, 
        OUTSIDE_WINDOW = 7, ///< frame arrived outside the nano's reply window
        CFG_PORT = 8, ///< port out of range, or an expander slot enabled with no expander
        CFG_DUPLICATE = 9, ///< smart port used twice, or the RS-485 link port was used
        CFG_LIMIT = 10, ///< more than 21 motors in total, or another limit exceeded
        CFG_VERSION = 11, ///< unsupported proto_version
        REQ_TARGET = 12, ///< request had a bad target slot
        REQ_TYPE = 13, ///< request had an unknown type
        REQ_DEVICE = 14 ///< the device failed while running the request
    };

    /// @brief Bits of StateFixed::status.
    namespace status {
        constexpr uint8_t DISABLED = 1 << 0; ///< robot disabled
        constexpr uint8_t AUTON = 1 << 1; ///< autonomous period
        constexpr uint8_t COMP = 1 << 2; ///< competition switch or field connected
        constexpr uint8_t FIELD = 1 << 3; ///< field control connected
        constexpr uint8_t WATCHDOG = 1 << 4; ///< watchdog tripped. cleared by the next valid COMMAND
        constexpr uint8_t CONTROLLER = 1 << 5; ///< master controller connected
        constexpr uint8_t NO_EXPANDER = 1 << 6; ///< ADI expander configured but not detected
    }

    /// @brief Bits of GroupState::flags and RotationState::flags.
    /// For a group, a fault bit is set if any motor in it reports that fault.
    namespace fault {   
        constexpr uint8_t OVER_TEMP = 1 << 0;
        constexpr uint8_t DRIVER = 1 << 1;
        constexpr uint8_t OVER_CURRENT = 1 << 2;
        constexpr uint8_t DRV_OVER_CURRENT = 1 << 3;
        constexpr uint8_t MISSING = 1 << 7; ///< device disconnected or the read failed. left out of group averages
                                        ///< a rotation sensor keeps its last good reading
    }

    // Wire structs: packed, sizes checked by the static_asserts below.
    #pragma pack(push, 1)

    /// @brief CONFIG fixed part (14 bytes). Followed by GroupCfg[n_groups] and RotationCfg[n_rotations].
    struct ConfigFixed {
        uint8_t config_id; ///< 1–255. must change whenever the content changes. resending the same ID is ignored
        uint8_t state_period_ms; ///< STATE interval, 5–50. default 10
        uint8_t proto_version; ///< must equal PROTO_VERSION
        uint16_t watchdog_ms; ///< stop all groups after this long with no valid COMMAND. 20–1000, default 100
        uint8_t n_groups; ///< 1–MAX_GROUPS
        uint8_t n_rotations; ///< 0–MAX_ROTATIONS
        uint8_t adi_expander_port; ///< smart port of the 3-wire expander. 0 = none
        uint16_t adi_out_enable; ///< bit k set = ADI slot k is a digital output
        uint16_t adi_initial; ///< bit k = state of slot k when the config is applied
        uint16_t adi_active_low; ///< bit k set = slot k inverts the logic level
    };

    /// @brief One motor group in CONFIG (11 bytes). Its position in the array is its slot number.
    struct GroupCfg {
        uint8_t n_motors; ///< 1–MAX_MOTORS
        uint8_t gearset; ///< Gearset
        uint8_t brake_mode; ///< BrakeMode
        int8_t ports[MAX_MOTORS]; ///< smart ports 1–21. negative = reversed. unused entries must be 0
    };

    /// @brief One tracking wheel in CONFIG (2 bytes).
    struct RotationCfg {
        uint8_t port; ///< smart port 1–21
        uint8_t flags; ///< bit0 = reversed
    };

    /// @brief STATE fixed part (30 bytes). Followed by GroupState[n_groups] and RotationState[n_rotations].
    struct StateFixed {
        uint32_t t_us; ///< brain µs clock, taken after device reads, just before sending. wraps every ~71.6 min
        uint32_t cmd_age_us; ///< µs from the end of the last valid COMMAND to t_us. NO_COMMAND = none since config
        uint16_t state_seq; ///< +1 per STATE. runs 1–65535, then wraps to 1 (0 is never used)
        uint16_t delta_base_seq; ///< the STATE the deltas are measured from. 0 = when the config was applied
        uint16_t last_cmd_seq; ///< cmd_seq of the last COMMAND applied
        uint8_t config_id; ///< applied config. 0 = not configured, or the brain program restarted
        uint8_t last_request_id; ///< ID of the last one-shot request executed
        uint8_t last_error; ///< Error
        uint8_t status; ///< status:: bits
        uint16_t battery_mV; ///< battery voltage
        int8_t ctrl_axes[4]; ///< master controller LX, LY, RX, RY, −127..127
        uint16_t ctrl_buttons; ///< bits 0–11: L1 L2 R1 R2 up down left right X B Y A
        uint16_t rx_errors; ///< running count of frames the brain dropped. wraps
        uint8_t n_groups;
        uint8_t n_rotations;
    };

    /// @brief One group in STATE (9 bytes). Values are averaged over the group's working motors
    struct GroupState {
        int32_t ime_delta_cdeg; ///< IME change since delta_base_seq, in centidegrees at the output shaft. reversal already applied
        float omega_rad_s; ///< rad/s at the output shaft. multiply by the external gear ratio to get wheel ω
        uint8_t flags; ///< fault:: bits
    };
    /// @brief One tracking wheel in STATE (5 bytes).
    struct RotationState {
        int32_t delta_cdeg; ///< change since delta_base_seq, in centidegrees
        uint8_t flags; ///< fault::MISSING only
    };

    /// @brief COMMAND fixed part (17 bytes). Followed by GroupCmd[n_groups].
    /// The newest COMMAND is taken. Each valid one resets the watchdog.
    struct CommandFixed {
        uint16_t cmd_seq; ///< +1 per COMMAND. wraps
        uint16_t ack_state_seq; ///< newest state_seq the nano has received and recorded. moves the brain's delta base
        uint8_t config_id; ///< must equal StateFixed::config_id, or the frame is ignored
        uint8_t n_groups; ///< must equal the configured group count
        uint8_t request_id; ///< one-shot request, 1–255. 0 = none. repeat it until it's echoed in last_request_id
        uint8_t request_type; ///< Request
        uint8_t request_target; ///< slot index
        float request_value; ///< request argument (see Request)
        uint16_t adi_out_mask; ///< bit k set = apply bit k of adi_out_state. clear = leave slot k unchanged
        uint16_t adi_out_state; ///< bit k = logical state of ADI slot k
    };

    /// @brief One group's command (7 bytes). Every motor in the group gets the same command
    struct GroupCmd {
        uint8_t mode; ///< GroupMode. sets the units of value and aux
        int16_t aux; ///< max rpm in POSITION mode. unused otherwise
        float value; ///< see GroupMode
    };
    #pragma pack(pop)
}