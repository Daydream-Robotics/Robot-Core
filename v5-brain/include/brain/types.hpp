/** @file types.hpp
 *  @brief Value types exchanged between the V5 Brain subsystems.
 */

#pragma once

#include "protocol/packets.hpp"

#include <array>
#include <cstdint>

namespace brain {

/** @brief Operating mode requested for the robot. */
enum class RobotMode : std::uint8_t {
    /// Robot output is disabled and should be safe.
    Disabled,
    /// The Brain uses local driver input for control.
    Manual,
    /// The Nano supplies autonomous motor commands.
    Autonomous
};

/** @brief Snapshot of the master controller's current input. */
struct DriverInput {
    /// Controller axes in LX, LY, RX, RY order, each in the range -127 to 127.
    std::array<std::int8_t, 4> axes{};
    /// Controller button state encoded as the protocol button bit mask.
    std::uint16_t buttons = 0;
    /// Whether the master controller is currently connected.
    bool connected = false;
};

/** @brief Telemetry collected for one configured motor group. */
struct MotorGroupTelemetry {
    /// Encoder position change in centidegrees over the reported interval.
    std::int32_t positionDelta = 0;
    /// Continuous output-shaft position in radians since the current configuration was applied.
    double positionRad = 0.0;
    /// Output-shaft angular velocity in radians per second.
    float velocity = 0.0f;
    /// Fault bits defined by the shared serial protocol.
    std::uint8_t faultFlags = 0;
};

/** @brief Telemetry collected for one configured tracking-wheel sensor. */
struct TrackingWheelTelemetry {
    /// Rotation change in centidegrees over the reported interval.
    std::int32_t delta = 0;
    /// Continuous sensor position in radians since the current configuration was applied.
    double positionRad = 0.0;
    /// Fault bits defined by the shared serial protocol.
    std::uint8_t faultFlags = 0;
};

/** @brief Sensor readings collected during one acquisition cycle. */
struct SensorSnapshot {
    /// Brain clock timestamp, in microseconds, associated with this snapshot.
    std::uint32_t sampledAt = 0;
    /// Heading change in centidegrees, counter-clockwise positive.
    std::int32_t headingDelta = 0;
    /// Number of valid entries in @ref motorGroups.
    std::uint8_t motorGroupCount = 0;
    /// Number of valid entries in @ref trackingWheels.
    std::uint8_t trackingWheelCount = 0;
    /// Per-group sensor data, indexed by configured motor-group slot.
    std::array<MotorGroupTelemetry, packets::MAX_GROUPS> motorGroups{};
    /// Per-wheel sensor data, indexed by configured tracking-wheel slot.
    std::array<TrackingWheelTelemetry, packets::MAX_ROTATIONS> trackingWheels{};
};

/** @brief Combined Brain telemetry published to control and communication tasks. */
struct BrainState {
    /// Most recent physical sensor readings.
    SensorSnapshot sensors{};
    /// Most recent master-controller input.
    DriverInput driver{};
    /// V5 battery voltage in millivolts.
    std::uint16_t batteryVoltage = 0;
    /// Status bits using the shared protocol's @c packets::status definitions.
    std::uint8_t statusFlags = 0;
};

/** @brief Motor commands returned by the selected control source. */
struct MotorCommand {
    /// Number of valid entries in @ref groups.
    std::uint8_t groupCount = 0;
    /// Per-group commands, indexed by configured motor-group slot.
    std::array<packets::GroupCmd, packets::MAX_GROUPS> groups{};
};

/** @brief A motor command received from the Jetson Nano. */
struct NanoCommand {
    /// Shared command metadata, including sequence, acknowledgments and ADI requests.
    packets::CommandFixed fixed{};
    /// Group commands; fixed.nGroups is the sole count of valid entries.
    std::array<packets::GroupCmd, packets::MAX_GROUPS> groups{};
    /// Whether the command passed reception and validation.
    bool valid = false;
};

} // namespace brain
