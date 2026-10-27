/** @file types.hpp
 *  @brief Value types exchanged between the V5 Brain subsystems.
 */

#pragma once

#include "serial/packets.hpp"

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
    /// Output-shaft angular velocity in radians per second.
    float velocity = 0.0f;
    /// Fault bits defined by the shared serial protocol.
    std::uint8_t faultFlags = 0;
};

/** @brief Telemetry collected for one configured tracking-wheel sensor. */
struct TrackingWheelTelemetry {
    /// Rotation change in centidegrees over the reported interval.
    std::int32_t delta = 0;
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

/** @brief Control representation used for one motor group. */
enum class MotorCommandMode : std::uint8_t {
    /// Stop the group using its configured brake behavior.
    Stop,
    /// Set motor voltage; see the serial protocol for the value range and units.
    Voltage,
    /// Set motor velocity; see the serial protocol for the value units.
    Velocity,
    /// Move to a position; see the serial protocol for value and auxiliary units.
    Position
};

/** @brief Requested control mode and values for one motor group. */
struct MotorGroupCommand {
    /// Interpretation of @ref value and @ref auxiliary.
    MotorCommandMode mode = MotorCommandMode::Stop;
    /// Mode-dependent set point for the motor group.
    float value = 0.0f;
    /// Additional mode-dependent argument, such as a position speed limit.
    std::int16_t auxiliary = 0;
};

/** @brief Motor commands returned by the selected control source. */
struct MotorCommand {
    /// Number of valid entries in @ref groups.
    std::uint8_t groupCount = 0;
    /// Per-group commands, indexed by configured motor-group slot.
    std::array<MotorGroupCommand, packets::MAX_GROUPS> groups{};
};

/** @brief A motor command received from the Jetson Nano. */
struct NanoCommand {
    /// Sequence number assigned by the Nano.
    std::uint16_t sequence = 0;
    /// Motor-group commands carried by this Nano command.
    MotorCommand motors{};
    /// Whether the command passed reception and validation.
    bool valid = false;
};

} // namespace brain
