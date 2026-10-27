#pragma once

#include "serial/packets.hpp"

#include <array>
#include <cstdint>

namespace brain {

enum class RobotMode : std::uint8_t {
    Disabled,
    Manual,
    Autonomous
};

struct DriverInput {
    std::array<std::int8_t, 4> axes{};
    std::uint16_t buttons = 0;
    bool connected = false;
};

struct MotorGroupTelemetry {
    std::int32_t positionDelta = 0;
    float velocity = 0.0f;
    std::uint8_t faultFlags = 0;
};

struct TrackingWheelTelemetry {
    std::int32_t delta = 0;
    std::uint8_t faultFlags = 0;
};

struct SensorSnapshot {
    std::uint32_t sampledAt = 0;
    std::int32_t headingDelta = 0;
    std::uint8_t motorGroupCount = 0;
    std::uint8_t trackingWheelCount = 0;
    std::array<MotorGroupTelemetry, packets::MAX_GROUPS> motorGroups{};
    std::array<TrackingWheelTelemetry, packets::MAX_ROTATIONS> trackingWheels{};
};

struct BrainState {
    SensorSnapshot sensors{};
    DriverInput driver{};
    std::uint16_t batteryVoltage = 0;
    std::uint8_t statusFlags = 0;
};

enum class MotorCommandMode : std::uint8_t {
    Stop,
    Voltage,
    Velocity,
    Position
};

struct MotorGroupCommand {
    MotorCommandMode mode = MotorCommandMode::Stop;
    float value = 0.0f;
    std::int16_t auxiliary = 0;
};

struct MotorCommand {
    std::uint8_t groupCount = 0;
    std::array<MotorGroupCommand, packets::MAX_GROUPS> groups{};
};

struct NanoCommand {
    std::uint16_t sequence = 0;
    MotorCommand motors{};
    bool valid = false;
};

} // namespace brain
