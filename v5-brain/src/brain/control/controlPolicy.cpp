/** @file controlPolicy.cpp
 *  @brief Arcade manual drive and per-mode selection of the motor command source.
 */

#include "brain/control/controlPolicy.hpp"

#include <algorithm>   
#include <cmath>       
#include <cstddef>     
#include <cstdlib>     

namespace brain {

namespace {

/// motor-group slot of each drive side
constexpr std::size_t LEFT_DRIVE_GROUP = 0;
constexpr std::size_t RIGHT_DRIVE_GROUP = 1;
/// number of groups manual control drives
constexpr std::uint8_t DRIVE_GROUP_COUNT = 2;

/// controller axis indices in DriverInput::axes
constexpr std::size_t AXIS_LEFT_Y = 1;
constexpr std::size_t AXIS_RIGHT_X = 2;

/// stick readings at or below this count as centered
constexpr int STICK_DEADBAND = 5;
/// full stick deflection
constexpr double STICK_MAX = 127.0;
/// motor voltage at full deflection, in millivolts
constexpr double MAX_VOLTAGE_MV = 12000.0;

/// zeroes small stick readings so a resting stick doesn't creep
int applyDeadband(int value) {
    return std::abs(value) <= STICK_DEADBAND ? 0 : value;
}

} // namespace

MotorCommand ManualControl::command(const DriverInput& input) const {
    MotorCommand command{};   // starts with groupCount 0, every group Stop

    // no controller, so stop everything
    if (!input.connected) {
        return command;
    }

    // left stick forward/back, right stick turns
    double forward = applyDeadband(input.axes[AXIS_LEFT_Y]);
    double turn = applyDeadband(input.axes[AXIS_RIGHT_X]);

    double left = forward + turn;
    double right = forward - turn;

    // if either side is over full power, scale both down together so the turn ratio stays the same
    double largest = std::max({std::abs(left), std::abs(right), STICK_MAX});
    left = left / largest * MAX_VOLTAGE_MV;
    right = right / largest * MAX_VOLTAGE_MV;

    command.groupCount = DRIVE_GROUP_COUNT;
    command.groups[LEFT_DRIVE_GROUP] = {MotorCommandMode::Voltage, static_cast<float>(left), 0};
    command.groups[RIGHT_DRIVE_GROUP] = {MotorCommandMode::Voltage, static_cast<float>(right), 0};
    return command;
}

MotorCommand ControlModeCoordinator::select(RobotMode mode,
                                            const DriverInput& input,
                                            const NanoCommand* nanoCommand,
                                            bool linkHealthy) const {
    switch (mode) {
        case RobotMode::Manual:
            // the driver is in charge
            return ManualControl{}.command(input);

        case RobotMode::Autonomous:
            // only trust the Nano when there is a command, it passed validation, and the link is up
            if (nanoCommand != nullptr && nanoCommand->valid && linkHealthy) {
                return nanoCommand->motors;
            }
            // otherwise stop rather than drive on a missing or stale command
            return MotorCommand{};

        case RobotMode::Disabled:
        default:
            // no groups commanded, so MotorOutput brakes every group
            return MotorCommand{};
    }
}

} // namespace brain
