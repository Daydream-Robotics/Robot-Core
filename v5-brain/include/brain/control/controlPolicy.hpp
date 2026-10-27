/** @file controlPolicy.hpp
 *  @brief Interfaces for converting operator or Nano input into motor commands.
 */

#pragma once

#include "brain/types.hpp"

namespace brain {

/** @brief Maps a driver-input snapshot to the manual motor command. */
class ManualControl {
public:
    /** @brief Produce the motor command requested by the current driver input.
     *  @param input Controller snapshot to interpret.
     *  @return Motor-group command for manual operation.
     */
    MotorCommand command(const DriverInput& input) const;
};

/** @brief Selects the command source appropriate for the robot's current mode. */
class ControlModeCoordinator {
public:
    /** @brief Select a motor command using mode, operator input, and Nano health.
     *  @param mode Current robot mode.
     *  @param input Latest driver input, used when the selected mode requires it.
     *  @param nanoCommand Optional latest command from the Nano; may be null when absent.
     *  @param linkHealthy Whether the Nano link is considered usable by the caller.
     *  @return Motor command selected for the current mode and available inputs.
     */
    MotorCommand select(RobotMode mode,
                        const DriverInput& input,
                        const NanoCommand* nanoCommand,
                        bool linkHealthy) const;
};

} // namespace brain
