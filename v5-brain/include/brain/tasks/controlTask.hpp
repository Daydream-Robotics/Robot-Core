/** @file controlTask.hpp
 *  @brief Periodic task that selects commands and applies motor output.
 */

#pragma once

#include "brain/control/controlPolicy.hpp"
#include "brain/control/motorOutput.hpp"
#include "brain/mailboxes.hpp"

#include "pros/rtos.hpp"

#include <memory>

namespace brain {

/** @brief Coordinates control-mode selection and motor output in its own task. */
class ControlTask {
public:
    /** @brief Bind command selection, output, and task mailboxes.
     *  @param coordinator Selects manual or Nano command behavior.
     *  @param output Applies selected commands to the motor groups.
     *  @param states Source mailbox for current BrainState values.
     *  @param commands Source mailbox for NanoCommand values.
     *  @note Referenced objects must outlive this task.
     */
    ControlTask(ControlModeCoordinator& coordinator,
                MotorOutput& output,
                BrainStateMailbox& states,
                NanoCommandMailbox& commands);

    /** @brief Start the control loop. */
    void start();
    /** @brief Stop the control loop and wait for its task to exit. */
    void stop();
    /** @brief Change the mode used by command selection.
     *  @param mode New control mode.
     */
    void setMode(RobotMode mode);

private:
    /** @brief Read current state, select a command, and apply it repeatedly. */
    void run();

    /// Non-owning command-selection policy.
    ControlModeCoordinator& coordinator_;
    /// Non-owning motor output adapter.
    MotorOutput& output_;
    /// Source of current controller and sensor state.
    BrainStateMailbox& states_;
    /// Source of the latest command received from the Nano.
    NanoCommandMailbox& commands_;
    /// PROS task handle; empty when the control loop is stopped.
    std::unique_ptr<pros::Task> task_;
};

} // namespace brain
