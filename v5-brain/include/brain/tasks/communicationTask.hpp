/** @file communicationTask.hpp
 *  @brief Periodic task that exchanges Brain state and Nano commands.
 */

#pragma once

#include "brain/communication/nanoLink.hpp"
#include "brain/mailboxes.hpp"

#include "pros/rtos.hpp"

#include <memory>

namespace brain {

/** @brief Runs the NanoLink exchange loop and distributes received commands. */
class CommunicationTask {
public:
    /** @brief Bind the link and mailboxes used by the communication loop.
     *  @param link Nano serial-link adapter.
     *  @param states Source mailbox for the latest BrainState.
     *  @param commands Destination mailbox for received NanoCommand values.
     *  @note Referenced objects must outlive this task.
     */
    CommunicationTask(NanoLink& link,
                      BrainStateMailbox& states,
                      NanoCommandMailbox& commands);

    /** @brief Start the periodic communication loop. */
    void start();
    /** @brief Stop communication and wait for its task to exit. */
    void stop();

private:
    /** @brief Perform one exchange repeatedly until the task is stopped. */
    void run();

    /// Non-owning adapter used to exchange state and commands with the Nano.
    NanoLink& link_;
    /// Source of the latest sensor and controller state.
    BrainStateMailbox& states_;
    /// Destination for commands received from the Nano.
    NanoCommandMailbox& commands_;
    /// PROS task handle; empty when the communication loop is stopped.
    std::unique_ptr<pros::Task> task_;
};

} // namespace brain
