/** @file mailboxes.hpp
 *  @brief Thread-safe latest-value handoff points between Brain tasks.
 */

#pragma once

#include "brain/types.hpp"

#include "pros/rtos.hpp"

namespace brain {

/** @brief Shares the most recently published BrainState between tasks. */
class BrainStateMailbox {
public:
    /** @brief Publish a new state snapshot for readers.
     *  @param state State value to copy into the mailbox.
     */
    void publish(const BrainState& state);
    /** @brief Read the most recently published state.
     *  @param state Receives the copied state when one has been published.
     *  @return True if a state value is available; false before the first publish.
     */
    bool read(BrainState& state) const;

private:
    /// Protects the state and availability flag during cross-task access.
    mutable pros::Mutex mutex_;
    /// Latest value supplied to @ref publish.
    BrainState state_{};
    /// True after the first successful publication.
    bool hasValue_ = false;
};

/** @brief Shares the most recently received NanoCommand between tasks. */
class NanoCommandMailbox {
public:
    /** @brief Publish a new Nano command for readers.
     *  @param command Command value to copy into the mailbox.
     */
    void publish(const NanoCommand& command);
    /** @brief Read the most recently published Nano command.
     *  @param command Receives the copied command when one has been published.
     *  @return True if a command value is available; false before the first publish.
     */
    bool read(NanoCommand& command) const;

private:
    /// Protects the command and availability flag during cross-task access.
    mutable pros::Mutex mutex_;
    /// Latest value supplied to @ref publish.
    NanoCommand command_{};
    /// True after the first successful publication.
    bool hasValue_ = false;
};

} // namespace brain
