/** @file mailboxes.cpp
*  @brief Thread-safe latest-value mailboxes shared between Brain tasks.
 */

//brainState holds what the robot senses
//nanoCommand is what the nano wants the motors to do 

#include "brain/mailboxes.hpp"

#include <mutex>   // for std::lock_guard

namespace brain {

void BrainStateMailbox::publish(const BrainState& state) {
    // lock so no one reads while we write. unlocks at the closing 
    std::lock_guard<pros::Mutex> lock(mutex_);
    // store the newest state
    state_ = state;
    // mark that there's a value to read now
    hasValue_ = true;
}

bool BrainStateMailbox::read(BrainState& state) const {
    // lock so no one writes while we read
    std::lock_guard<pros::Mutex> lock(mutex_);
    // nothing published yet, leave the caller's state alone
    if (!hasValue_) {
        return false;
    }
    // copy the newest state out to the caller
    state = state_;
    return true;
}

void NanoCommandMailbox::publish(const NanoCommand& command) {
    // lock so no one reads while we write
    std::lock_guard<pros::Mutex> lock(mutex_);
    // store the newest command
    command_ = command;
    // mark that there's a value to read now
    hasValue_ = true;
}

bool NanoCommandMailbox::read(NanoCommand& command) const {
    // lock so no one writes while we read
    std::lock_guard<pros::Mutex> lock(mutex_);
    // nothing published yet, leave the caller's command alone
    if (!hasValue_) {
        return false;
    }
    // copy the newest command out to the caller
    command = command_;
    return true;
}

} // namespace brain
