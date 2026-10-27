#pragma once

#include "brain/types.hpp"

#include "pros/rtos.hpp"

namespace brain {

class BrainStateMailbox {
public:
    void publish(const BrainState& state);
    bool read(BrainState& state) const;

private:
    mutable pros::Mutex mutex_;
    BrainState state_{};
    bool hasValue_ = false;
};

class NanoCommandMailbox {
public:
    void publish(const NanoCommand& command);
    bool read(NanoCommand& command) const;

private:
    mutable pros::Mutex mutex_;
    NanoCommand command_{};
    bool hasValue_ = false;
};

} // namespace brain
