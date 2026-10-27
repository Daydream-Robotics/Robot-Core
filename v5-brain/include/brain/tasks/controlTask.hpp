#pragma once

#include "brain/control/controlPolicy.hpp"
#include "brain/control/motorOutput.hpp"
#include "brain/mailboxes.hpp"

#include "pros/rtos.hpp"

#include <memory>

namespace brain {

class ControlTask {
public:
    ControlTask(ControlModeCoordinator& coordinator,
                MotorOutput& output,
                BrainStateMailbox& states,
                NanoCommandMailbox& commands);

    void start();
    void stop();
    void setMode(RobotMode mode);

private:
    void run();

    ControlModeCoordinator& coordinator_;
    MotorOutput& output_;
    BrainStateMailbox& states_;
    NanoCommandMailbox& commands_;
    std::unique_ptr<pros::Task> task_;
};

} // namespace brain
