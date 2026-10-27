#pragma once

#include "brain/communication.hpp"
#include "brain/types.hpp"

#include "pros/motor_group.hpp"
#include "pros/rtos.hpp"

#include <cstddef>
#include <memory>

namespace brain {

class ManualControl {
public:
    MotorCommand command(const DriverInput& input) const;
};

class ControlModeCoordinator {
public:
    MotorCommand select(RobotMode mode,
                        const DriverInput& input,
                        const NanoCommand* nanoCommand,
                        bool linkHealthy) const;
};

class MotorOutput {
public:
    MotorOutput(pros::MotorGroup* const* groups, std::size_t groupCount);

    void apply(const MotorCommand& command);
    void stop();

private:
    pros::MotorGroup* const* groups_;
    std::size_t groupCount_;
};

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
