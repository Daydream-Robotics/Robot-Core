#pragma once

#include "brain/types.hpp"

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

} // namespace brain
