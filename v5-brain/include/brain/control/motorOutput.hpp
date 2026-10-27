#pragma once

#include "brain/types.hpp"

#include "pros/motor_group.hpp"

#include <cstddef>

namespace brain {

class MotorOutput {
public:
    MotorOutput(pros::MotorGroup* const* groups, std::size_t groupCount);

    void apply(const MotorCommand& command);
    void stop();

private:
    pros::MotorGroup* const* groups_;
    std::size_t groupCount_;
};

} // namespace brain
