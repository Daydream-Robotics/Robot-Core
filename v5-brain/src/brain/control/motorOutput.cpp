/** @file motorOutput.cpp
 *  @brief Applies MotorCommand values to the configured PROS motor groups.
 */

#include "brain/control/motorOutput.hpp"

#include <cmath>   

namespace brain {

MotorOutput::MotorOutput(pros::MotorGroup* const* groups, std::size_t groupCount)
    : groups_(groups), groupCount_(groupCount) {}

void MotorOutput::apply(const MotorCommand& command) {
    
    for(std::size_t i = 0; i < groupCount_; ++i) {
        pros::MotorGroup* group = groups_[i];

        if (group == nullptr) {
            continue; // skip null pointers
        }

        if (i >= command.groupCount) {
            group->brake(); // stop if no command for this group
            continue;
        }
            
        const MotorGroupCommand& groupCommand = command.groups[i];
        // never send a NaN or infinite set point to a motor
        if (!std::isfinite(groupCommand.value)) {
            group->brake();
            continue;
        }

        switch (groupCommand.mode) {
            case MotorCommandMode::Voltage:
                group->move_voltage(static_cast<std::int32_t>(std::lround(groupCommand.value)));
                break;
            case MotorCommandMode::Velocity:
                group->move_velocity(static_cast<std::int32_t>(std::lround(groupCommand.value))); 
                break;
            case MotorCommandMode::Position:
                group->move_absolute(groupCommand.value, groupCommand.auxiliary);
                break;
            case MotorCommandMode::Stop:
            default:
                group->brake();
                break;
        }
    }
}

void MotorOutput::stop() {
    for(std::size_t i = 0; i < groupCount_; ++i) {
        pros::MotorGroup* group = groups_[i];
        if (group == nullptr) {
            continue; // skip null pointers
        }
        group->brake();
    }
}

} // namespace brain