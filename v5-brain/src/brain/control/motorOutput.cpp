/** @file motorOutput.cpp
 *  @brief Applies MotorCommand values to the configured PROS motor groups.
 */

#include "brain/control/motorOutput.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace brain {

MotorOutput::MotorOutput(pros::MotorGroup* const* groups, std::size_t groupCount)
    : m_groups(groups), m_groupCount(groupCount) {}

void MotorOutput::apply(const MotorCommand& command) {
    if (m_groups == nullptr) return;
    for (std::size_t i = 0; i < m_groupCount; ++i) {
        pros::MotorGroup* group = m_groups[i];

        if (group == nullptr) {
            continue; // skip null pointers
        }

        if (i >= command.groupCount || i >= command.groups.size()) {
            group->brake(); // stop if no command for this group
            continue;
        }
            
        const packets::GroupCmd& GROUP_COMMAND = command.groups[i];
        // never send a NaN or infinite set point to a motor
        if (!std::isfinite(GROUP_COMMAND.value)) {
            group->brake();
            continue;
        }

        switch (static_cast<packets::GroupMode>(GROUP_COMMAND.mode)) {
            case packets::GroupMode::VOLTAGE:
                group->move_voltage(static_cast<std::int32_t>(
                    std::lround(std::clamp(GROUP_COMMAND.value, -12000.0f, 12000.0f))));
                break;
            case packets::GroupMode::VELOCITY:
                group->move_velocity(static_cast<std::int32_t>(std::lround(std::clamp(
                    static_cast<double>(GROUP_COMMAND.value),
                    static_cast<double>(std::numeric_limits<std::int32_t>::min()),
                    static_cast<double>(std::numeric_limits<std::int32_t>::max())))));
                break;
            case packets::GroupMode::POSITION:
                group->move_absolute(GROUP_COMMAND.value, GROUP_COMMAND.aux);
                break;
            case packets::GroupMode::STOP:
            default:
                group->brake();
                break;
        }
    }
}

void MotorOutput::stop() {
    if (m_groups == nullptr) return;
    for (std::size_t i = 0; i < m_groupCount; ++i) {
        pros::MotorGroup* group = m_groups[i];
        if (group == nullptr) {
            continue; // skip null pointers
        }
        group->brake();
    }
}

} // namespace brain
