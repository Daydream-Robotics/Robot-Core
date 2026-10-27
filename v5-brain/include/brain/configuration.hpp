#pragma once

#include "robotConfig.hpp"
#include "serial/packets.hpp"

#include <array>

namespace brain {

struct BrainConfiguration {
    shared::RobotConfig robot{};
    packets::ConfigFixed fixed{};
    std::array<packets::GroupCfg, packets::MAX_GROUPS> motorGroups{};
    std::array<packets::RotationCfg, packets::MAX_ROTATIONS> trackingWheels{};
};

} // namespace brain
