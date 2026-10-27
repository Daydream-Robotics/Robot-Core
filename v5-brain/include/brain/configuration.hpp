/** @file configuration.hpp
 *  @brief Brain-side configuration shared with the Nano communication link.
 */

#pragma once

#include "robotConfig.hpp"
#include "serial/packets.hpp"

#include <array>

namespace brain {

/** @brief Physical robot and serial-protocol configuration sent to the Nano. */
struct BrainConfiguration {
    /// Robot dimensions and wheel geometry used by shared calculations.
    shared::RobotConfig robot{};
    /// Fixed fields of the Nano CONFIG packet.
    packets::ConfigFixed fixed{};
    /// Per-slot motor-group configuration sent with the fixed CONFIG fields.
    std::array<packets::GroupCfg, packets::MAX_GROUPS> motorGroups{};
    /// Per-slot rotation-sensor configuration sent with the fixed CONFIG fields.
    std::array<packets::RotationCfg, packets::MAX_ROTATIONS> trackingWheels{};
};

} // namespace brain
