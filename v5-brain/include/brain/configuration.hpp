/** @file configuration.hpp
 *  @brief Brain-side configuration shared with the Nano communication link.
 */

#pragma once

#include "robotConfig.hpp"
#include "protocol/packets.hpp"

#include <array>

namespace brain {

/** @brief Physical robot geometry and serial configuration applied on the Brain. */
struct BrainConfiguration {
    /// Robot dimensions and wheel geometry used by shared calculations.
    shared::RobotConfig robot{};
    /// Fixed fields of the boot configuration or CONFIG received from the Nano.
    packets::ConfigFixed fixed{};
    /// Per-slot motor-group configuration accompanying the fixed CONFIG fields.
    std::array<packets::GroupCfg, packets::MAX_GROUPS> motorGroups{};
    /// Per-slot rotation-sensor configuration accompanying the fixed CONFIG fields.
    std::array<packets::RotationCfg, packets::MAX_ROTATIONS> trackingWheels{};
};

} // namespace brain
