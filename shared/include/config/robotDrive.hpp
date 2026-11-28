#pragma once

// which drivetrain this robot runs, shared by the brain and the Nano
// it sets the link config's drive groups (packets::buildLinkConfig), the MPC the Nano builds (MPC in mpc.hpp),
// and how the path follower reads and commands the drive. header-only and std only, so the brain's PROS build includes it too

#include <cstddef>

#include "config/constants.h"
#include "mpc/drivetrain.hpp"

//the robot's drivetrain: kinematics::DiffDrive, kinematics::XDrive or kinematics::HDrive
using RobotDrive = kinematics::DiffDrive;

/**
 * @brief external gear ratio of one of RobotDrive's motor groups
 *
 * @param group group index, in RobotDrive's group order (see mpc/drivetrain.hpp)
 * @return wheel rpm / motor cartridge rpm. multiply the brain's output shaft speed by it to get wheel speed
 */
constexpr double driveGearRatio(std::size_t group) {
    if constexpr (RobotDrive::MODE == kinematics::DriveMode::DIFF) {
        return DRIVE_GEAR_RATIO;
    }
    else if constexpr (RobotDrive::MODE == kinematics::DriveMode::X) {
        return XDRIVE_GEAR_RATIO;
    }
    else {
        //group order L, R, strafe
        return group == 2 ? HDRIVE_STRAFE_GEAR_RATIO : HDRIVE_GEAR_RATIO;
    }
}
