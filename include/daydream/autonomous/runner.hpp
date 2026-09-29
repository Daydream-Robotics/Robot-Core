/**
 * @file runner.hpp
 * @brief Project interface or implementation.
 */

#pragma once

#include "daydream/motion/odometry.hpp"
#include "daydream/subsystems/drivetrain.hpp"
#include "daydream/subsystems/intake.hpp"
#include "daydream/subsystems/pneumatics.hpp"

namespace daydream {
namespace autonomous {

/**
 * @brief Runs the autonomous routine using the application's shared subsystems.
 * @param drivetrain Shared drivetrain interface.
 * @param intake Shared intake interface.
 * @param pneumatics Shared pneumatic actuator interface.
 * @param odometry Shared pose-estimation service.
 */
void run(Drivetrain& drivetrain, Intake& intake, Pneumatics& pneumatics,
         Odometry& odometry);

}  // namespace autonomous
}  // namespace daydream
