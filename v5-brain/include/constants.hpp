/** @file constants.hpp
 *  @brief V5 Brain hardware port assignments and Brain-specific constants.
 *
 *  Keep physical Brain wiring assignments here. Robot dimensions and other
 *  values used by both the Brain and Nano belong in shared configuration.
 */

#pragma once

#include <array>
#include <cstdint>

namespace brain::constants {

/// V5 smart port connected to the heading IMU.
inline constexpr std::int8_t IMU_PORT = 1;

/// V5 smart port connected to the parallel tracking-wheel rotation sensor.
inline constexpr std::int8_t PARALLEL_TRACKING_WHEEL_PORT = 6;

/// V5 smart port connected to the perpendicular tracking-wheel rotation sensor.
inline constexpr std::int8_t PERPENDICULAR_TRACKING_WHEEL_PORT = 10;

/// V5 smart ports for the left drivetrain motors; negative entries are reversed.
inline constexpr std::array<std::int8_t, 3> LEFT_DRIVE_MOTOR_PORTS{{-14, -12, -13}};

/// V5 smart ports for the right drivetrain motors.
inline constexpr std::array<std::int8_t, 3> RIGHT_DRIVE_MOTOR_PORTS{{17, 18, 19}};

/// Brain three-wire port used by the ball blocker pneumatic.
inline constexpr char BALL_BLOCKER_PORT = 'A';

/// Brain three-wire port used by the scoring lifter pneumatic.
inline constexpr char SCORING_LIFTER_PORT = 'B';

/// Brain three-wire port used by the matchloader pneumatic.
inline constexpr char MATCHLOADER_PORT = 'C';

/// Brain three-wire port used by the descorer pneumatic.
inline constexpr char DESCORER_PORT = 'D';

} // namespace brain::constants
