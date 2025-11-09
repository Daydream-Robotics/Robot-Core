/**
 * \file constants.h
 *
 * Contains constants used throughout the robot.
 */

#ifndef CONSTANTS_H
#define CONSTANTS_H

/* - - - - - - - - - - - - - - [PORTS] - - - - - - - - - - - - - - */

constexpr int LEFT_TRACKING_WHEEL_PORT = -19;
constexpr int RIGHT_TRACKING_WHEEL_PORT = 13;
constexpr int BACK_TRACKING_WHEEL_PORT = 15;

#define LEFT_DRIVE_WHEEL_PORTS {-1, 2, -3, -4}
#define RIGHT_DRIVE_WHEEL_PORTS {13, -17, 14, 12}

/* - - - - - - - - - - - - - - [DRIVE] - - - - - - - - - - - - - - */

constexpr int DEADZONE = 3;

#endif