/**
 * \file constants.h
 *
 * Contains constants used throughout the robot.
 */

#ifndef CONSTANTS_H
#define CONSTANTS_H

/* - - - - - - - - - - - - - - [PORTS] - - - - - - - - - - - - - - */

constexpr int LEFT_TRACKING_WHEEL_PORT = -1;
constexpr int RIGHT_TRACKING_WHEEL_PORT = 5;
constexpr int BACK_TRACKING_WHEEL_PORT = 13;

#define LEFT_DRIVE_WHEEL_PORTS {LEFT_TRACKING_WHEEL_PORT, 2, -3, -4}
#define RIGHT_DRIVE_WHEEL_PORTS {RIGHT_TRACKING_WHEEL_PORT, -17, 14, 12}

constexpr int IMU_PORT = 10;

/* - - - - - - - - - - - - - - [DRIVE] - - - - - - - - - - - - - - */

constexpr int DEADZONE = 3;

/* - - - - - - - - - - - - - - [PIDS] - - - - - - - - - - - - - - */

constexpr double TURN_KP = 1.0;
constexpr double TURN_KI = 0.0;
constexpr double TURN_KD = 0.0;

constexpr double DRIVE_KP = 0.0;
constexpr double DRIVE_KI = 0.0;
constexpr double DRIVE_KD = 0.0;


#endif