/**
 * \file constants.h
 *
 * Contains constants used throughout the robot.
 */

#ifndef CONSTANTS_H
#define CONSTANTS_H

/* - - - - - - - - - - - - - - [PORTS] - - - - - - - - - - - - - - */

constexpr int LEFT_TRACKING_WHEEL_PORT = -18;
constexpr int RIGHT_TRACKING_WHEEL_PORT = 13;
constexpr int BACK_TRACKING_WHEEL_PORT = 15;

#define LEFT_DRIVE_WHEEL_PORTS {-1, 2, -3, -4}
#define RIGHT_DRIVE_WHEEL_PORTS {13, -17, 14, 12}

// ESTIMATED
constexpr int IMU_PORT = 11;

/* - - - - - - - - - - - - - - [DRIVE] - - - - - - - - - - - - - - */

constexpr int DEADZONE = 3;

/* - - - - - - - - - - - - - - [ODOMETRY] - - - - - - - - - - - - - - */

constexpr double LEFT_TRACKING_WHEEL_DISTANCE = 1.58;
constexpr double RIGHT_TRACKING_WHEEL_DISTANCE = 1.58;
constexpr double BACK_TRACKING_WHEEL_DISTANCE = 1.188;

constexpr double TRACKING_WHEEL_DIAMETER = 2.00;
constexpr double BACK_TRACKING_WHEEL_DIAMETER = 2.75;

/* - - - - - - - - - - - - - - [PIDS] - - - - - - - - - - - - - - */

constexpr double TURN_KP = 3.7;
constexpr double TURN_KI = 0.00003;
constexpr double TURN_KD = 0.14;

constexpr double MOVE_KP = 1.5;
constexpr double MOVE_KI = 0.0;
constexpr double MOVE_KD = 0.0;

#endif