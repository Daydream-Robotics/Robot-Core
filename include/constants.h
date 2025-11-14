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

constexpr int FRONT_INTAKE_PORT = -6;
constexpr int MAIN_INTAKE_PORT = 9;
constexpr int BACK_INTAKE_PORT = 8;

constexpr char IMU_PORT = 11;

constexpr char PNEUMATIC_PORT = 'A';

/* - - - - - - - - - - - - - - [DRIVE] - - - - - - - - - - - - - - */

constexpr int DEADZONE = 3;

/* - - - - - - - - - - - - - - [GENERAL] - - - - - - - - - - - - - - */

constexpr int MAX_VOLTAGE = 127;
constexpr int HIGH_VOLTAGE = 100;
constexpr int MID_VOLTAGE = 60;
constexpr int LOW_VOLTAGE = 40;
constexpr int STOP = 0;

#endif