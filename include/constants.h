/**
 * \file constants.h
 *
 * Contains constants used throughout the robot.
 */

#ifndef CONSTANTS_H
#define CONSTANTS_H

/* - - - - - - - - - - - - - - [PORTS] - - - - - - - - - - - - - - */

constexpr int PARALLEL_TRACKING_WHEEL_PORT = 10;
constexpr int PERPENDICULAR_TRACKING_WHEEL_PORT = 6;

#define LEFT_DRIVE_WHEEL_PORTS {-13, -11, -1, 2}
#define RIGHT_DRIVE_WHEEL_PORTS {16, 18, 19, -20}

constexpr int FRONT_INTAKE_PORT = -5;
constexpr int MID_INTAKE_PORT = 4;
constexpr int BACK_INTAKE_PORT = -8;

constexpr char IMU_PORT = 9;

constexpr char UNLOADER_PORT = 'C'; // C
constexpr char CENTER_SCORE_PORT = 'A'; // A
constexpr char DESCORE_PORT = 'B'; // B

/* - - - - - - - - - - - - - - [DRIVE] - - - - - - - - - - - - - - */

constexpr int DEADZONE = 3;

/* - - - - - - - - - - - - - - [GENERAL] - - - - - - - - - - - - - - */

constexpr int MAX_VOLTAGE = 127;
constexpr int HIGH_VOLTAGE = 100;
constexpr int MID_VOLTAGE = 60;
constexpr int LOW_VOLTAGE = 40;
constexpr int STOP = 0;

/* - - - - - - - - - - - - - - [ODOMETRY] - - - - - - - - - - - - - - */

constexpr double PERPINDICULAR_TRACKING_WHEEL_DISTANCE = 2.588;

constexpr double PARALLEL_TRACKING_WHEEL_DIAMETER = 2.00;
constexpr double PERPENDICULAR_TRACKING_WHEEL_DIAMETER = 2.75;

/* - - - - - - - - - - - - - - [PIDS] - - - - - - - - - - - - - - */

constexpr double TURN_KP = 3.5;
constexpr double TURN_KI = 0.0000000;//0.00001;
constexpr double TURN_KD = 0.04; //0.16; // 0.04

constexpr double MOVE_KP = 1.5;
constexpr double MOVE_KI = 0.0;
constexpr double MOVE_KD = 0.0;

constexpr double MOVE_HEADING_KP = 0.1;
constexpr double MOVE_HEADING_KI = 0.001;
constexpr double MOVE_HEADING_KD = 0.001;
constexpr double MOVE_HEADING_INTEGRATOR_LIMIT = 0.5;

enum class DriveType {
    SPLIT_ARCADE,
    TANK
};

#endif