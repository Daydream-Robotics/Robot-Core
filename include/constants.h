/**
 * \file constants.h
 *
 * Contains constants used throughout the robot.
 */

#ifndef CONSTANTS_H
#define CONSTANTS_H
 #include <stdio.h>
/* - - - - - - - - - - - - - - [PORTS] - - - - - - - - - - - - - - */

constexpr int PARALLEL_TRACKING_WHEEL_PORT = -6;
constexpr int PERPENDICULAR_TRACKING_WHEEL_PORT = 10;

#define LEFT_DRIVE_WHEEL_PORTS {-13, -11, -1, 2}
#define RIGHT_DRIVE_WHEEL_PORTS {-3, 18, 19, 20}

constexpr int FRONT_INTAKE_PORT = -14;
constexpr int MID_INTAKE_PORT = 4;
constexpr int BACK_INTAKE_PORT = -5;

constexpr char IMU_PORT = 12;

constexpr char UNLOADER_PORT = 'C'; 
constexpr char CENTER_SCORE_PORT = 'A'; 
constexpr char DESCORE_PORT = 'B'; 

/* - - - - - - - - - - - - - - [DRIVE] - - - - - - - - - - - - - - */

constexpr int DEADZONE = 3;

/* - - - - - - - - - - - - - - [GENERAL] - - - - - - - - - - - - - - */

constexpr int MAX_VOLTAGE = 127;
constexpr int HIGH_VOLTAGE = 100;
constexpr int MID_VOLTAGE = 60;
constexpr int LOW_VOLTAGE = 40;
constexpr int STOP = 0;

/* - - - - - - - - - - - - - - [ODOMETRY] - - - - - - - - - - - - - - */

constexpr double PERPINDICULAR_TRACKING_WHEEL_DISTANCE = 2.25;

constexpr double PARALLEL_TRACKING_WHEEL_DIAMETER = 2.03895;
constexpr double PERPENDICULAR_TRACKING_WHEEL_DIAMETER = 2.0425;

/* - - - - - - - - - - - - - - [PIDS] - - - - - - - - - - - - - - */

constexpr double DISTANCE_KP = 4.25;
constexpr double DISTANCE_KI = 0.0; 
constexpr double DISTANCE_KD = 0.0; 
constexpr double DISTANCE_KI_THRESHOLD = 1.0;

constexpr double HEADING_KP = 0.01; 
constexpr double HEADING_KI = 0.0;
constexpr double HEADING_KD = 0.0;  
constexpr double HEADING_KI_THRESHOLD = 0.0;

constexpr double TURN_KP = 1.2; 
constexpr double TURN_KI = 0.01; 
constexpr double TURN_KD = 0.001; 
constexpr double TURN_KI_THRESHOLD = 15.0;


enum class DriveType {
    SPLIT_ARCADE,
    TANK
};


#endif