/**
 * \file constants.h
 *
 * Contains constants used throughout the robot.
 */

#ifndef CONSTANTS_H
#define CONSTANTS_H

#include "config/linkConstants.hpp"

/* - - - - - - - - - - - - - - [PORTS] - - - - - - - - - - - - - - */


constexpr int INITIAL_INTAKE_PORT = 22;
constexpr int INITIAL_INTAKE_PORT2 = -22;
constexpr int LEVER_ONE_PORT = -22;
constexpr int LEVER_TWO_PORT = 22;

constexpr int IMU_PORT = 1; 


/* - - - - - - - - - - - - - - [ODOMETRY] - - - - - - - - - - - - - - */

// diameter of tracking wheels in inches
constexpr double PARALLEL_TRACKING_WHEEL_DIAMETER = 2.03895;
constexpr double PERPENDICULAR_TRACKING_WHEEL_DIAMETER = 2.0425;

// distance from tracking center to wheels
constexpr double PARALLEL_TRACKING_WHEEL_OFFSET = 0.0; // positive is to the right of tracking center
constexpr double PERPENDICULAR_TRACKING_WHEEL_OFFSET = 0.0; // positive is behind tracking center

/* - - - - - - - - - - - - - - [ROBOT DIMS] - - - - - - - - - - - - - - */

constexpr double DRIVE_WHEEL_DIAMETER_INCHES = 3.25;
constexpr double DRIVE_GEAR_RATIO = 450.0/600.0; //450 rpm wheel / 600 rpm cartridge
constexpr double TRACK_WIDTH = 10.5; // drive wheel track width (in), also sent to the Nano's MPC

/* - - - - - - - - - - - - - - [DRIVE] - - - - - - - - - - - - - - */

constexpr int DEADZONE = 2;

/* - - - - - - - - - - - - - - [GENERAL] - - - - - - - - - - - - - - */

constexpr int MAX_VOLTAGE = 127;
constexpr int HIGH_VOLTAGE = 100;
constexpr int MID_VOLTAGE = 60;
constexpr int LOW_VOLTAGE = 40;
constexpr int STOP = 0;
constexpr double MM_TO_IN = 1.0 / 25.4;

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


/* - - - - - - - - - - - - - - [Motion Controllers] - - - - - - - - - - - - - - */
constexpr double END_TOLERANCE = 0.5;

enum class DriveType {
    SPLIT_ARCADE,
    TANK
};

/* - - - - - - - - - - - - - - [LIDAR] - - - - - - - - - - - - - - */
// starting values from the LiDAR plan's parameter table; tune with the simulator and robot logs

constexpr double LIDAR_TAU_0 = 6.0;       // starting ICP max match distance before any data (in)
constexpr double LIDAR_DELTA_MIN = 0.5;   // ICP corrections smaller than this are ignored by the adaptive threshold (in)
constexpr double LIDAR_R_MAX = 204.0;     // max range considered, ~ diagonal of a 12 ft field (in)


#endif
