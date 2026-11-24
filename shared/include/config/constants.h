/**
 * \file constants.h
 *
 * Contains constants used throughout the robot.
 */

#ifndef CONSTANTS_H
#define CONSTANTS_H

#include <cstdint>

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

// robot footprint for the self-filter, from the EKF origin (in), TODO (#139/#154): measure on the robot
constexpr double LIDAR_FOOTPRINT_FRONT_IN = 12.0;
constexpr double LIDAR_FOOTPRINT_REAR_IN = 12.0;
constexpr double LIDAR_FOOTPRINT_LEFT_IN = 9.0;
constexpr double LIDAR_FOOTPRINT_RIGHT_IN = 9.0;
constexpr double LIDAR_FOOTPRINT_MARGIN_IN = 1.0; // extra clearance around the footprint (in)

// C1 usable range (in), TODO: measure on hardware
constexpr double LIDAR_R_MIN_IN = 2.0;

// sensor 0 mount, robot frame (+X forward, +Y left, CCW+), TODO: measure, placeholder values
constexpr double LIDAR_0_X_IN = 6.0;
constexpr double LIDAR_0_Y_IN = 4.0;
constexpr double LIDAR_0_PSI_RAD = 0.0; // mount yaw plus the C1's 0 deg offset, check with a target on the left
constexpr double LIDAR_0_HEIGHT_IN = 8.0; // scan plane height, below the field wall top (#144)

// sensor 1 mount, robot frame (+X forward, +Y left, CCW+), TODO: measure, placeholder values
constexpr double LIDAR_1_X_IN = 6.0;
constexpr double LIDAR_1_Y_IN = -4.0;
constexpr double LIDAR_1_PSI_RAD = 0.0; // mount yaw plus the C1's 0 deg offset, check with a target on the left
constexpr double LIDAR_1_HEIGHT_IN = 8.0; // scan plane height, below the field wall top (#144)

/* - - - - - - - - - - - - - - [MOCK EKF] - - - - - - - - - - - - - - */
// stand-in EKF for LiDAR tests and the simulator (jetson-nano/src/localization/ekf/mockEkf.cpp)

constexpr double MOCK_EKF_ODOM_RATE_HZ = 100.0; // default odom_ekf_ publish rate (A2)
constexpr int64_t MOCK_EKF_HISTORY_US = 1000000; // how far back mainPoseAt() can look (us)

// fixed covariance put on every mock pose
constexpr double MOCK_EKF_POSE_SIGMA_IN = 0.5; // position std dev (in)
constexpr double MOCK_EKF_POSE_SIGMA_RAD = 0.01; // heading std dev (rad)


#endif
