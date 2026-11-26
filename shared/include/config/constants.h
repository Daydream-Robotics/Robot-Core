/**
 * \file constants.h
 *
 * Contains constants used throughout the robot.
 */

#ifndef CONSTANTS_H
#define CONSTANTS_H

#include <cstddef>
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

constexpr std::size_t LIDAR_NUM_SENSORS = 2; // sensors 0 and 1

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

constexpr double LIDAR_MASK_EPSILON_RAD = 1e-12; // tolerance on bearing mask edges (rad)

// driver sweep building and sync fit
constexpr std::size_t LIDAR_MAX_SCAN_SAMPLES = 4096; // most samples kept in one sweep
constexpr std::size_t LIDAR_SYNC_FIT_WINDOW = 16; // sync times fitted over this many sweeps, ~1.6 s at 10 Hz
constexpr double LIDAR_SYNC_MAX_SLOPE_ERROR = 0.02; // fitted sample period must stay this close to us_per_sample (fraction)
constexpr double LIDAR_SYNC_RESYNC_US = 3000.0; // sync this late is a read delay once, dropped samples twice in a row (us)

// sensor 0 mount, robot frame, x sideways (+X at heading 0), y forward (+Y at heading 0), yaw CW+, TODO: measure, placeholder values
constexpr double LIDAR_0_X_IN = -4.0;
constexpr double LIDAR_0_Y_IN = 6.0;
constexpr double LIDAR_0_PSI_RAD = 0.0; // mount yaw plus the C1's 0 deg offset, check with a target on the left (should read bearing −90 deg)
constexpr double LIDAR_0_HEIGHT_IN = 8.0; // scan plane height, below the field wall top (#144)

// sensor 1 mount, robot frame, x sideways (+X at heading 0), y forward (+Y at heading 0), yaw CW+, TODO: measure, placeholder values
constexpr double LIDAR_1_X_IN = 4.0;
constexpr double LIDAR_1_Y_IN = 6.0;
constexpr double LIDAR_1_PSI_RAD = 0.0; // mount yaw plus the C1's 0 deg offset, check with a target on the left (should read bearing −90 deg)
constexpr double LIDAR_1_HEIGHT_IN = 8.0; // scan plane height, below the field wall top (#144)

// odom_ekf_ pose buffer
constexpr int LIDAR_POSE_BUFFER_CAPACITY = 200; // poses kept, 1 s at 200 Hz
constexpr uint64_t LIDAR_POSE_EXTRAPOLATE_US = 5000; // poseAt() returns the newest pose up to this far past it (us)
constexpr uint64_t LIDAR_POSE_GAP_US = 30000; // gap between odom poses that gets logged (us)
constexpr double LIDAR_POSE_MIN_HZ = 50.0; // odom rate below this gets logged once (Hz)

/* - - - - - - - - - - - - - - [MOCK EKF] - - - - - - - - - - - - - - */
// stand-in EKF for LiDAR tests and the simulator (jetson-nano/src/localization/ekf/mockEkf.cpp)

constexpr double MOCK_EKF_ODOM_RATE_HZ = 100.0; // default odom_ekf_ publish rate (A2)
constexpr int64_t MOCK_EKF_HISTORY_US = 1000000; // how far back mainPoseAt() can look (us)

// fixed covariance put on every mock pose
constexpr double MOCK_EKF_POSE_SIGMA_IN = 0.5; // position std dev (in)
constexpr double MOCK_EKF_POSE_SIGMA_RAD = 0.01; // heading std dev (rad)

// deskew and scan merge
constexpr uint64_t LIDAR_SWEEP_US = 100000; // one C1 sweep at 10 Hz (us)
constexpr int LIDAR_STALE_SWEEPS = 3; // a sensor is stale after this many missed sweeps


#endif
