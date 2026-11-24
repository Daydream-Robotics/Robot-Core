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

/* - - - - - - - - - - - - - - [NANO IMU] - - - - - - - - - - - - - - */
// BNO085 on the Nano's I2C bus (jetson-nano/src/localization/imu/bno085.cpp)

constexpr int NANO_IMU_RATE_HZ = 100; // game rotation vector report rate requested from the chip (Hz)
constexpr double NANO_IMU_HISTORY_SEC = 1.0; // how far back headingIncrement() can look (s)
constexpr int NANO_IMU_MAX_READ_ERRORS = 10; // consecutive read errors before recover()
constexpr int NANO_IMU_RESET_ATTEMPTS = 3; // chip resets tried in a row by recover() before giving up until the next error streak
constexpr int NANO_IMU_POLL_HZ = 400; // how often the read thread checks for a packet, sample times are good to about half of 1 / this
constexpr int NANO_IMU_REPORT_TIMEOUT_MS = 200; // no heading report for this long means the chip is stuck, so recover()
constexpr int NANO_IMU_MAX_EXTRAPOLATE_US = 10000; // how far past the newest sample thetaAt() extrapolates (us), one report period
constexpr unsigned NANO_IMU_ACCURACY_TRUSTED = 2; // report accuracy needed for valid data (0 unreliable, 1 low, 2 medium, 3 high)
constexpr double NANO_IMU_YAW_SIGN = 1.0; // 1 with the chip's Z up, -1 if the board is mounted upside down
constexpr bool NANO_IMU_SAVE_CALIBRATION = true; // save the chip's calibration to its own flash once it's trusted


enum class DriveType {
    SPLIT_ARCADE,
    TANK
};

/* - - - - - - - - - - - - - - [ODOM EKF] - - - - - - - - - - - - - - */
// starting values from the EKF design doc, tune with robot logs (jetson-nano/src/localization/ekf/odomEkf.cpp)

// process noise, how fast the velocities can change between cycles
constexpr double ODOM_EKF_ACCEL_SIGMA = 200.0; // linear acceleration std dev (in/s^2), about 0 to 60 in/s in 0.3 s
constexpr double ODOM_EKF_ANG_ACCEL_SIGMA = 20.0; // angular acceleration std dev (rad/s^2), about 0 to 4 rad/s in 0.2 s
constexpr double ODOM_EKF_MAX_DT = 0.1; // longer gap between cycles is a link stall (s), velocity uncertainty is reopened

// measurement noise per row, sigma = SIGMA_PER_IN * |ds| + FLOOR (+ IME turn term)
constexpr double ODOM_EKF_TRACKING_SIGMA_PER_IN = 0.01; // tracking wheel error per inch of travel
constexpr double ODOM_EKF_TRACKING_SIGMA_FLOOR = 0.002; // tracking wheel error when still (in), encoder resolution
constexpr double ODOM_EKF_IME_SIGMA_PER_IN = 0.05; // drive IME error per inch of travel, drive wheels slip
constexpr double ODOM_EKF_IME_SIGMA_PER_OMEGA = 0.01; // extra IME error per rad/s of turn rate (in), wheel scrub in turns
constexpr double ODOM_EKF_IME_SIGMA_FLOOR = 0.005; // drive IME error when still (in)
constexpr double ODOM_EKF_IME_SIGMA_PER_ACCEL = 1e-4; // extra IME error per in/s^2 of forward acceleration (in), placeholder, 0.02 in at 200 in/s^2
// TODO (#193): retune on the robot, these are the BNO055's integrated-gyro values. The BNO085 game
// rotation vector's increment error is mostly heading jitter plus drift (about 0.5 deg/min, 1.5e-4 rad/s)
constexpr double ODOM_EKF_IMU_NOISE_DENSITY = 2.4e-4; // IMU increment noise (rad/sqrt(s)), heading sigma = this * sqrt(dt)
constexpr double ODOM_EKF_IMU_BIAS_SIGMA = 0.005; // IMU heading drift (rad/s), heading sigma += this * dt

// per-row outlier gate on nu^2 / S, chi-squared 1 DOF at 99%
constexpr double ODOM_EKF_ROW_GATE = 6.63;

/* - - - - - - - - - - - - - - [ODOM INPUTS] - - - - - - - - - - - - - - */
// link sample -> OdomInputs (jetson-nano/src/localization/ekf/odomInputs.cpp)

// slots in the link config, TODO: confirm against the config the Nano sends the brain
constexpr int ODOM_LEFT_GROUP = 0; // left drive motor group
constexpr int ODOM_RIGHT_GROUP = 1; // right drive motor group
constexpr int ODOM_PAR_WHEEL = 0; // parallel tracking wheel rotation sensor
constexpr int ODOM_PERP_WHEEL = 1; // perpendicular tracking wheel rotation sensor

// sanity limits, faster than this can't be real motion
constexpr double ODOM_INPUT_MAX_SPEED = 200.0; // in/s, 2 in per 10 ms
constexpr double ODOM_INPUT_MAX_OMEGA = 20.0; // rad/s
constexpr double ODOM_INPUT_STALL_DT = 0.2; // s, longer gap means the link stalled


/* - - - - - - - - - - - - - - [FULL EKF] - - - - - - - - - - - - - - */
// starting guesses, tune with robot logs (jetson-nano/src/localization/ekf/fullEkf.cpp)

// extra process noise per predict, for what odometry can't see (collisions, being pushed)
constexpr double FULL_EKF_Q_EXTRA_XY_SIGMA = 0.01; // per-cycle position std dev (in)
constexpr double FULL_EKF_Q_EXTRA_THETA_SIGMA = 0.0005; // per-cycle heading std dev (rad)

// update gate on d^2 = nu^T S^-1 nu, chi-squared 3 DOF at 99%
constexpr double FULL_EKF_GATE = 11.34;

// recovery: after this many consecutive gate rejections, P is raised to the recovery covariance
constexpr int FULL_EKF_RECOVERY_REJECTS = 5; // about 0.5 s of ICP
constexpr double FULL_EKF_RECOVERY_XY_SIGMA = 12.0; // recovery position std dev (in)
constexpr double FULL_EKF_RECOVERY_THETA_SIGMA = 0.26; // recovery heading std dev (rad)

// history for late measurements, about 1 s at 10 ms cycles
constexpr int FULL_EKF_HISTORY_SIZE = 100;


/* - - - - - - - - - - - - - - [LIDAR] - - - - - - - - - - - - - - */
// starting values from the LiDAR plan's parameter table; tune with the simulator and robot logs

constexpr double LIDAR_TAU_0 = 6.0;       // starting ICP max match distance before any data (in)
constexpr double LIDAR_DELTA_MIN = 0.5;   // ICP corrections smaller than this are ignored by the adaptive threshold (in)
constexpr double LIDAR_R_MAX = 204.0;     // max range considered, ~ diagonal of a 12 ft field (in)


#endif
