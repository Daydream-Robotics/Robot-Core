/**
 * \file constants.h
 *
 * Contains constants used throughout the robot.
 */

#ifndef CONSTANTS_H
#define CONSTANTS_H

#include "config/linkConstants.hpp"
#include <cstddef>

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
constexpr double TRACK_WIDTH = 10.5; // drive wheel track width (in), also used by the Nano's MPC

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

/* - - - - - - - - - - - - - - [NANO] - - - - - - - - - - - - - - */
// Nano program settings. the link hardware isn't confirmed yet (docs/SERIAL_PROTOCOL_INTEGRATION_STATUS.md)

constexpr bool NANO_LINK_USE_USB = true; // true: brain USB port, false: RS-485
constexpr const char* NANO_LINK_DEVICE = "/dev/ttyACM1"; // brain USB port, or the RS-485 UART
constexpr int NANO_LINK_BAUD = 921600; // RS-485 baud, ignored over USB
constexpr int NANO_DE_GPIO = -1; // sysfs GPIO for the RS-485 DE and /RE pins, -1 = none
constexpr const char* NANO_PATH_FILE = "path.jerryio.txt"; // JerryIO path file, relative to where the program runs

/* - - - - - - - - - - - - - - [MPC] - - - - - - - - - - - - - - */
// used by the Nano's MPC. wheel size, track width and gear ratio come from [ROBOT DIMS]. tuning notes in docs/MPC_DOCS.md

// horizons (compile-time, the controller's matrix sizes depend on them)
constexpr std::size_t MPC_PREDICTION_HORIZON = 15; // F, steps
constexpr std::size_t MPC_CONTROL_HORIZON = 15; // V, steps
constexpr double MPC_SAMPLE_PERIOD = 0.02; // control step (s)

// motor model at the wheel: omega_dot = -a*omega + b*V
constexpr double MPC_MOTOR_A = 16.7211; // motor time-constant (1/s)
constexpr double MPC_MOTOR_B = 71.3105; // motor gain constant (rad/s^2 per V)

// reference generation
constexpr double MPC_A_MAX = 60.0; // max accel/decel of the reference (in/s^2)
constexpr double MPC_V_REF_MIN = 3.0; // reference speed floor (in/s)

// cost weights
constexpr double MPC_P_X = 200.0; // output weight for x
constexpr double MPC_P_Y = 200.0; // output weight for y
constexpr double MPC_P_THETA = 60.0; // output weight for theta
constexpr double MPC_Q_U = 0.1; // input change penalty
constexpr double MPC_Q_ZERO_MULT = 2.0; // multiplier for the first input change penalty
constexpr double MPC_Q_FINAL_MULT = 1.0; // multiplier for the last input change penalty
constexpr double MPC_P_FINAL_MULT = 20.0; // multiplier for the last output weight

// limits
constexpr double MPC_V_MAX = 12.0; // max drive voltage (V)
constexpr double MPC_DELTA_U_MAX = 8.0; // max voltage increase per step (V)
constexpr double MPC_DELTA_U_MIN = -8.0; // max voltage decrease per step (V)
constexpr double MPC_X_FIELD_MAX = 140.0; // max |x| on the field (in)
constexpr double MPC_Y_FIELD_MAX = 140.0; // max |y| on the field (in)
constexpr double MPC_OMEGA_WHEEL_MAX = 100.0; // max wheel speed (rad/s)

enum class DriveType {
    SPLIT_ARCADE,
    TANK
};


#endif
