#pragma once

// Robot device assignments shared by both ends of the link.
constexpr int PARALLEL_TRACKING_WHEEL_PORT = 6;
constexpr int PERPENDICULAR_TRACKING_WHEEL_PORT = 10;
#define LEFT_DRIVE_WHEEL_PORTS {-14, -12, -13}
#define RIGHT_DRIVE_WHEEL_PORTS {17, 18, 19}

// drive motor ports for the X and H drives (RobotDrive in config/robotDrive.hpp picks which drive the link configures)
// negative = reversed. set the signs so positive voltage drives each group its positive way (mpc/drivetrain.hpp):
// forward on every X drive wheel, forward on the H drive sides, and left on the H drive strafe wheel
// PLACEHOLDER ports until those drives are built
#define XDRIVE_FL_PORTS {-1}
#define XDRIVE_FR_PORTS {2}
#define XDRIVE_BL_PORTS {-3}
#define XDRIVE_BR_PORTS {4}
#define HDRIVE_LEFT_PORTS LEFT_DRIVE_WHEEL_PORTS
#define HDRIVE_RIGHT_PORTS RIGHT_DRIVE_WHEEL_PORTS
#define HDRIVE_STRAFE_PORTS {11}
constexpr char BALL_BLOCKER_PORT = 'A';
constexpr char SCORING_LIFTER = 'B';
constexpr char MATCHLOADER_PORT = 'C';
constexpr char DESCORE_PORT = 'D';
