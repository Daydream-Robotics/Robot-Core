#pragma once


#include "main.h"
#include "odometry.hpp"
#include "constants.h"

inline pros::Controller controller(pros::E_CONTROLLER_MASTER);

inline pros::Rotation parallelTrackingWheel(PARALLEL_TRACKING_WHEEL_PORT);
inline pros::Rotation perpendicularTrackingWheel(PERPENDICULAR_TRACKING_WHEEL_PORT);

inline pros::MotorGroup leftMotors(LEFT_DRIVE_WHEEL_PORTS, pros::v5::MotorGears::blue);
inline pros::MotorGroup rightMotors(RIGHT_DRIVE_WHEEL_PORTS, pros::v5::MotorGears::blue);

// inline pros::Motor frontIntake(FRONT_INTAKE_PORT);
// inline pros::Motor midIntake(MID_INTAKE_PORT);
// inline pros::Motor backIntake(BACK_INTAKE_PORT);

// inline pros::Motor initIntake(INITIAL_INTAKE_PORT);
// inline pros::Motor initIntake2(INITIAL_INTAKE_PORT2);
inline pros::MotorGroup intake({INITIAL_INTAKE_PORT, INITIAL_INTAKE_PORT2});
// inline pros::Motor leverOne(LEVER_ONE_PORT, pros::MotorGears::red);
// inline pros::Motor leverTwo(LEVER_TWO_PORT,  pros::MotorGears::red);
inline pros::MotorGroup lever({LEVER_ONE_PORT, LEVER_TWO_PORT}, pros::v5::MotorGears::red);

inline pros::IMU imu(IMU_PORT);

inline pros::adi::Pneumatics ballBlocker(BALL_BLOCKER_PORT, false);
inline pros::adi::Pneumatics scoringLifter(SCORING_LIFTER, false);
inline pros::adi::Pneumatics matchloader(MATCHLOADER_PORT, false);
inline pros::adi::Pneumatics descorer(DESCORE_PORT, false);
