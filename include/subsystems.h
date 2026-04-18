#pragma once


#include "main.h"
#include "odometry.hpp"



extern pros::Controller controller;

extern pros::Rotation parallelTrackingWheel;
extern pros::Rotation perpendicularTrackingWheel;

extern pros::MotorGroup leftMotors;
extern pros::MotorGroup rightMotors;

extern pros::Motor frontIntake;
extern pros::Motor midIntake;
extern pros::Motor backIntake;

extern pros::Motor initIntake;

extern pros::Motor leverOne;
extern pros::Motor leverTwo;

extern pros::IMU imu;

extern pros::adi::Pneumatics unloader;
extern pros::adi::Pneumatics centerScore;
extern pros::adi::Pneumatics descorer;


