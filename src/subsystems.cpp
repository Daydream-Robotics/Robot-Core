#include "main.h"
#include "constants.h"
#include "subsystems.h"



pros::Controller controller(pros::E_CONTROLLER_MASTER);



pros::Rotation parallelTrackingWheel(PARALLEL_TRACKING_WHEEL_PORT);
pros::Rotation perpendicularTrackingWheel(PERPENDICULAR_TRACKING_WHEEL_PORT);

pros::MotorGroup leftMotors(LEFT_DRIVE_WHEEL_PORTS);
pros::MotorGroup rightMotors(RIGHT_DRIVE_WHEEL_PORTS);

pros::Motor frontIntake(FRONT_INTAKE_PORT);
pros::Motor midIntake(MID_INTAKE_PORT);
pros::Motor backIntake(BACK_INTAKE_PORT);

pros::Motor initIntake(INITIAL_INTAKE_PORT);
pros::Motor leverOne(LEVER_ONE_PORT, pros::MotorGears::red);
pros::Motor leverTwo(LEVER_TWO_PORT,  pros::MotorGears::red);

pros::IMU imu(IMU_PORT);

pros::adi::Pneumatics unloader(UNLOADER_PORT, false);
pros::adi::Pneumatics centerScore(CENTER_SCORE_PORT, false);
pros::adi::Pneumatics descorer(DESCORE_PORT, false);
