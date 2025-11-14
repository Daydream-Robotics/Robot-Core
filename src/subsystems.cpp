#include "main.h"
#include "constants.h"

pros::Controller controller(pros::E_CONTROLLER_MASTER);

pros::MotorGroup leftMotors(LEFT_DRIVE_WHEEL_PORTS);
pros::MotorGroup rightMotors(RIGHT_DRIVE_WHEEL_PORTS);

pros::Motor frontIntake(FRONT_INTAKE_PORT);
pros::Motor mainIntake(MAIN_INTAKE_PORT);
pros::Motor backIntake(BACK_INTAKE_PORT);

pros::adi::Pneumatics piston(PNEUMATIC_PORT, false);