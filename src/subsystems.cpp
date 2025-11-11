#include "main.h"
#include "constants.h"

pros::Controller controller(pros::E_CONTROLLER_MASTER);

pros::MotorGroup leftMotors(LEFT_DRIVE_WHEEL_PORTS);
pros::MotorGroup rightMotors(RIGHT_DRIVE_WHEEL_PORTS);

pros::Motor leftIntake(LEFT_INTAKE_PORT);
pros::Motor rightIntake(RIGHT_INTAKE_PORT);

pros::adi::Pneumatics piston(PNEUMATIC_PORT, false);