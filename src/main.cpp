#include "main.h"
#include "subsystems.h"
#include "constants.h"
#include "odometry.h"

#include <numbers>


void initialize() {
	pros::lcd::initialize();
}

void disabled() {}

void competition_initialize() {}

void autonomous() {}

void opcontrol() {
	// Set chassis brake mode to coast
	leftMotors.set_brake_mode_all(pros::E_MOTOR_BRAKE_COAST);
	rightMotors.set_brake_mode_all(pros::E_MOTOR_BRAKE_COAST);

	frontIntake.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
	backIntake.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);

	while(true){

		update_position_and_angle();
	
		/* - - - - - - - - - - - - - - [CHASSIS CONTROLS] - - - - - - - - - - - - - - */

		drive(DriveType::TANK);

		/* - - - - - - - - - - - - - - [MATCH UNLOADER] - - - - - - - - - - - - - - */

		if (controller.get_digital_new_press(DIGITAL_L1)) {
			unloader.toggle();
		}

		/* - - - - - - - - - - - - - - [CENTER TOGGLE] - - - - - - - - - - - - - - */

		if (controller.get_digital_new_press(DIGITAL_A)) {
			centerScore.toggle();
		}

		// if (controller.get_digital(DIGITAL_B)) {
		// 	move_intake(-HIGH_VOLTAGE, -HIGH_VOLTAGE);
		// }

		/* - - - - - - - - - - - - - - [DESCORE TOGGLE] - - - - - - - - - - - - - - */

		if (controller.get_digital_new_press(DIGITAL_L2)) {
			descorer.toggle();
		}

		/* - - - - - - - - - - - - - - [INTAKE] - - - - - - - - - - - - - - */

		// Main intake
		if (controller.get_digital(DIGITAL_R1)) { // intake
			move_intake(HIGH_VOLTAGE);
		} else if (controller.get_digital(DIGITAL_R2)){ // outtake
			move_intake(HIGH_VOLTAGE, HIGH_VOLTAGE);
		} else if (controller.get_digital(DIGITAL_L1)){ // score
			//descorer.toggle();
		} else if (controller.get_digital(DIGITAL_B)) { // full outtake
			move_intake(-HIGH_VOLTAGE, -HIGH_VOLTAGE);
		} else {
			move_intake(STOP);
		}

		// Delay added to prevent crashing
		pros::delay(20);
	}
}

void move_intake(int front, int back, double seconds) {

	// check for stalling later and stop motors if stalling

	frontIntake.move(front);
	midIntake.move(front);
	backIntake.move(back);

	if (seconds != 0) {
		pros::delay(seconds * 1000);

		frontIntake.move(STOP);
		midIntake.move(STOP);
		backIntake.move(STOP);
	}
}

void drive(DriveType type) {
	if (!controller.get_digital(DIGITAL_UP) && !controller.get_digital(DIGITAL_DOWN)) {
		switch (type) {
			case DriveType::TANK: {
				// Get joystick values
				int leftY = controller.get_analog(ANALOG_LEFT_Y);
				int rightY = controller.get_analog(ANALOG_RIGHT_Y);

				// Dead zone for both motors
				if(abs(leftY) > DEADZONE) {
					leftMotors.move(leftY);
				} else {
					leftMotors.move(STOP);
				}

				if(abs(rightY) > DEADZONE) {
					rightMotors.move(rightY);
				} else { 
					rightMotors.move(STOP);
				}
				break;
			}
			case DriveType::SPLIT_ARCADE: {
				// Get joystick values
				int power = controller.get_analog(ANALOG_LEFT_Y);
				int turn = controller.get_analog(ANALOG_RIGHT_X);

				int left = power + turn;
				int right = power - turn;

				// Dead zone for both motors
				if(abs(left) > DEADZONE) {
					leftMotors.move(left);
				} else {
					leftMotors.move(0);
				}

				if(abs(right) > DEADZONE) {
					rightMotors.move(right);
				} else { 
					rightMotors.move(0);
				}
				break;
			}
		}
	} else {
		if (controller.get_digital(DIGITAL_UP)) { // slow move forward
			leftMotors.move(LOW_VOLTAGE);
			rightMotors.move(LOW_VOLTAGE);
		} else if (controller.get_digital(DIGITAL_DOWN)){ // slow move backward
			leftMotors.move(-LOW_VOLTAGE);
			rightMotors.move(-LOW_VOLTAGE);
		}
	}
}