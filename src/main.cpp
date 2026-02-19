#include "main.h"
#include "subsystems.h"
#include "constants.h"
#include "autonomous.hpp"

#include <numbers>


void initialize() {
	pros::lcd::initialize();
	pros::lcd::print(0, "Reg: Initialize");
	imu.reset();
	while (imu.is_calibrating()) {
		pros::delay(20);
	}
}

void disabled() {}

void competition_initialize() {}

void autonomous() {

	Autonomous auton = Autonomous();

	// // Intake pre-loads
	move_intake(STOP, HIGH_VOLTAGE, HIGH_VOLTAGE);
	auton.travel(8, 40, 0);
	// travelDistanceWithHeading(8, 40, 0, -1);
	// pros::delay(250);

	// // Backup to match loader
	auton.travel(-41.5, 70, 0);
	// travelDistanceWithHeading(-35.25, 70, 0, -1);

	// // Face match loader
	move_intake(STOP, STOP, STOP);
	auton.turnTo(90);
	// turn_pid(90, 0);

	// // Match load
	unloader.set_value(true);
	pros::delay(250);
	move_intake(STOP, HIGH_VOLTAGE, HIGH_VOLTAGE);
	auton.travel(16, 50, 90, 1.150);
	// travelDistanceWithHeading(14, 70, 90, 1350);
	// travelDistanceWithHeading(100, 35, 90, 1000);
	auton.travel(-12, 50, 90, 0.10);
	auton.travel(12, 50, 90, 0.1);
	pros::delay(3000);
	move_intake(STOP);
	// travelDistanceWithHeading(100, 25, 90, 1000);

	// travelDistanceWithHeading(100, 35, 90, 1000);


	// // Reverse to Long Goal
	// travelDistanceWithHeading(-50, 60, 90, 2000);
	auton.travel(-50, 80, 75, 2);
	move_intake(-HIGH_VOLTAGE, -HIGH_VOLTAGE, STOP, 0.3);
	unloader.set_value(false);
	move_intake(HIGH_VOLTAGE, HIGH_VOLTAGE, HIGH_VOLTAGE, 3);

	auton.travel(20, 80, 90);

	auton.turnTo(25);

	move_intake(HIGH_VOLTAGE, HIGH_VOLTAGE, HIGH_VOLTAGE);
	auton.travel(70, 100, 25, 1.5);

	pros::delay(1000);
	move_intake(STOP);


	// move_intake(-MID_VOLTAGE, -MID_VOLTAGE, 0.2);
	// // Score on Long Goal
	// move_intake(HIGH_VOLTAGE, HIGH_VOLTAGE, 3);
	// unloader.set_value(false);


	// // Backup and face side balls
	// travelDistanceWithHeading(16, 70, 90, -1);
	// turn_pid(0, 0);
	
	// move_intake(HIGH_VOLTAGE, HIGH_VOLTAGE);
	// // Park
	// travelDistanceWithHeading(40, 100, 115, 1900);
	
	// move_intake(STOP, STOP);

}

void opcontrol() {
	// Set chassis brake mode to coast
	leftMotors.set_brake_mode_all(pros::E_MOTOR_BRAKE_COAST);
	rightMotors.set_brake_mode_all(pros::E_MOTOR_BRAKE_COAST);

	frontIntake.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
	backIntake.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);

	while(true){
	
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


		/* - - - - - - - - - - - - - - [DESCORE TOGGLE] - - - - - - - - - - - - - - */

		if (controller.get_digital_new_press(DIGITAL_L2)) {
			descorer.toggle();
		}

		/* - - - - - - - - - - - - - - [INTAKE] - - - - - - - - - - - - - - */

		// Main intake
		if (controller.get_digital(DIGITAL_R1)) { // intake
			move_intake(STOP, HIGH_VOLTAGE, HIGH_VOLTAGE);
		} else if (controller.get_digital(DIGITAL_R2)){ // outtake
			move_intake(HIGH_VOLTAGE, HIGH_VOLTAGE, HIGH_VOLTAGE);
		} else if (controller.get_digital(DIGITAL_L1)){ // score
			//descorer.toggle();
		} else if (controller.get_digital(DIGITAL_B)) { // full outtake
			move_intake(-HIGH_VOLTAGE, -HIGH_VOLTAGE, -HIGH_VOLTAGE);
		} else {
			move_intake(STOP);
		}

		// Delay added to prevent crashing
		pros::delay(20);
	}
}

void move_intake(int front, int mid, int back, double seconds) {

	// check for stalling later and stop motors if stalling

	frontIntake.move(front);
	midIntake.move(mid);
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