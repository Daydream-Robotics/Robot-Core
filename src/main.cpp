#include "main.h"
#include "subsystems.hpp"
#include "constants.h"
#include "autonomous.hpp"
#include "slam.h"
#include "objectHandler.h"
#include <numbers>
#include "arclengthSplining.hpp"
#include "paths.hpp"
#include "sd_card_logging.hpp"
#include "purePursuit.hpp"

// ALS_Path als_path1;
// ALS_Path als_path2;
// ALS_Path als_path3;
std::vector<ALS_Path> als_paths;
PurePursuit purePursuit;
Autonomous auton = Autonomous();

void initialize() {
	// Initialize subsystems
	pros::lcd::initialize();
	pros::lcd::print(0, "Reg: Initialize");
	imu.reset();
	while (imu.is_calibrating()) {
		pros::delay(20);
	}
	pros::Task frame_task(UpdateFrame_task_fn, (void*)"PROS_Task_Param", TASK_PRIORITY_DEFAULT, TASK_STACK_DEPTH_DEFAULT, "Vision Frame Update");

	// Precompute Sample Tables (need to do this for each path)
	// ALS_Path als_path1;
	
	Logger::getInstance().init("/usd/log.txt");
    LOG("Program Start");

	als_paths = buildAllPaths(0.25);

	matchloader.set_value(true);
}

void disabled() {}

void competition_initialize() {}

void autonomous() {
	// Pure pursuit test
	purePursuit.setPath(als_paths[0]);
	while (not purePursuit.step()) {
		pros::delay(10);
	}

	purePursuit.setPath(als_paths[1]);
	while (not purePursuit.step()) {
		pros::delay(10);
	}

}

void opcontrol() {
	leftMotors.set_brake_mode_all(pros::E_MOTOR_BRAKE_COAST);
	rightMotors.set_brake_mode_all(pros::E_MOTOR_BRAKE_COAST);

	intake.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);

	lever.move(-HIGH_VOLTAGE);
	pros::delay(100);
	lever.set_zero_position(lever.get_position());

	bool raised = false;
	bool lowered = true;
	bool leverToggle = false;

	while(true){

		/* - - - - - - - - - - - - - - [CHASSIS CONTROLS] - - - - - - - - - - - - - - */

		drive(DriveType::SPLIT_ARCADE);

		// /* - - - - - - - - - - - - - - [MATCH UNLOADER] - - - - - - - - - - - - - - */

		// if (controller.get_digital_new_press(DIGITAL_L1)) {
		// 	unloader.toggle();
		// }

		// /* - - - - - - - - - - - - - - [CENTER TOGGLE] - - - - - - - - - - - - - - */

		// if (controller.get_digital_new_press(DIGITAL_A)) {
		// 	centerScore.toggle();
		// }


		// /* - - - - - - - - - - - - - - [DESCORE TOGGLE] - - - - - - - - - - - - - - */

		// if (controller.get_digital_new_press(DIGITAL_L2)) {
		// 	descorer.toggle();
		// }

		// /* - - - - - - - - - - - - - - - - [AI] - - - - - - - - - - - - - - - - - - */		
		// if(controller.get_digital_new_press(DIGITAL_LEFT)){
		// 	matchload(false);
		// }

		// if(controller.get_digital_new_press(DIGITAL_RIGHT)){
		// 	collect(GamePiece::RED_BALL, 4);
		// }

		// if(controller.get_digital_new_press(DIGITAL_DOWN)){
		// 	collect(GamePiece::RED_BALL);
		// }
		
		/* - - - - - - - - - - - - - - [INTAKE] - - - - - - - - - - - - - - */
		// Main intake
		// if (controller.get_digital(DIGITAL_R1)) { // intake
		// 	move_intake(STOP, HIGH_VOLTAGE, HIGH_VOLTAGE);
		// } else if (controller.get_digital(DIGITAL_R2)){ // outtake
		// 	move_intake(HIGH_VOLTAGE, HIGH_VOLTAGE, HIGH_VOLTAGE);
		// } else if (controller.get_digital(DIGITAL_L1)){ // score
		// 	//descorer.toggle();
		// } else if (controller.get_digital(DIGITAL_B)) { // full outtake
		// 	move_intake(-HIGH_VOLTAGE, -HIGH_VOLTAGE, -HIGH_VOLTAGE);
		// } else {
		// 	move_intake(STOP);
		// }

		// NEW INTAKE
		if (controller.get_digital(DIGITAL_R1)) { // intake
			intake.move(HIGH_VOLTAGE);
		} else if (controller.get_digital(DIGITAL_L2)) { // outtake
			intake.move(-HIGH_VOLTAGE);
		} else {
			intake.move(STOP);
		}

		// Matchloader
		if (controller.get_digital(DIGITAL_Y)) {
			matchloader.set_value(false);
		} else {
			matchloader.set_value(true);
		}

		// Descore Wing
		if (controller.get_digital(DIGITAL_L1)) {
			descorer.set_value(true);
		} else {
			descorer.set_value(false);
		}

		// Raise Lifter
		if (controller.get_digital_new_press(DIGITAL_RIGHT)) {
			scoringLifter.toggle();
		}

		// if (controller.get_digital_new_press(DIGITAL_X)) {
		// 	leverToggle = !leverToggle;
		// }

		// Lever Hold
		if (controller.get_digital_new_press(DIGITAL_R2)) {
			lever.move(MAX_VOLTAGE);
			ballBlocker.set_value(true);
		} else if(controller.get_digital_new_release(DIGITAL_R2)){
			lever.move_absolute(0, 100);
			ballBlocker.set_value(false);
		}



		

		// 	raised = false;
		// 	if (not raised) { // if lowered, raise
		// 		lever.move(MAX_VOLTAGE);
		// 		pros::delay(100);
		// 	}
		// 	if (lever.get_actual_velocity() < 5) {
		// 		lever.move(STOP);
		// 		raised = true;
		// 		lowered = false;
		// 	}
		// } else if (raised) {
		// 	if (not lowered) {
		// 		lever.move_absolute(0, 100);
		// 		pros::delay(100);
		// 	}
		// 	if (lever.get_actual_velocity() < 5) {
		// 		lever.move(STOP);
		// 		lowered = true;
		// 		raised = false;
		// 	}
		// } else {
		// 	lever.move(STOP);
		// }

		// // Lever Toggle
		// if (controller.get_digital_new_press(DIGITAL_X)) {
		// 	lever.move(MAX_VOLTAGE);
		// 	pros::delay(100);
		// 	if (lever.get_actual_velocity() < 5) {
		// 		lever.move(STOP);
		// 	}
		// } else if (controller.get_digital_new_press(DIGITAL_B)) {
		// 	lever.move_absolute(0, 100);
		// }



		// Delay added to prevent crashing
		pros::delay(20);
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