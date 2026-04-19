#include "main.h"
#include "subsystems.h"
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
PurePursuit purePursuit = PurePursuit();
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


	
	// als_path1.buildFromPoints(path, 0.25); //(path id, sample spacing)
	// als_path1.isValid() ? pros::lcd::print(2, "Path build success") : pros::lcd::print(2, "Path build failure");
	
	// als_path2.buildFromPoints(path2, 0.25); //(path id, sample spacing)
	// als_path2.isValid() ? pros::lcd::print(3, "Path build success") : pros::lcd::print(3, "Path build failure");

	// als_path3.buildFromPoints(path3, 0.25); //(path id, sample spacing)
	// als_path3.isValid() ? pros::lcd::print(3, "Path build success") : pros::lcd::print(3, "Path build failure");

	als_paths = buildAllPaths(0.25);
}

void disabled() {}

void competition_initialize() {}

void autonomous() {

	// Path 1

	// Pure pursuit test
	purePursuit.setPath(als_paths[0]);
	while (not purePursuit.step()) {
		if ((purePursuit.m_distFromEnd < 10.0 && GetObject(GamePiece::RED_BALL).has_value()) && IsConnected()) { // If we're within 10 inches of the end of the path and we see a red ball, break to collect it
			break;
		}
		pros::delay(10);
	}

	// Collect balls
	if (IsConnected()) {
		collect(GamePiece::RED_BALL);
		auton.turnTo(-90);
		collect(GamePiece::BLUE_BALL);
	}
	move_intake(STOP, STOP, STOP, 0.5);

	// Path 2

	purePursuit.setPath(als_paths[1]);
	while (not purePursuit.step()) {
		pros::delay(10);
	}
}

void opcontrol() {
	// Set chassis brake mode to coast

	
	// Autonomous auton = Autonomous();

	// // Go to matchloader
	// // auton.travel(-36, 70, 0);

	// // Turn to matchloader
	// // unloader.set_value(true);
	// // auton.turnTo(90);

	//  matchload(false);

	// collect(GamePiece::RED_BALL, 4);

	 


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

		/* - - - - - - - - - - - - - - - - [AI] - - - - - - - - - - - - - - - - - - */		
		if(controller.get_digital_new_press(DIGITAL_LEFT)){
			matchload(false);
		}

		if(controller.get_digital_new_press(DIGITAL_RIGHT)){
			collect(GamePiece::RED_BALL, 4);
		}

		if(controller.get_digital_new_press(DIGITAL_DOWN)){
			collect(GamePiece::RED_BALL);
		}

		if(controller.get_digital_new_press(DIGITAL_UP)){
			trackingMode(GamePiece::RED_BALL);
		}

		if(controller.get_digital_new_press(DIGITAL_Y)){
			while(true){
				collect(GamePiece::RED_BALL);

				if(controller.get_digital_new_press(DIGITAL_Y)){
					break;
				}
			}
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