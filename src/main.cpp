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

ALS_Path als_path1;
ALS_Path als_path2;

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


	
	als_path1.buildFromPoints(path, 0.25); //(path id, sample spacing)
	als_path1.isValid() ? pros::lcd::print(2, "Path build success") : pros::lcd::print(2, "Path build failure");
	
	als_path2.buildFromPoints(path2, 0.25); //(path id, sample spacing)
	als_path2.isValid() ? pros::lcd::print(3, "Path build success") : pros::lcd::print(3, "Path build failure");
}

void disabled() {}

void competition_initialize() {}

void autonomous() {

	

	PurePursuit purePursuit(path2, als_path2);
	while (not purePursuit.step()) {
		pros::delay(10);
	}
	

	// Autonomous auton = Autonomous();

	// // Go to matchloader
	//  auton.travelToPoint(-36, 0, 70, true);
	

	// // Turn to matchloader
	// unloader.set_value(true);
	// auton.turnTo(90);
	// matchload(false);

	// auton.travelToPoint(-35,-22, 80, true, 5);

	
	// trackingMode(GamePiece::BLUE_BALL);

	// // Approach matchloader
	// move_intake(STOP, HIGH_VOLTAGE, HIGH_VOLTAGE);
	// auton.travel(16, 50, 90, 1.150);

	// Matchload
	// for (int i = 0; i < 2; i++){
	// 	auton.travel(-12, 50, 90, 0.15);
	// 	auton.travel(12, 60, 90, 0.35);
	//  	pros::delay(500);
	// }
	// move_intake(STOP);

	// // Reverse and score on long goal
	// auton.travel(-50, 80, 75, 2);
	// move_intake(-HIGH_VOLTAGE, -HIGH_VOLTAGE, STOP, 0.2);
	// move_intake(HIGH_VOLTAGE, HIGH_VOLTAGE, HIGH_VOLTAGE, 0.1);
	// move_intake(-HIGH_VOLTAGE, -HIGH_VOLTAGE, STOP, 0.2);
	// unloader.set_value(false);
	// move_intake(MAX_VOLTAGE, MAX_VOLTAGE, MAX_VOLTAGE, 3);

	// // Back away from goal
	// auton.travel(20, 80, 90);

	// // Go past goal
	// auton.turnTo(0);
	// auton.travel(24, 70, 0);
	// auton.turnTo(-90);

	// // Travel down field and face balls
	// move_intake(HIGH_VOLTAGE, HIGH_VOLTAGE, HIGH_VOLTAGE);
	// auton.travel(99, 85, -90);
	// auton.turnTo(180);

	// // Get side balls
	// move_intake(STOP, HIGH_VOLTAGE, HIGH_VOLTAGE);
	// auton.travel(40, 70, 180, 2.8);
	// auton.travel(-6, 50, 180, 0.8);
	// auton.travel(8, 60, 180, 0.8);
	// pros::delay(500);
	// move_intake(STOP);

	// // Back up to matchloader
	// auton.travel(-16.25, 70, 180);
	// unloader.set_value(true);
	// auton.turnTo(-90);

	// // Matchload
	// move_intake(STOP, HIGH_VOLTAGE, HIGH_VOLTAGE);
	// auton.travel(16, 50, -90, 1.150);
	// for (int i = 0; i < 2; i++){
	// 	auton.travel(-12, 50, -90, 0.15);
	// 	auton.travel(12, 60, -90, 0.35);
	//  	pros::delay(500);
	// }
	// move_intake(STOP);
	
	// // Approach long goal and score
	// auton.travel(-50, 80, -105, 2);
	// move_intake(-HIGH_VOLTAGE, -HIGH_VOLTAGE, STOP, 0.2);
	// move_intake(HIGH_VOLTAGE, HIGH_VOLTAGE, HIGH_VOLTAGE, 0.1);
	// move_intake(-HIGH_VOLTAGE, -HIGH_VOLTAGE, STOP, 0.2);
	// unloader.set_value(false);
	// move_intake(MAX_VOLTAGE, MAX_VOLTAGE, MAX_VOLTAGE, 1.75);

	// // Move away from goal
	// auton.travel(6, 90, -90);
	// auton.turnTo(0);
	// auton.travel(24, 90, 0);
	// auton.turnTo(90);

	// // Travel down field
	// move_intake(HIGH_VOLTAGE, HIGH_VOLTAGE, HIGH_VOLTAGE);
	// auton.travel(94, 150, 90, 5.25);

	// // Face goal and park
	// auton.turnTo(15);
	// auton.travel(72, 110, 15, 1.1);
	// // auton.travel(-6, 60, 0, 0.5);

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