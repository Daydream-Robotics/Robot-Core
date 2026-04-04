#include "main.h"
#include "subsystems.h"
#include "constants.h"
#include "autonomous.hpp"
#include "slam.h"
#include "objectHandler.h"
#include <numbers>

#include "purePursuit.hpp"


void initialize() {
	pros::lcd::initialize();
	pros::lcd::print(0, "Reg: Initialize");
	imu.reset();
	while (imu.is_calibrating()) {
		pros::delay(20);
	}
	pros::Task frame_task(UpdateFrame_task_fn, (void*)"PROS_Task_Param", TASK_PRIORITY_DEFAULT, TASK_STACK_DEPTH_DEFAULT, "Vision Frame Update");
}

void disabled() {}

void competition_initialize() {}

void autonomous() {
std::vector<Position> path = {
		{0.00, 0.00},
		{1.00, -0.00},
		{2.00, 0.00},
		{3.00, 0.02},
		{4.00, 0.04},
		{5.00, 0.07},
		{6.00, 0.10},
		{7.00, 0.13},
		{8.00, 0.16},
		{9.00, 0.19},
		{10.00, 0.21},
		{11.00, 0.21},
		{12.00, 0.19},
		{13.00, 0.14},
		{13.99, 0.04},
		{14.98, -0.10},
		{15.95, -0.33},
		{16.91, -0.63},
		{17.82, -1.04},
		{18.67, -1.56},
		{19.45, -2.19},
		{20.14, -2.91},
		{20.74, -3.71},
		{21.27, -4.56},
		{21.72, -5.45},
		{22.11, -6.37},
		{22.45, -7.31},
		{22.75, -8.26},
		{23.02, -9.23},
		{23.25, -10.20},
		{23.45, -11.18},
		{23.64, -12.16},
		{23.80, -13.15},
		{23.95, -14.14},
		{24.08, -15.13},
		{24.21, -16.12},
		{24.31, -17.12},
		{24.41, -18.11},
		{24.50, -19.11},
		{24.58, -20.10},
		{24.66, -21.10},
		{24.73, -22.10},
		{24.79, -23.10},
		{24.84, -24.10},
		{24.89, -25.09},
		{24.94, -26.09},
		{24.98, -27.09},
		{25.02, -28.09},
		{25.05, -29.09},
		{25.08, -30.09},
		{25.11, -31.09},
		{25.14, -32.09},
		{25.16, -33.09},
		{25.18, -34.09},
		{25.19, -35.09},
		{25.21, -36.09},
		{25.22, -37.09},
		{25.23, -38.09},
		{25.24, -39.09},
		{25.25, -40.09},
		{25.26, -41.09},
		{25.26, -42.09},
		{25.26, -43.09},
		{25.27, -44.09},
		{25.27, -45.09},
		{25.28, -46.09},
		{25.28, -47.09},
		{25.29, -48.09},
		{25.30, -49.09},
		{25.30, -50.09},
		{25.31, -51.09},
		{25.31, -52.09},
		{25.32, -53.09},
		{25.32, -54.09},
		{25.33, -55.09},
		{25.33, -56.09},
		{25.33, -57.09},
		{25.33, -58.09},
		{25.33, -59.09},
		{25.33, -60.09},
		{25.33, -61.09},
		{25.32, -62.09},
		{25.31, -63.09},
		{25.30, -64.09},
		{25.29, -65.09},
		{25.27, -66.09},
		{25.25, -67.09},
		{25.22, -68.09},
		{25.18, -69.09},
		{25.14, -70.09},
		{25.08, -71.08},
		{25.01, -72.08},
		{24.93, -73.08},
		{24.83, -74.07},
		{24.71, -75.07},
		{24.55, -76.05},
		{24.35, -77.03},
		{24.09, -78.00},
		{23.75, -78.94},
		{23.27, -79.81},
		{22.59, -80.54},
		{21.69, -80.97},
		{20.70, -81.04},
		{19.72, -80.85},
		{18.78, -80.51},
		{17.87, -80.09},
		{17.00, -79.61},
		{16.14, -79.10},
		{15.30, -78.56},
		{14.46, -78.00},
		{13.64, -77.43},
		{12.83, -76.85},
		{12.02, -76.26},
		{11.22, -75.67},
		{10.41, -75.07},
		{9.62, -74.47},
		{8.82, -73.87},
		{8.02, -73.26},
		{7.23, -72.65},
		{6.43, -72.05},
		{5.64, -71.44},
		{4.85, -70.83},
		{4.05, -70.23},
		{3.25, -69.62},
		{2.46, -69.01},
		{1.66, -68.41},
		{0.86, -67.81},
		{-0.12, -67.78},
		{-1.12, -67.82},
		{-2.12, -67.87},
		{-3.12, -67.91},
		{-4.11, -67.95},
		{-5.11, -67.99},
		{-6.11, -68.03},
		{-7.11, -68.06},
		{-8.11, -68.07},
		{-9.11, -68.08},
		{-10.11, -68.07},
		{-11.11, -68.05},
		{-12.11, -68.00},
		{-13.11, -67.93},
		{-14.10, -67.82},
		{-15.09, -67.68},
		{-16.07, -67.49},
		{-17.04, -67.25},
		{-18.00, -66.95},
		{-18.93, -66.59},
		{-19.83, -66.15},
		{-20.68, -65.63},
		{-21.49, -65.04},
		{-22.23, -64.37},
		{-22.90, -63.63},
		{-23.49, -62.82},
		{-24.02, -61.97},
		{-24.47, -61.08},
		{-24.85, -60.16},
		{-25.18, -59.21},
		{-25.45, -58.25},
		{-25.67, -57.27},
		{-25.85, -56.29},
		{-25.99, -55.30},
		{-26.10, -54.31},
		{-26.18, -53.31},
		{-26.23, -52.31},
		{-26.26, -51.31},
		{-26.27, -50.31},
		{-26.25, -49.31},
		{-26.22, -48.31},
		{-26.18, -47.31},
		{-26.11, -46.32},
		{-26.04, -45.32},
		{-25.95, -44.32},
		{-25.85, -43.33},
		{-25.74, -42.33},
		{-25.62, -41.34},
		{-25.50, -40.35},
		{-25.36, -39.36},
		{-25.21, -38.37},
		{-25.05, -37.38},
		{-24.88, -36.40},
		{-24.70, -35.41},
		{-24.50, -34.43},
		{-24.28, -33.46},
		{-24.05, -32.49},
		{-23.79, -31.52},
		{-23.52, -30.56},
		{-23.22, -29.60},
		{-22.89, -28.66},
		{-22.53, -27.72},
		{-22.14, -26.81},
		{-21.71, -25.90},
		{-21.24, -25.02},
		{-20.72, -24.17},
		{-20.14, -23.35},
		{-19.51, -22.57},
		{-18.83, -21.84},
		{-18.08, -21.18},
		{-17.27, -20.59},
		{-16.42, -20.07},
		{-15.52, -19.64},
		{-14.58, -19.29},
		{-13.62, -19.01},
		{-12.64, -18.80},
		{-11.66, -18.65},
		{-10.66, -18.54},
		{-9.66, -18.48},
		{-8.66, -18.45},
		{-7.66, -18.44},
		{-6.66, -18.45},
		{-5.66, -18.47},
		{-4.66, -18.50},
		{-3.66, -18.53},
		{-2.66, -18.56},
		{-1.67, -18.59},
		{-0.98, -18.60},
		{-0.98, -18.60}
	};



	PurePursuit purePursuit(path);

	bool finished = false;
	while (not finished) {
		// odom.updatePose();


		finished = purePursuit.step();
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