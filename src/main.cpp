#include "main.h"
#include "subsystems.hpp"
#include "constants.h"
#include "autonomous.hpp"
// #include "slam.h"
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
	// pros::Task frame_task(UpdateFrame_task_fn, (void*)"PROS_Task_Param", TASK_PRIORITY_DEFAULT, TASK_STACK_DEPTH_DEFAULT, "Vision Frame Update");

	// Precompute Sample Tables (need to do this for each path)
	// ALS_Path als_path1;
	
	Logger::getInstance().init("/usd/log.txt");
    LOG("Program Start");

	als_paths = buildAllPaths(0.25);

	// matchloader.set_value(true);

}

void disabled() {}

void competition_initialize() {}
//all after path actions are commented out for path testing purposes
void autonomous() {
	uint32_t startTime;
    double startDist;

	descorer.set_value(true);

    intake.move(MAX_VOLTAGE);
    matchloader.set_value(true); //r
    
	startTime = pros::millis();
	pros::lcd::print(0, "Path: FIRST_MATCHLOAD");
	purePursuit.setPath(als_paths[PathName::FIRST_MATCHLOAD]);
	while (not purePursuit.step()) {
		if (pros::millis() - startTime > 3000) {
			pros::lcd::print(1, "Timeout: FIRST_MATCHLOAD");
			leftMotors.move_velocity(0);
			rightMotors.move_velocity(0);
			break;
		}

		pros::delay(10);
	}

    // matchload
    matchload();

    // prepare for score
    scoringLifter.set_value(true);
    pros::delay(50);
    ballBlocker.set_value(true);
	// intake.move(-MAX_VOLTAGE);
    
	startTime = pros::millis();
	pros::lcd::print(0, "Path: FIRST_SCORE");
	purePursuit.setPath(als_paths[PathName::FIRST_SCORE]);
	while (not purePursuit.step(-1, 0.6)) {
        if (pros::millis() - startTime > 2000) {
            pros::lcd::print(1, "Timeout: FIRST_SCORE");
			leftMotors.move_velocity(0);
			rightMotors.move_velocity(0);
			break;
		}

		// if (odom.getPosY() < 3) {
		// 	matchloader.set_value(true);
		// }
		pros::delay(10);
	}
	// matchloader.set_value(true);
	
    // score
    score();
	intake.move(-MAX_VOLTAGE);
	matchloader.set_value(ture); //r

	// SECOND_MATCHLOAD
	startTime = pros::millis();
	pros::lcd::print(0, "Path: SECOND_MATCHLOAD");
	purePursuit.setPath(als_paths[PathName::SECOND_MATCHLOAD]);
	while (not purePursuit.step()) {
		if (pros::millis() - startTime > 2500) {
			pros::lcd::print(1, "Timeout: SECOND_MATCHLOAD");
			leftMotors.move_velocity(0);
			rightMotors.move_velocity(0);
			break;
		}
		
		if (odom.getPosY() > 5) {
			intake.move(MAX_VOLTAGE);
			// lever.move(STOP);
		} 
		// else if (odom.getPosY() > 0) {
		// 	lever.move(-MAX_VOLTAGE);
		// } else if (odom.getPosY() > -15) {
		// 	lever.move(MAX_VOLTAGE);
		// } 
		
		pros::delay(10);
	}



	lever.move(STOP);

	// lever.move(MAX_VOLTAGE);
	// pros::delay(500);
	// lever.move(-MAX_VOLTAGE);
	// pros::delay(200);
	// lever.move(STOP);

	// scoringLifter.set_value(false);
	matchload();

	// leftMotors.move_velocity(-100);
	// rightMotors.move_velocity(-100);
	// pros::delay(600);
	// leftMotors.move_velocity(0);
	// rightMotors.move_velocity(0);
	lever.move(MAX_VOLTAGE);
	pros::delay(500);
	lever.move(-MAX_VOLTAGE);
	pros::delay(500);
	lever.move(STOP);
	// pros::delay(500);
	// leftMotors.move_velocity(100);
	// rightMotors.move_velocity(100);
	// pros::delay(600);
	// leftMotors.move_velocity(0);
	// rightMotors.move_velocity(0);

	matchload();


	
	
	// BEFORE_MIDDLE
	startTime = pros::millis();
	pros::lcd::print(0, "Path: BEFORE_MIDDLE");
	purePursuit.setPath(als_paths[PathName::BEFORE_MIDDLE]);
	while (not purePursuit.step(-1.0)) {
		if (pros::millis() - startTime > 10000) {
			pros::lcd::print(1, "Timeout: BEFORE_MIDDLE");
			leftMotors.move_velocity(0);
			rightMotors.move_velocity(0);
			break;
		}

		if (odom.getPosY() < 0) {
			matchloader.set_value(false); //r
		}
		
		pros::delay(10);
	}
	matchloader.set_value(false); //r
	

	// MIDDLE
	startTime = pros::millis();
	pros::lcd::print(0, "Path: MIDDLE");
	purePursuit.setPath(als_paths[PathName::MIDDLE]);
	while (not purePursuit.step()) {
        if (pros::millis() - startTime > 5000) {
            pros::lcd::print(1, "Timeout: MIDDLE");
			leftMotors.move_velocity(0);
			rightMotors.move_velocity(0);
			break;
		}

		pros::delay(10);
	}

	// low mid score
	// leftMotors.move_velocity(80);
	// rightMotors.move_velocity(80);
	// pros::delay(100);
	intake.move(-90);
	pros::delay(1000);
	intake.move(STOP);
	// leftMotors.move_velocity(0);
	// rightMotors.move_velocity(0);

	// BEFORE_WING
	startTime = pros::millis();
	pros::lcd::print(0, "Path: BEFORE_WING");
	purePursuit.setPath(als_paths[PathName::BEFORE_WING]);
	while (not purePursuit.step(-1.0)) {
        if (pros::millis() - startTime > 10000) {
            pros::lcd::print(1, "Timeout: BEFORE_WING");
			leftMotors.move_velocity(0);
			rightMotors.move_velocity(0);
			break;
		}

		pros::delay(10);
	}

	scoringLifter.set_value(true);


	startTime = pros::millis();
	pros::lcd::print(0, "Path: WING");
	purePursuit.setPath(als_paths[PathName::WING]);
	while (not purePursuit.step(1.0, 0.7)) {
        if (pros::millis() - startTime > 10000) {
            pros::lcd::print(1, "Timeout: FIRST_SCORE");
			leftMotors.move_velocity(0);
			rightMotors.move_velocity(0);
			break;
		}

		if (odom.getPosY() < -29) {
			descorer.set_value(false);
		}

		pros::delay(10);
	}


}

void opcontrol() {
	leftMotors.set_brake_mode_all(pros::E_MOTOR_BRAKE_COAST);
	rightMotors.set_brake_mode_all(pros::E_MOTOR_BRAKE_COAST);

	intake.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);

	lever.move(-HIGH_VOLTAGE);
	pros::delay(100);
    lever.move(STOP);
	lever.set_zero_position(lever.get_position());

	// bool raised = false;
	// bool lowered = true;
	// bool leverToggle = false;


    bool lifterUp =  false;

    uint32_t leverMoveStart = pros::millis();
    bool leverMovingDown = false;
    uint32_t timeToMoveLeverDown = 500;

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
			matchloader.set_value(true); //r
		} else {
			matchloader.set_value(false); //r
		}
        
        // Raise Lifter
        if (controller.get_digital_new_press(DIGITAL_RIGHT)) {
            // scoringLifter.toggle();
            lifterUp = !lifterUp;
        }

		// Descore Wing
        if (controller.get_digital(DIGITAL_L1)) {
            descorer.set_value(false);
        } else {
            descorer.set_value(lifterUp);
        }


		// if (controller.get_digital_new_press(DIGITAL_X)) {
		// 	leverToggle = !leverToggle;
		// }

		// Lever Hold
		if (controller.get_digital_new_press(DIGITAL_R2)) {
            leverMovingDown = false;
			lever.move(MAX_VOLTAGE);
			ballBlocker.set_value(true);
		} else if(controller.get_digital_new_release(DIGITAL_R2)){
            leverMovingDown = true;
            leverMoveStart = pros::millis();
			// lever.move_absolute(5, 100);
			ballBlocker.set_value(false);
		}

        if (leverMovingDown) {
            if (pros::millis() - leverMoveStart > timeToMoveLeverDown) {
                lever.move(STOP);
                leverMovingDown = false;
            } else {
                lever.move(-MAX_VOLTAGE);
            }
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

        scoringLifter.set_value(lifterUp);

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
					leftMotors.move_voltage(leftY * 12000 / 127);
				} else {
					leftMotors.move_voltage(0);
				}

				if(abs(rightY) > DEADZONE) {
					rightMotors.move_voltage(rightY * 12000 / 127);
				} else { 
					rightMotors.move_voltage(0);
				}
				break;
			}
			case DriveType::SPLIT_ARCADE: {
                int left_adjusted, right_adjusted;

				// Get joystick values
				int power = controller.get_analog(ANALOG_LEFT_Y);
				int turn = controller.get_analog(ANALOG_RIGHT_X);

                // pros::lcd::print(0, "Power: %d", power);
                // pros::lcd::print(1, "Turn: %d", turn);

				int left = power + turn;
				int right = power - turn;

				// int left_adjusted = left * 600 / 127;
				// int right_adjusted = right * 600 / 127;
                left_adjusted = left * 12000 / 127;
                right_adjusted = right * 12000 / 127;

				// Dead zone for both motors
				if(abs(left) > DEADZONE) {
					leftMotors.move_voltage(left_adjusted);
				} else {
					leftMotors.move_voltage(0);
				}

				if(abs(right) > DEADZONE) {
					rightMotors.move_voltage(right_adjusted);
				} else { 
					rightMotors.move_voltage(0);
				}

                // pros::lcd::print(0, "Left: %d", left_adjusted);
                // pros::lcd::print(1, "Right: %d", right_adjusted);

				break;
			}
		}
	} else {
		if (controller.get_digital(DIGITAL_UP)) { // slow move forward
			leftMotors.move_voltage(LOW_VOLTAGE * 12000 / 127);
			rightMotors.move_voltage(LOW_VOLTAGE * 12000 / 127);
		} else if (controller.get_digital(DIGITAL_DOWN)){ // slow move backward
			leftMotors.move_voltage(-LOW_VOLTAGE * 12000 / 127);
			rightMotors.move_voltage(-LOW_VOLTAGE * 12000 / 127);
		}
	}
}



// score
void score() {
	// lever up
    intake.move(MAX_VOLTAGE);
	pros::delay(500);

    lever.move(125);
    pros::delay(500);
    intake.move(-MAX_VOLTAGE);
    pros::delay(500);

	// lever down
	lever.move(-MAX_VOLTAGE);
	pros::delay(600);
	lever.move(STOP);

    intake.move(STOP);
    pros::delay(100);
}


void matchload() {
    intake.move(MAX_VOLTAGE);
    pros::delay(300);

	for (int i = 0; i < 1; i++) {
		leftMotors.move_velocity(-70);
		rightMotors.move_velocity(-70);
		pros::delay(400);
		leftMotors.move_velocity(0);
		rightMotors.move_velocity(0);
		pros::delay(400);
		leftMotors.move_velocity(70);
		rightMotors.move_velocity(70);
		pros::delay(400);
		leftMotors.move_velocity(0);
		rightMotors.move_velocity(0);
		pros::delay(400);
	}
    intake.move(STOP);
}

// void fullMatchload() {
// 	intake.move(MAX_VOLTAGE);
//     pros::delay(300);
//     leftMotors.move_velocity(-70);
//     rightMotors.move_velocity(-70);
//     pros::delay(250);
// 	leftMotors.move_velocity(0);
// 	rightMotors.move_velocity(0);
// 	pros::delay(250);
//     leftMotors.move_velocity(70);
//     rightMotors.move_velocity(70);
//     pros::delay(250);
//     leftMotors.move_velocity(0);
//     rightMotors.move_velocity(0);
// 	pros::delay(400);
//     intake.move(STOP);
// }

void wallBall() {
    int turnSpeed = 40;

    intake.move(MAX_VOLTAGE);
    leftMotors.move_velocity(turnSpeed);
    rightMotors.move_velocity(-turnSpeed);
    pros::delay(500);
    leftMotors.move_velocity(-turnSpeed);
    rightMotors.move_velocity(turnSpeed);
    pros::delay(1000);
    leftMotors.move_velocity(turnSpeed);
    rightMotors.move_velocity(-turnSpeed);
    pros::delay(500);
    leftMotors.move_velocity(0);
    rightMotors.move_velocity(0);
    intake.move(STOP);
}
