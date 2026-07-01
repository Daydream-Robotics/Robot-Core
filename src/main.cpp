#include "main.h"
#include "sd_card_logging.hpp"
// #include "constants.h"
#include "subsystems.hpp"
#include "autonomous.hpp"
#include "paths.hpp"
#include "pathFollower.hpp"
#include "purePursuit.hpp"


// #include "slam.h"
// #include "objectHandler.h"
// #include <numbers>
// #include "arclengthSplining.hpp"

Autonomous auton = Autonomous();

PurePursuitController purePursuit = PurePursuitController();
PathFollower pathFollower = PathFollower(purePursuit);

std::vector<ALS_Path> paths;

void initialize() {
	// Initialize subsystems
	pros::lcd::initialize();
	pros::lcd::print(0, "Reg: Initialize");

	// logger setup
	Logger::getInstance().init("/usd/log.txt");
	LOG("Program Start");
	
	// imu setup
	imu.reset();
	while (imu.is_calibrating()) {
		pros::delay(20);
	}

	// path setup
	printf("[MAIN] Loading paths...\n");
	paths = Path::buildAllPathsFromJerryIO("/usd/path.jerryio.txt");
	printf("[MAIN] Paths loaded: %zu\n", paths.size());
	if (paths.size() != PathName::COUNT) {
		printf("[MAIN] ERROR: Path count mismatch\n");
		pros::lcd::print(1, "ERROR: Path count mismatch");
		pros::lcd::print(2, "Expected: %d, Actual: %d", PathName::COUNT, paths.size());
	} else {
		printf("[MAIN] Trajectories Loaded successfully\n");
		pros::lcd::print(1, "Trajectories Loaded");
	}

	// object handler setup
	// pros::Task frame_task(UpdateFrame_task_fn, (void*)"PROS_Task_Param", TASK_PRIORITY_DEFAULT, TASK_STACK_DEPTH_DEFAULT, "Vision Frame Update");
	printf("[MAIN] Initialize complete\n");
}

void disabled() {}

void competition_initialize() {}
//all after path actions are commented out for path testing purposes
void autonomous() {
	printf("[MAIN] Starting autonomous()\n");
	
	if (paths.size() <= PathName::FIRST_PATH) {
		printf("[MAIN-ERROR] FIRST_PATH index out of bounds! Array size is %zu\n", paths.size());
		return;
	}


	FieldLogger sin_log(LoggerType::VALUE, "test","sine");
	FieldLogger cos_log(LoggerType::VALUE, "test","cosine");
	printf("[MAIN] Setting FIRST_PATH...\n");
	pathFollower.setPath(paths[PathName::FIRST_PATH], PathFlag::FORWARDS, true, "test");
	printf("[MAIN] FIRST_PATH set. Tracking...\n");
	while (not pathFollower.step()) {
		sin_log.log(sin(pros::millis()/1000), pros::millis()/1000);
		cos_log.log(cos(pros::millis()/1000), pros::millis()/1000);
		pros::delay(20);
	}
	sin_log.flush();
	sin_log.close();
	cos_log.flush();
	cos_log.close();
	pros::lcd::print(0, "done Loaded");
	printf("[MAIN] FIRST_PATH tracking complete.\n");

	printf("[MAIN] Delaying 2000ms...\n");
	pros::delay(2000);
	if (paths.size() <= PathName::SECOND_PATH) {
		printf("[MAIN-ERROR] SECOND_PATH index out of bounds! Array size is %zu\n", paths.size());
		return;
	}

	printf("[MAIN] Setting SECOND_PATH...\n");
	
	pathFollower.setPath(paths[PathName::SECOND_PATH], PathFlag::REVERSE, true, "test_back");
	printf("[MAIN] SECOND_PATH set. Tracking...\n");
	while (not pathFollower.step()) {
		pros::delay(20);
	}
	printf("[MAIN] SECOND_PATH tracking complete.\n");

}

void opcontrol() {
	leftMotors.set_brake_mode_all(pros::E_MOTOR_BRAKE_COAST);
	rightMotors.set_brake_mode_all(pros::E_MOTOR_BRAKE_COAST);

	descorer.set_value(true);

	intake.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);

	lever.move(-HIGH_VOLTAGE);
	pros::delay(100);
    lever.move(STOP);
	lever.set_zero_position(lever.get_position());

	// bool raised = false;
	// bool lowered = true;
	// bool leverToggle = false;

	
    bool lifterUp =  true;

    uint32_t leverMoveStart = pros::millis();
    bool leverMovingDown = false;
    uint32_t timeToMoveLeverDown = 500;

	while(true){

		drive(DriveType::SPLIT_ARCADE);

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


void matchload(int numRam) {
    intake.move(MAX_VOLTAGE);
    pros::delay(300);

	for (int i = 0; i < numRam; i++) {
		leftMotors.move_velocity(-70);
		rightMotors.move_velocity(-70);
		pros::delay(400);
		leftMotors.move_velocity(0);
		rightMotors.move_velocity(0);
		pros::delay(300);
		leftMotors.move_velocity(70);
		rightMotors.move_velocity(70);
		pros::delay(350); //400
		leftMotors.move_velocity(0);
		rightMotors.move_velocity(0);
		pros::delay(500);
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