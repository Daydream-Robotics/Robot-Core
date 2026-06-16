#include "main.h"
#include "sd_card_logging.hpp"
#include "subsystems.hpp"
#include "autonomous.hpp"
#include "paths.hpp"
#include "pathFollower.hpp"
#include "mpcSerial.hpp"

Autonomous auton = Autonomous();

// Initialize our remote serial MPC follower (20ms loop interval)
MPCSerial::Params mpc_serial_params(0.02);
MPCSerial mpc_serial_controller(mpc_serial_params);
PathFollower pathFollower(mpc_serial_controller);

std::vector<ALS_Path> paths;

void initialize() {
	pros::lcd::initialize();
	pros::lcd::print(0, "MPC Serial Initialization");

	Logger::getInstance().init("/usd/log.txt");
	LOG("VEX Brain tracking code starts");
	
	imu.reset();
	while (imu.is_calibrating()) {
		pros::delay(20);
	}

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
	printf("[MAIN] Initialize complete\n");
}

void disabled() {}

void competition_initialize() {}

void autonomous() {
	printf("[MAIN] Starting autonomous() under MPC Guidance\n");
	
	if (paths.size() <= PathName::FIRST_PATH) {
		printf("[MAIN-ERROR] FIRST_PATH out of bounds\n");
		return;
	}

	printf("[MAIN] Target: FIRST_PATH... Starting MPC\n");
	pathFollower.setPath(paths[PathName::FIRST_PATH]);
	
	while (not pathFollower.step()) {
		pros::delay(20); // Syncs with our 20ms MPC period
	}
	printf("[MAIN] FIRST_PATH tracking complete.\n");

	pros::delay(1000);

	if (paths.size() <= PathName::SECOND_PATH) {
		printf("[MAIN-ERROR] SECOND_PATH out of bounds\n");
		return;
	}

	printf("[MAIN] Target: SECOND_PATH... Starting MPC\n");
	pathFollower.setPath(paths[PathName::SECOND_PATH]);
	while (not pathFollower.step()) {
		pros::delay(20);
	}
	printf("[MAIN] SECOND_PATH tracking complete.\n");
}

void opcontrol() {
	leftMotors.set_brake_mode_all(pros::E_MOTOR_BRAKE_COAST);
	rightMotors.set_brake_mode_all(pros::E_MOTOR_BRAKE_COAST);
	
	// Un-comment to run motor characterization routine on startup:
	// MPCSerial::identifyMotorModel();

	descorer.set_value(true);
	intake.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);

	lever.move(-HIGH_VOLTAGE);
	pros::delay(100);
	lever.move(STOP);
	lever.set_zero_position(lever.get_position());

	bool lifterUp = true;
	uint32_t leverMoveStart = pros::millis();
	bool leverMovingDown = false;
	uint32_t timeToMoveLeverDown = 500;

	while (true) {
		drive(DriveType::SPLIT_ARCADE);

		// Intake controller binds
		if (controller.get_digital(DIGITAL_R1)) {
			intake.move(HIGH_VOLTAGE);
		} else if (controller.get_digital(DIGITAL_L2)) {
			intake.move(-HIGH_VOLTAGE);
		} else {
			intake.move(STOP);
		}

		// Matchloader
		if (controller.get_digital(DIGITAL_Y)) {
			matchloader.set_value(true);
		} else {
			matchloader.set_value(false);
		}
        
		// Lifter
		if (controller.get_digital_new_press(DIGITAL_RIGHT)) {
			lifterUp = !lifterUp;
		}

		// Descorer
		if (controller.get_digital(DIGITAL_L1)) {
			descorer.set_value(false);
		} else {
			descorer.set_value(lifterUp);
		}

		// Lever Controller Logic
		if (controller.get_digital_new_press(DIGITAL_R2)) {
			leverMovingDown = false;
			lever.move(MAX_VOLTAGE);
			ballBlocker.set_value(true);
		} else if(controller.get_digital_new_release(DIGITAL_R2)){
			leverMovingDown = true;
			leverMoveStart = pros::millis();
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
		pros::delay(20);
	}
}

void drive(DriveType type) {
	if (!controller.get_digital(DIGITAL_UP) && !controller.get_digital(DIGITAL_DOWN)) {
		switch (type) {
			case DriveType::TANK: {
				int leftY = controller.get_analog(ANALOG_LEFT_Y);
				int rightY = controller.get_analog(ANALOG_RIGHT_Y);

				if (abs(leftY) > DEADZONE) {
					leftMotors.move_voltage(leftY * 12000 / 127);
				} else {
					leftMotors.move_voltage(0);
				}

				if (abs(rightY) > DEADZONE) {
					rightMotors.move_voltage(rightY * 12000 / 127);
				} else { 
					rightMotors.move_voltage(0);
				}
				break;
			}
			case DriveType::SPLIT_ARCADE: {
				int power = controller.get_analog(ANALOG_LEFT_Y);
				int turn = controller.get_analog(ANALOG_RIGHT_X);

				int left = power + turn;
				int right = power - turn;

				int left_adjusted = left * 12000 / 127;
				int right_adjusted = right * 12000 / 127;

				if (abs(left) > DEADZONE) {
					leftMotors.move_voltage(left_adjusted);
				} else {
					leftMotors.move_voltage(0);
				}

				if (abs(right) > DEADZONE) {
					rightMotors.move_voltage(right_adjusted);
				} else { 
					rightMotors.move_voltage(0);
				}
				break;
			}
		}
	} else {
		if (controller.get_digital(DIGITAL_UP)) {
			leftMotors.move_voltage(LOW_VOLTAGE * 12000 / 127);
			rightMotors.move_voltage(LOW_VOLTAGE * 12000 / 127);
		} else if (controller.get_digital(DIGITAL_DOWN)) {
			leftMotors.move_voltage(-LOW_VOLTAGE * 12000 / 127);
			rightMotors.move_voltage(-LOW_VOLTAGE * 12000 / 127);
		}
	}
}

void score() {
	intake.move(MAX_VOLTAGE);
	pros::delay(500);

	lever.move(125);
	pros::delay(500);
	intake.move(-MAX_VOLTAGE);
	pros::delay(500);

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
		pros::delay(350);
		leftMotors.move_velocity(0);
		rightMotors.move_velocity(0);
		pros::delay(500);
	}
	intake.move(STOP);
}

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