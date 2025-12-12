#include "main.h"
#include "subsystems.h"
#include "constants.h"
#include "odometry.h"

#include <numbers>


void initialize() {
	pros::lcd::initialize();

	// Initialize tracking wheels to zero
	LTWheel.reset_position();
	RTWheel.reset_position();
	BTWheel.reset_position();

	imu.reset();

	while (imu.is_calibrating()) {
		pros::delay(20);
	}
}

void disabled() {}

void competition_initialize() {}

void autonomous() {
	leftMotors.set_brake_mode_all(pros::E_MOTOR_BRAKE_BRAKE);
	rightMotors.set_brake_mode_all(pros::E_MOTOR_BRAKE_BRAKE);

	// Move away from parking zone
	move_dist_pid(34.5, 50, -1, true);

	// Turn to match loader
	turn_pid(-90, 0);

	piston.set_value(true);
	pros::delay(1000);

	// Intake from matchloader
	frontIntake.move(HIGH_VOLTAGE);
	mainIntake.move(HIGH_VOLTAGE);

	// Move to matchloader
	move_dist_pid(12, 35, 1250, false);
	pros::delay(2500);

	// Back up
	move_dist_pid(20, 35, -1, true);

	// Approach Long Goal and Unleash
	move_dist_pid(7, 15, 1250, true);

	mainIntake.move(-MID_VOLTAGE);
	backIntake.move(-MID_VOLTAGE);
	pros::delay(300);

	frontIntake.move(MID_VOLTAGE);
	mainIntake.move(MID_VOLTAGE);
	backIntake.move(MID_VOLTAGE);
	pros::delay(800);
	backIntake.move(STOP);

	move_dist_pid(10, 35, 1500, false);
	backIntake.move(MID_VOLTAGE);
	move_dist_pid(10, 34, 1500, false);

	move_dist_pid(7, 15, 1000, false);
	pros::delay(2500);

}

void opcontrol() {
	// Set chassis brake mode to coast
	leftMotors.set_brake_mode_all(pros::E_MOTOR_BRAKE_COAST);
	rightMotors.set_brake_mode_all(pros::E_MOTOR_BRAKE_COAST);

	frontIntake.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
	mainIntake.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
	backIntake.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);

	bool pistonToggle = false, pistonLatch = false;

	while(true){

		// Start autonomous
		if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_B)) {
			autonomous();
		} else if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_X)) {
		}

		update_position_and_angle();
		controller.print(0, 0, "X: %.1lf Y: %.1lf O: %.1lf", pos_x, pos_y, theta * (180.0 / std::numbers::pi));
		// Get joystick values
		int leftY = controller.get_analog(ANALOG_LEFT_Y);
		int rightY = controller.get_analog(ANALOG_RIGHT_Y);

		// Dead zone for both motors
		if (!controller.get_digital(DIGITAL_UP) && !controller.get_digital(DIGITAL_DOWN)) {
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
		} else {
			if (controller.get_digital(DIGITAL_UP)) {
				leftMotors.move(LOW_VOLTAGE);
				rightMotors.move(LOW_VOLTAGE);
			} else if (controller.get_digital(DIGITAL_DOWN)){
				leftMotors.move(-LOW_VOLTAGE);
				rightMotors.move(-LOW_VOLTAGE);
			}
		}

		// Main intake
		if (controller.get_digital(DIGITAL_R1)) {
			frontIntake.move(HIGH_VOLTAGE);
			mainIntake.move(HIGH_VOLTAGE);
		} else if (controller.get_digital(DIGITAL_R2)){
			frontIntake.move(-HIGH_VOLTAGE);
			mainIntake.move(-HIGH_VOLTAGE);
	    } else if (controller.get_digital(DIGITAL_L2)) {
			frontIntake.move(-HIGH_VOLTAGE);
			mainIntake.move(-HIGH_VOLTAGE);
			backIntake.move(-HIGH_VOLTAGE);
		} else if (controller.get_digital(DIGITAL_L1)){
			frontIntake.move(HIGH_VOLTAGE);
			mainIntake.move(HIGH_VOLTAGE);
			backIntake.move(HIGH_VOLTAGE);
		} else if (controller.get_digital(DIGITAL_A)) {
			mainIntake.move(HIGH_VOLTAGE);
		} else {
			frontIntake.move(STOP);
			mainIntake.move(STOP);
			backIntake.move(STOP);
		}

		// // Top outtake (testing)
		// if(controller.get_digital(DIGITAL_X)) {
		// 	backIntake.move(HIGH_VOLTAGE);
		// } else if (controller.get_digital(DIGITAL_B)) {
		// 	backIntake.move(-HIGH_VOLTAGE);
		// } else {
		// 	backIntake.move(STOP);
		// }

		// Match unloader
		if (pistonToggle) {
			piston.set_value(true); // turns clamp solenoid on
		} else {
			piston.set_value(false); // turns clamp solenoid off
		}

		pros::delay(10);

		if (controller.get_digital_new_press(DIGITAL_LEFT)) {
			if(!pistonLatch){ // if latch is false, flip toggle one time and set latch to true
				pistonToggle = !pistonToggle;
				pistonLatch = true;
			}
		}
		else
			pistonLatch = false; // once button is released then release the latch too

		// Delay added to prevent crashing
		pros::delay(20);
	}

}