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

	turn_pid(-90, 0);

	pros::delay(5000);

	turn_pid(90, 0);

	// // Drive in front of the match loader
	// move_pid({ 34, 0 }, 25, 0, 0, -1);
	// pros::delay(300);

	// // Start intake
	// frontIntake.move(HIGH_VOLTAGE);
	// mainIntake.move(HIGH_VOLTAGE);
	// pros::delay(300);

	// // Drop pneumatic
	// piston.set_value(true);
	// pros::delay(300);

	// // Ram match loader
	// move_pid({ 34, 19 }, 50, 0, 0, 7);
	// pros::delay(300);

	// // Back up from match loader
	// move_pid({ 34, 10 }, 25, 0, 1, -1);

	// // Put up pneumatic
	// piston.set_value(false);
	// pros::delay(300);

	// // Stop intake
	// frontIntake.move(STOP);
	// mainIntake.move(STOP);
	// pros::delay(300);

	// // Drive to match loader
	// move_pid({ 34, -30 }, 25, 0, 1, 8);
	// pros::delay(300);

	// // Outtake
	// frontIntake.move(HIGH_VOLTAGE);
	// mainIntake.move(HIGH_VOLTAGE);
	// backIntake.move(HIGH_VOLTAGE);
	// pros::delay(7000);

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
			turn_pid(90, 0);
		} else if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_X)) {
			move_dist_pid(75, 45);
			turn_pid(90, 0);
			move_dist_pid(75, 45);
			turn_pid(180, 0);
			move_dist_pid(75, 45);
			turn_pid(-90, 0);
			move_dist_pid(75, 45);
			turn_pid(0, 0);
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