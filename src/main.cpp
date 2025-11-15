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
}

void disabled() {}

void competition_initialize() {}

void autonomous() {
	leftMotors.set_brake_mode_all(pros::E_MOTOR_BRAKE_BRAKE);
	rightMotors.set_brake_mode_all(pros::E_MOTOR_BRAKE_BRAKE);

	while (imu.is_calibrating()) {
		pros::delay(20);
	}

	// move_pid({ 10, 20 }, 0, 0);
	// pros::delay(1000);
	// move_pid({ 25, 50 }, 0, 1);
	// pros::delay(1000);
	// move_pid({ 0, 0 }, 0, 1);
	// pros::delay(500);
	// turn_pid(0, 0);

	move_pid({ 160, 0 }, 80, 0, 0, 2);

}

void opcontrol() {
	// Set chassis brake mode to coast
	leftMotors.set_brake_mode_all(pros::E_MOTOR_BRAKE_COAST);
	rightMotors.set_brake_mode_all(pros::E_MOTOR_BRAKE_COAST);

	while(true){

		// Start autonomous
		if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_L2)) {
			autonomous();
		}

		update_position_and_angle();
		//controller.print(0, 0, "0: %.1lf", theta * (180.0 / M_PI));
		controller.print(0, 0, "X: %.1lf Y: %.1lf O: %.1lf", pos_x, pos_y, theta * (180.0 / std::numbers::pi));
		// Get joystick values
		int leftY = controller.get_analog(ANALOG_LEFT_Y);
		int rightY = controller.get_analog(ANALOG_RIGHT_Y);

		// Dead zone for both motors
		if(abs(leftY) > DEADZONE) {
			leftMotors.move(leftY);
		} else {
			leftMotors.move(0);
		}

		if(abs(rightY) > DEADZONE) {
			rightMotors.move(rightY);
		} else { 
			rightMotors.move(0);
		}

		// Delay added to prevent crashing
		pros::delay(20);
	}

}