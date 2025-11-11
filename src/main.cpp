#include "main.h"
#include "subsystems.h"
#include "constants.h"

void initialize() {
	pros::lcd::initialize();
}

void disabled() {}

void competition_initialize() {}

void autonomous() {}

void opcontrol() {
	// Set chassis brake mode to coast
	leftMotors.set_brake_mode_all(pros::E_MOTOR_BRAKE_COAST);
	rightMotors.set_brake_mode_all(pros::E_MOTOR_BRAKE_COAST);

	bool pistonToggle = false, pistonLatch = false;

	while(true){
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

		// Intaking/outtaking
		if(controller.get_digital(DIGITAL_L1)) {
			leftIntake.move(MAX_VOLTAGE);
			rightIntake.move(MAX_VOLTAGE);
		} else if(controller.get_digital(DIGITAL_R1)) {
			leftIntake.move(-MAX_VOLTAGE);
			rightIntake.move(-MAX_VOLTAGE);
		} else {
			leftIntake.move(0);
			rightIntake.move(0);
		}

		// Pneumatics
		if (pistonToggle)
			piston.set_value(true); // turns clamp solenoid on
		else
			piston.set_value(false); // turns clamp solenoid off

		if (controller.get_digital_new_press(DIGITAL_B)) {
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