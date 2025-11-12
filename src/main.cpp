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

	frontIntake.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
	mainIntake.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);

	bool pistonToggle = false, pistonLatch = false;

	while(true){
		// Arcade control scheme
		int leftY = controller.get_analog(ANALOG_LEFT_Y);    // Gets amount forward/backward from left joystick
		int rightX = controller.get_analog(ANALOG_RIGHT_X);  // Gets the turn left/right from right joystick

		if(abs(leftY) < DEADZONE) {
			leftY = 0;
		}

		if(abs(rightX) < DEADZONE) {
			rightX = 0;
		}

		leftMotors.move(leftY + rightX); // Sets left motor voltage
		rightMotors.move(leftY - rightX); // Sets right motor voltage

		// Main intake
		if (controller.get_digital(DIGITAL_R1)) {
			frontIntake.move(127);
		} else if (controller.get_digital(DIGITAL_R2)) {
			mainIntake.move(127);
		} else if (controller.get_digital(DIGITAL_LEFT)) {
			frontIntake.move(-60);
			mainIntake.move(-60);
		}

		// Front intake 
		else if (controller.get_digital(DIGITAL_UP)){
			frontIntake.move(60);
			mainIntake.move(60);
		} else if (controller.get_digital(DIGITAL_DOWN)){
			frontIntake.move(-127);
			mainIntake.move(-127);
	    } else {
			frontIntake.move(0);
			mainIntake.move(0);
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