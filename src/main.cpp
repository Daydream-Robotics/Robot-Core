#include "main.h"
#include "subsystems.h"
#include "constants.h"

#include <numbers>


#define LEFT_TRACKING_WHEEL_DISTANCE 1.65
#define RIGHT_TRACKING_WHEEL_DISTANCE 1.65
#define BACK_TRACKING_WHEEL_DISTANCE 2.4

#define TRACKING_WHEEL_DIAMETER 2.00
	
#define BACK_TRACKING_WHEEL_DIAMETER 2.75

pros::Rotation LTWheel(LEFT_TRACKING_WHEEL_PORT);
pros::Rotation RTWheel(RIGHT_TRACKING_WHEEL_PORT);
pros::Rotation BTWheel(BACK_TRACKING_WHEEL_PORT);


double del_theta;

// Position is in inches
// Theta is in degrees
double pos_x = 0, pos_y = 0, theta = 0;

typedef struct ArcLengths {
	double left;
	double right;
	double back;
} ArcLengths;

// Get arc lengths and updates encoder values
ArcLengths get_wheel_travel() {

	int currLeft = LTWheel.get_position();
	int currRight = RTWheel.get_position();
	int currBack = BTWheel.get_position();

	// Convert centidegrees to degrees and find distance travelled by wheel
	double del_L = (currLeft / 36000.0) * TRACKING_WHEEL_DIAMETER * std::numbers::pi; 
	double del_R = (currRight / 36000.0) * TRACKING_WHEEL_DIAMETER * std::numbers::pi; 
	double del_B = (currBack / 36000.0) * BACK_TRACKING_WHEEL_DIAMETER * std::numbers::pi;

	// Create a structure of lengths
	ArcLengths del;
	del.left = del_L;
	del.right = del_R;
	del.back = del_B;

	// Reset tracking for next measurements
	LTWheel.reset_position();
	RTWheel.reset_position();
	BTWheel.reset_position();

	return del;
}

// Returns the change in heading value (degrees) using parallel tracker wheels
double compute_heading_change(ArcLengths arcs) {
	// Calculate heading in radians
	double rads = (arcs.right - arcs.left) / (double) (RIGHT_TRACKING_WHEEL_DISTANCE + LEFT_TRACKING_WHEEL_DISTANCE);
	// Return heading in degrees
	return rads * (180.0 / std::numbers::pi);
}

// Convert Degrees to Radians
double convertDegToRad (double degree) {
	return degree * (std::numbers::pi / 180.0);
}

// Calculate and update global variables holding position and orientation
void update_position_and_angle() {
	// Calculate distance travelled by each tracking wheel
	ArcLengths arcs = get_wheel_travel();

	// Determine change in heading 
	del_theta = compute_heading_change(arcs);

	// Determine change in local x and in local y
	double dx_local = (arcs.left + arcs.right) / 2.0;
	double dy_local = arcs.back - (convertDegToRad(del_theta) * BACK_TRACKING_WHEEL_DISTANCE);

	// Compute using midpoint formula
	double heading_mid = convertDegToRad(theta + (del_theta / 2));

	double del_x = std::cos(heading_mid) * dx_local - std::sin(heading_mid) * dy_local;
	double del_y = std::sin(heading_mid) * dx_local + std::cos(heading_mid) * dy_local;

	// Increment position and angle by calculated changes
	pos_x += del_x;
	pos_y += del_y;
	theta += del_theta;

	// Keep theta between 0 and 360
	theta = std::fmod(theta, 360.0);
    if (theta < 0) theta += 360.0;
}


/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
void initialize() {
	pros::lcd::initialize();

	// Initialize tracking wheels to zero
	LTWheel.reset_position();
	RTWheel.reset_position();
	BTWheel.reset_position();
}

void disabled() {}

void competition_initialize() {}

void autonomous() {}

void opcontrol() {
	// Set chassis brake mode to coast
	leftMotors.set_brake_mode_all(pros::E_MOTOR_BRAKE_COAST);
	rightMotors.set_brake_mode_all(pros::E_MOTOR_BRAKE_COAST);

	while(true){
		update_position_and_angle();
		controller.print(0, 0, "X: %.1lf Y: %.1lf O: %.1lf", pos_x, pos_y, theta);
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