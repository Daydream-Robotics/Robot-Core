#include "main.h"
#include <numbers>

// All Distances in Inches

#define LEFT_TRACKING_WHEEL_PORT 1
#define RIGHT_TRACKING_WHEEL_PORT 2
#define BACK_TRACKING_WHEEL_PORT 3

#define LEFT_TRACKING_WHEEL_DISTANCE 6.4
#define RIGHT_TRACKING_WHEEL_DISTANCE 6.4
#define BACK_TRACKING_WHEEL_DISTANCE 5

#define TRACKING_WHEEL_DIAMETER 3.25
#define BACK_TRACKING_WHEEL_DIAMETER 3.00

pros::Rotation LTWheel(LEFT_TRACKING_WHEEL_PORT);
pros::Rotation RTWheel(RIGHT_TRACKING_WHEEL_PORT);
pros::Rotation BTWheel(BACK_TRACKING_WHEEL_PORT);

// Position is in inches
// Theta is in degrees
double pos_x = 0, pos_y = 0, theta = 0;

// Testing Vars
double distTraveled = 0;

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

	ArcLengths del;
	del.left = del_L;
	del.right = del_R;
	del.back = del_B;

	// Tracking for testing
	distTraveled += del_L;

	LTWheel.reset_position();
	RTWheel.reset_position();
	BTWheel.reset_position();

	return del;
}

// Returns the change in heading value using parallel tracker wheels
double compute_heading_change(ArcLengths arcs) {
	return (arcs.right - arcs.left) / (RIGHT_TRACKING_WHEEL_DISTANCE + LEFT_TRACKING_WHEEL_DISTANCE);
}

// Calculate and update global variables holding position and orientation
void update_position_and_angle() {
	// Calculate distance travelled by each tracking wheel
	ArcLengths arcs = get_wheel_travel();
	
	// Determine change in heading 
	double del_theta = compute_heading_change(arcs);

	// Determine change in local x and in local y
	double dx_local = (arcs.left + arcs.right) / 2.0;
	double dy_local = arcs.back - (del_theta * -BACK_TRACKING_WHEEL_DISTANCE);

	// Compute using midpoint formula
	double heading_mid = theta + (del_theta / 2);

	// Calculate change in x and y
	double del_x = std::cos(heading_mid) * dx_local - std::sin(heading_mid) * dy_local;
	double del_y = std::sin(heading_mid) * dx_local - std::sin(heading_mid) * dy_local;

	// Increment position and angle by calculated changes
	pos_x += del_x;
	pos_y += del_y;
	theta += del_theta;

	// Keep theta between 0 and 360
	if (theta < 0) theta += 360;
	if (theta > 360) theta -= 360;
}


/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
void initialize() {
	pros::lcd::initialize();
}

/**
 * Runs while the robot is in the disabled state of Field Management System or
 * the VEX Competition Switch, following either autonomous or opcontrol. When
 * the robot is enabled, this task will exit.
 */
void disabled() {}

/**
 * Runs after initialize(), and before autonomous when connected to the Field
 * Management System or the VEX Competition Switch. This is intended for
 * competition-specific initialization routines, such as an autonomous selector
 * on the LCD.
 *
 * This task will exit when the robot is enabled and autonomous or opcontrol
 * starts.
 */
void competition_initialize() {}

/**
 * Runs the user autonomous code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the autonomous
 * mode. Alternatively, this function may be called in initialize or opcontrol
 * for non-competition testing purposes.
 *
 * If the robot is disabled or communications is lost, the autonomous task
 * will be stopped. Re-enabling the robot will restart the task, not re-start it
 * from where it left off.
 */
void autonomous() {}

/**
 * Runs the operator control code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the operator
 * control mode.
 *
 * If no competition control is connected, this function will run immediately
 * following initialize().
 *
 * If the robot is disabled or communications is lost, the
 * operator control task will be stopped. Re-enabling the robot will restart the
 * task, not resume it from where it left off.
 */
void opcontrol() {

	while (true) {
		update_position_and_angle();
		pros::lcd::print(0, "Position X: %lf", pos_x);
		pros::lcd::print(1, "Position Y: %lf", pos_y);
		pros::lcd::print(2, "Orientation: %lf", theta);
		pros::delay(20);
	}

}