#include "main.h"
#include "odometry.h"
#include "constants.h"

pros::Rotation LTWheel(LEFT_TRACKING_WHEEL_PORT);
pros::Rotation RTWheel(RIGHT_TRACKING_WHEEL_PORT);
pros::Rotation BTWheel(BACK_TRACKING_WHEEL_PORT);

pros::IMU imu(IMU_PORT);

double pos_x = 0;
double pos_y = 0;
double theta = 0;


void update_position_and_angle() {
	// Calculate distance travelled by each tracking wheel
	ArcLengths arcs = get_wheel_travel();

	// Get orientation from
	theta = convertDegToRad(imu.get_yaw());

	// Determine change in heading 
	double del_theta = theta - prevTheta;

	// Determine change in local x and in local y
	double dx_local = (arcs.left + arcs.right) / 2.0;
	double dy_local = arcs.back - (del_theta * BACK_TRACKING_WHEEL_DISTANCE);

	// Compute using midpoint formula
	double heading_mid = theta + (del_theta / 2);

    // Compute change in x and y based on heading and local changes
	double del_x = std::cos(heading_mid) * dx_local - std::sin(heading_mid) * dy_local;
	double del_y = std::sin(heading_mid) * dx_local + std::cos(heading_mid) * dy_local;

	// Increment position and angle by calculated changes
	pos_x += del_x;
	pos_y += del_y;
	theta += del_theta;

	prevTheta = theta;
}


ArcLengths get_wheel_travel() {

    // Get current centidegree position of tracking wheels
	int currLeft = LTWheel.get_position();
	int currRight = RTWheel.get_position();
	int currBack = BTWheel.get_position();

	// Convert centidegrees to degrees and find distance travelled by wheel
	double del_L = (currLeft / 36000.0) * TRACKING_WHEEL_DIAMETER * std::numbers::pi; 
	double del_R = (currRight / 36000.0) * TRACKING_WHEEL_DIAMETER * std::numbers::pi; 
	double del_B = (currBack / 36000.0) * BACK_TRACKING_WHEEL_DIAMETER * std::numbers::pi;

	// Create a structure of lengths
	ArcLengths del(del_L, del_R, del_B);

	// Reset tracking for next measurements
	LTWheel.reset_position();
	RTWheel.reset_position();
	BTWheel.reset_position();

	return del;
}

double compute_heading_change(ArcLengths arcs) {
	// Calculate heading in radians
	return (arcs.right - arcs.left) / (double) (RIGHT_TRACKING_WHEEL_DISTANCE + LEFT_TRACKING_WHEEL_DISTANCE);
}

double convertDegToRad (double degree) {
	return degree * (std::numbers::pi / 180.0);
}