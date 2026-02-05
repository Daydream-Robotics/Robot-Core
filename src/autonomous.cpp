#include "autonomous.hpp"
#include "subsystems.h"

Autonomous::Autonomous() {}

void Autonomous::travel(double distance, double speed, double target_heading, double timer_s) {}

void Autonomous::updatePose(void) {
	// Calculate distance travelled by each tracking wheel
	WheelLengths arcs = getOdomWheelTravel();

	static double prevTheta = convertDegToRad(getYaw() - 180);

	// Get orientation from IMU
	double theta = convertDegToRad(getYaw() - 180);
	theta = normalizeAngle(theta);

	// Determine change in heading 
	double del_theta = normalizeAngle(theta - prevTheta);

	// Determine change in local x and in local y
	double dx_local = arcs.parallel;
	double dy_local = arcs.perpendicular - (del_theta * PERPINDICULAR_TRACKING_WHEEL_DISTANCE);

	double theta_mid = prevTheta + del_theta / 2.0;
    theta_mid = normalizeAngle(theta_mid);

    // Compute change in x and y based on heading and local changes
	double del_x = std::cos(theta_mid) * dx_local - std::sin(theta_mid) * dy_local;
	double del_y = std::sin(theta_mid) * dx_local + std::cos(theta_mid) * dy_local;

	// Increment position and angle by calculated changes
	pos_x += del_x;
	pos_y += del_y;

	prevTheta = theta;
}

double Autonomous::getYaw(void) {
    pros::quaternion_s_t qt = imu.get_quaternion();

    // If encounter IMU failure, and retry
    if (qt.w == PROS_ERR_F) {
        qt = imu.get_quaternion();
        if (qt.w == PROS_ERR_F) {
            return -1.0;
        }
    }

    // yaw formula = atan2(2(wz + xy), 1 - 2(y^2 + z^2))
	double yaw = atan2(2 * ((qt.w * qt.z) + (qt.x * qt.y)), 1 - (2 * ((qt.y * qt.y) + (qt.z * qt.z))));

	// returns yaw converted from rad to deg; angle is returned from -180 to 180 (+ 180 for [0, 360])
	return ((yaw * (180 / M_PI)) + 180);
}

WheelLengths Autonomous::getOdomWheelTravel(void) {
    // Get Initial Wheel Position
    static double lastParallel = parallelTrackingWheel.get_position();
	static double lastPerpendicular = perpendicularTrackingWheel.get_position();

    // Get current centidegree position of tracking wheels
	int currParallel = parallelTrackingWheel.get_position();
	int currPerpendicular = perpendicularTrackingWheel.get_position();

    // Get delta between current and last frame
	double dTicksL = currParallel - lastParallel; 
	double dTicksS = currPerpendicular - lastPerpendicular;


	// Convert centidegrees to degrees and find distance travelled by wheel
	double delParallel = (dTicksL / 36000.0) * PARALLEL_TRACKING_WHEEL_DIAMETER * std::numbers::pi; 
	double delPerpendicular = (dTicksS / 36000.0) * PERPENDICULAR_TRACKING_WHEEL_DIAMETER * std::numbers::pi; 

    // Save current position as previous
	lastParallel = currParallel;
	lastPerpendicular = currPerpendicular;

	// Create a structure of lengths
	WheelLengths del(delParallel, delPerpendicular);

	return del;
}

// ====== Helper functions ======

// Normalize angle between [-180, 180]
static double normalizeAngle(double a) {
    return std::atan2(std::sin(a), std::cos(a));
}

// Return Euclidean distance btwn points p1 and p2
static double getDistance(Position p1, Position p2) {
    return std::sqrt(std::pow((p2.x - p1.x), 2) + std::pow((p2.y - p1.y), 2));
}

// Convert Degrees to Radians
static double convertDegToRad(double degree) {
    return degree * (std::numbers::pi / 180.0);
}

// Covert Radians to Degrees
static double convertRadToDeg(double rad) {
    return rad * (180.0 / M_PI);
}

// Subtract angles with [-180, 180] wrapping
static double angleDiffDeg(double a, double b) {
    double c = a - b;
	while (c > 180.0) c -= 360.0;
	while (c <= -180.0) c += 360.0;
	return c;
}