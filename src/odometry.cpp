#include "odometry.hpp"
#include "subsystems.hpp"
#include "helpers.hpp"
#include <cmath>
#include <numbers>

void Odometry::updatePose(void) {
	// Calculate distance travelled by each tracking wheel
	WheelLengths arcs = getOdomWheelTravel();

	const double yaw = getYaw();

	if (yaw < -180.0) {
		pros::lcd::print(0, "[Update Pose] IMU Failure! %lf", yaw);
		return;
	}

	static double prevTheta = convertDegToRad(yaw);

	// Get orientation from IMU
	double theta_rad = convertDegToRad(yaw);
	theta_rad = normalizeAngle(theta_rad);

	// Determine change in heading 
	double del_theta = normalizeAngle(theta_rad - prevTheta);

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

	// print to the controller every 100 ms
	static uint32_t lastPrintTime = 0;
	if (pros::millis() - lastPrintTime > 100) {
		// controller.print(0, 0, "X:%.1f Y:%.1f", pos_x, pos_y);
		lastPrintTime = pros::millis();
	}

	prevTheta = theta_rad;
}

double Odometry::getYaw(void) {

	// imu disconnected
	if (!imu.is_installed()) {
		return -180.1;
	}

	// imu still calibrating
	if (imu.is_calibrating()) {
		return -180.2;
	}

    pros::quaternion_s_t qt = imu.get_quaternion();
	
    // If encounter IMU failure, and retry
    if (std::isnan(qt.w) || qt.w == PROS_ERR_F) {
        qt = imu.get_quaternion();
		// pros comm error
        if (std::isnan(qt.w) || qt.w == PROS_ERR_F) return -180.3;
    }

    // yaw formula = atan2(2(wz + xy), 1 - 2(y^2 + z^2))
	double yaw_rad = std::atan2(2 * ((qt.w * qt.z) + (qt.x * qt.y)), 1 - (2 * ((qt.y * qt.y) + (qt.z * qt.z))));

	// convert to degrees
	double yaw_deg = yaw_rad * (180.0 / std::numbers::pi);

	// angle is returned from -180 to 180
	return -yaw_deg;
}

double Odometry::getPosX() {
    return pos_x;
}

double Odometry::getPosY() {
    return pos_y;
}

WheelLengths Odometry::getOdomWheelTravel(void) {
    // Get Initial Wheel Position
    static double lastParallel = parallelTrackingWheel.get_position();
	static double lastPerpendicular = perpendicularTrackingWheel.get_position();


	// pros::delay(100);

    // Get current centidegree position of tracking wheels
	double currParallel = parallelTrackingWheel.get_position();
	double currPerpendicular = perpendicularTrackingWheel.get_position();

	// controller.print(0,0, "%d", currParallel);
    // Get delta between current and last frame
	double dTicksL = currParallel - lastParallel; 
	double dTicksS = currPerpendicular - lastPerpendicular;


	// Convert centidegrees to degrees and find distance travelled by wheel
	double delParallel = (dTicksL / 36000.0) * PARALLEL_TRACKING_WHEEL_DIAMETER * std::numbers::pi; 
	double delPerpendicular = (dTicksS / 36000.0) * PERPENDICULAR_TRACKING_WHEEL_DIAMETER * std::numbers::pi; 

    // Save current position as previous
	lastParallel = currParallel;
	lastPerpendicular = currPerpendicular;
	// controller.print(0, 0, "O: %.2f\n", delParallel);
	// Create a structure of lengths
	WheelLengths del(delParallel, delPerpendicular);
	

	return del;
}

void Odometry::setPose(double x, double y) {
	pos_x = x;
	pos_y = y;
}

double Odometry::getParallelVel() {
	double deg_s = parallelTrackingWheel.get_velocity();
	return (deg_s / 360.0) * PARALLEL_TRACKING_WHEEL_DIAMETER * std::numbers::pi;
}

// could be used to do odom in background
void Odometry::odomTask() {
	while (true) {
		odom.updatePose();
		pros::delay(10);
	}
}

Odometry odom;
