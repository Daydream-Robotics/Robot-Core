#include "daydream/motion/odometry.hpp"
#include "daydream/config/constants.h"
#include "daydream/config/subsystems.hpp"
#include "daydream/utils/helpers.hpp"
#include <cmath>
#include <numbers>
#include <vector>

static double averageMotorGroupPosition(const pros::MotorGroup& group) {
    std::vector<double> positions = group.get_position_all();
    if (positions.empty()) {
        return 0.0;
    }

    double sum = 0.0;
    for (double pos : positions) {
        sum += pos;
    }

    return sum / static_cast<double>(positions.size());
}

Odometry::Odometry(OdomConfig config) : m_config(config) {}

void Odometry::updatePose(void) {
	const double yaw_deg = getYaw(); 
	
	if (yaw_deg < -180.0) {
		pros::lcd::print(0, "[Update Pose] IMU Failure! %lf", yaw_deg);
		return;
	}
	
	// Get orientation from IMU
	double theta_rad = convertDegToRad(yaw_deg);
	theta_rad = normalizeAngle(theta_rad);
	
	if (!m_initialized) {
		m_prevTheta = theta_rad;

		if (m_config.useMotorEncoders) {
			m_prevLeft = averageMotorGroupPosition(leftMotors);
			m_prevRight = averageMotorGroupPosition(rightMotors);
		} else {
			m_prevParallel = parallelTrackingWheel.get_position();
			m_prevPerpendicular = perpendicularTrackingWheel.get_position();
		}
		
		m_initialized = true;
		return;
	}

	// Calculate distance travelled by either drive encoders or tracking wheels
	WheelLengths arcs;
	if (m_config.useMotorEncoders) {
		arcs = getDriveEncoderTravel();
	} else {
		arcs = getOdomWheelTravel();
	}
	
	// Determine change in heading 
	double del_theta = normalizeAngle(theta_rad - m_prevTheta);

	// Determine change in local x and in local y
	double dx_local = arcs.parallel - (del_theta * m_config.parallelTrackingWheelOffset);
	double dy_local = arcs.perpendicular + (del_theta * m_config.perpendicularTrackingWheelOffset);

	// Arc-length, chord-length correction
	double chordFactor = 1.0;
	if (std::abs(del_theta) > 1e-9) {
		chordFactor = 2.0 * std::sin(del_theta / 2.0) / del_theta;
	}
	dx_local *= chordFactor;
	dy_local *= chordFactor;

	double theta_mid = m_prevTheta + del_theta / 2.0;
    theta_mid = normalizeAngle(theta_mid);

    // Compute change in x and y based on heading and local changes
	double del_x = std::cos(theta_mid) * dx_local - std::sin(theta_mid) * dy_local;
	double del_y = std::sin(theta_mid) * dx_local + std::cos(theta_mid) * dy_local;

	// Increment position and angle by calculated changes
	m_mutex.take();
	m_currentPosition.x += del_x;
	m_currentPosition.y += del_y;
	m_currentPosition.theta = theta_rad;
	m_mutex.give();

	// !IMPORTANT!: Likely not task safe
	// print to the controller every 100 ms 
	// static uint32_t lastPrintTime = 0;
	// if (pros::millis() - lastPrintTime > 100) {
	// 	controller.print(0, 0, "X:%.1f Y:%.1f", m_currentPosition.x, m_currentPosition.y);
	// 	lastPrintTime = pros::millis();
	// }

	m_prevTheta = theta_rad;
}

Pose Odometry::getPose() {
	m_mutex.take();
    Pose pose = m_currentPosition;
    m_mutex.give();
    return pose;
}

void Odometry::setPose(Pose pose) {
	m_mutex.take();
	m_currentPosition.x = pose.x;
	m_currentPosition.y = pose.y;
	m_currentPosition.theta = pose.theta;
	m_mutex.give();
}

Position Odometry::getPosition() {
	m_mutex.take();
	Position pos = {m_currentPosition.x, m_currentPosition.y};
	m_mutex.give();
	return pos;
}

double Odometry::getPosX() {
	m_mutex.take();
    double x = m_currentPosition.x;
    m_mutex.give();
    return x;
}

double Odometry::getPosY() {
	m_mutex.take();
    double y = m_currentPosition.y;
    m_mutex.give();
    return y;
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

WheelLengths Odometry::getOdomWheelTravel(void) {
	if (!m_initialized) {
		return {0, 0};
	}

    // Get current centidegree position of tracking wheels
	double currParallel = parallelTrackingWheel.get_position();
	double currPerpendicular = perpendicularTrackingWheel.get_position();

    // Get delta between current and last frame 
	double dTicksL = currParallel - m_prevParallel; 
	double dTicksS = currPerpendicular - m_prevPerpendicular;

	// Convert centidegrees to degrees and find distance travelled by wheel
	double delParallel = (dTicksL / 36000.0) * m_config.parallelWheelDiameter * std::numbers::pi; 
	double delPerpendicular = (dTicksS / 36000.0) * m_config.perpendicularWheelDiameter * std::numbers::pi; 

    // Save current position as previous
	m_prevParallel = currParallel;
	m_prevPerpendicular = currPerpendicular;

	return {delParallel, delPerpendicular};
}

WheelLengths Odometry::getDriveEncoderTravel(void) {
	if (!m_initialized) {
		return {0, 0};
	}

    double currLeft = averageMotorGroupPosition(leftMotors);
    double currRight = averageMotorGroupPosition(rightMotors);

    double dLeft = currLeft - m_prevLeft;
    double dRight = currRight - m_prevRight;

    double leftInches = (dLeft / 360.0) * m_config.driveWheelDiameter * std::numbers::pi;
    double rightInches = (dRight / 360.0) * m_config.driveWheelDiameter * std::numbers::pi;

    m_prevLeft = currLeft;
    m_prevRight = currRight;

    double forwardInches = (leftInches + rightInches) / 2.0;
    return {forwardInches, 0.0};
}

double Odometry::getParallelVel() {
	double deg_s = parallelTrackingWheel.get_velocity() / 100.0;
	return (deg_s / 360.0) * m_config.parallelWheelDiameter * std::numbers::pi;
}

// could be used to do odom in background
void Odometry::odomTask() {
	while (true) {
		odom.updatePose();
		pros::delay(10);
	}
}

Odometry odom({
	PARALLEL_TRACKING_WHEEL_DIAMETER,
	PERPENDICULAR_TRACKING_WHEEL_DIAMETER,
	PARALLEL_TRACKING_WHEEL_OFFSET,
	PERPINDICULAR_TRACKING_WHEEL_OFFSET,
	true,
	DRIVE_WHEEL_DIAMETER_INCHES
});
