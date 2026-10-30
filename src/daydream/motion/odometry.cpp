#include "daydream/motion/odometry.hpp"
#include "daydream/config/constants.h"
#include "daydream/config/subsystems.hpp"
#include "daydream/utils/helpers.hpp"
#include <cmath>
#include <numbers>
#include <vector>
#include <atomic>

Odometry::Odometry(OdomConfig config) : m_config(config) {}

void Odometry::updatePose(void) {
	YawResult yawResult = getYaw();

	if (yawResult.error != OdomError::NONE) {
		m_lastOdomError = yawResult.error;
		static uint32_t lastImuFailurePrint = 0;
		const uint32_t now = pros::millis();
		if (now - lastImuFailurePrint >= 500) {
			pros::lcd::print(0, "[Update Pose] IMU Failure! %lf", yawResult.yaw);
			lastImuFailurePrint = now;
		}
		return;
	}

	const double yaw_deg = yawResult.yaw;
	
	// Get orientation from IMU
	double theta_rad = yaw_deg;
	theta_rad = normalizeAngle(theta_rad);
	
	if (!m_initialized) {
		m_prevTheta = theta_rad;
		m_lastRawTheta = theta_rad;

		if (m_config.useMotorEncoders) {
			m_prevLeft = averageMotorGroupPosition(leftMotors);
			m_prevRight = averageMotorGroupPosition(rightMotors);
		} else {
			m_prevParallel = parallelTrackingWheel.get_position();
			m_prevPerpendicular = perpendicularTrackingWheel.get_position();

			if (m_prevParallel == PROS_ERR || m_prevPerpendicular == PROS_ERR) {
                // TODO: Handle error for tracking wheel initialization failure
                // FIX: add file wide error variable to track odom errors and display on controller
                // The file wide error variable should be accessible to other files through a public getter
                m_lastOdomError = OdomError::TRACKING_WHEEL_ERROR;
				return;
			}
		}

		m_lastOdomError = m_usingIMEFallback ? OdomError::IMU_FALLBACK : OdomError::NONE;
		m_initialized = true;
		return;
	}

	// Calculate distance travelled by either drive encoders or tracking wheels
	WheelTravelResult wheelResult;
	if (m_config.useMotorEncoders) {
		wheelResult = getDriveEncoderTravel();
	} else {
		wheelResult = getOdomWheelTravel();
	}

	if (wheelResult.error != OdomError::NONE) {
		m_lastOdomError = wheelResult.error;
		pros::lcd::print(0, "[Update Pose] Tracking Wheel Error!");
		return;
	}

	WheelLengths arcs = wheelResult.travel;

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



	// Increment position and angle by calculated changes
	m_mutex.take();
	double theta_mid = normalizeAngle(m_prevTheta + m_headingOffset + del_theta / 2.0);

    // Compute change in x and y based on heading and local changes
	double del_x = std::cos(theta_mid) * dx_local - std::sin(theta_mid) * dy_local;
	double del_y = std::sin(theta_mid) * dx_local + std::cos(theta_mid) * dy_local;
	m_currentPosition.x += del_x;
	m_currentPosition.y += del_y;
	m_currentPosition.theta = normalizeAngle(theta_rad + m_headingOffset);
	m_lastRawTheta = theta_rad;
	m_mutex.give();

	m_prevTheta = theta_rad;

	// !IMPORTANT!: Likely not task safe
	// print to the controller every 100 ms 
	// static uint32_t lastPrintTime = 0;
	// if (pros::millis() - lastPrintTime > 100) {
	// 	controller.print(0, 0, "X:%.1f Y:%.1f", m_currentPosition.x, m_currentPosition.y);
	// 	lastPrintTime = pros::millis();
	// }

	m_prevTheta = theta_rad;

    m_lastOdomError = m_usingIMEFallback ? OdomError::IMU_FALLBACK : OdomError::NONE;
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
	m_currentPosition.theta = normalizeAngle(pose.theta);
    m_headingOffset = normalizeAngle(pose.theta - m_lastRawTheta);
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

OdomError Odometry::getOdomError() {
	return m_lastOdomError.load();
}

double Odometry::getEncoderHeading() {
    // Get average left and right motor encoder positions.
    double leftTicks = averageMotorGroupPosition(leftMotors);
    double rightTicks = averageMotorGroupPosition(rightMotors);

    // Reject invalid encoder readings.
    if (!std::isfinite(leftTicks) ||
        !std::isfinite(rightTicks)) {
        return NAN;
    }

    // Validate the configured wheel diameter and track width.
    if (!std::isfinite(m_config.driveWheelDiameter) ||
        m_config.driveWheelDiameter <= 0.0 ||
        TRACK_WIDTH <= 0.0) {
        return NAN;
    }

    // Convert encoder degrees into wheel travel in inches.
    const double wheelCircumference =
        m_config.driveWheelDiameter * std::numbers::pi;

    const double leftInches =
        (leftTicks / 360.0) *
        DRIVE_GEAR_RATIO *
        wheelCircumference;

    const double rightInches =
        (rightTicks / 360.0) *
        DRIVE_GEAR_RATIO *
        wheelCircumference;

    // Calculate heading in radians.
    const double heading = (rightInches - leftInches) / TRACK_WIDTH;
    return heading;
}


YawResult Odometry::getIMEYaw(OdomError imuError) {
	double encHeading = getEncoderHeading();
	if (std::isnan(encHeading)) {
		return {0.0, imuError};
	}

	m_mutex.take();
	double yaw = m_imuRefYaw + (encHeading - m_encRefHeading);
	m_mutex.give();

	double normalizedYaw = normalizeAngle(yaw);
	return {normalizedYaw, OdomError::NONE};
}

YawResult Odometry::getIMUYaw(void)
{

	// imu disconnected
	if (!imu.is_installed()) {
		return {0.0, OdomError::IMU_DISCONNECTED};
	}

	// imu still calibrating
	if (imu.is_calibrating()) {
		return {0.0, OdomError::IMU_CALIBRATING};
	}

	pros::quaternion_s_t qt = imu.get_quaternion();
	
	// If encounter IMU failure, and retry
	if (std::isnan(qt.w) || qt.w == PROS_ERR_F) {
		qt = imu.get_quaternion();
		// pros comm error
		if (std::isnan(qt.w) || qt.w == PROS_ERR_F) return {0.0, OdomError::IMU_COMMUNICATION_ERROR};
	}

	// yaw formula = atan2(2(wz + xy), 1 - 2(y^2 + z^2))
	double yaw_rad = std::atan2(2 * ((qt.w * qt.z) + (qt.x * qt.y)), 1 - (2 * ((qt.y * qt.y) + (qt.z * qt.z))));

	// IMU is clockwise positive, odom is CCW positive
	double ccwYaw = -yaw_rad;
	return {ccwYaw, OdomError::NONE};}


YawResult Odometry::getYaw(void) {
	YawResult imuResult = getIMUYaw();

	if (imuResult.error == OdomError::NONE) {
		double encHeading = getEncoderHeading();

		m_mutex.take();
		if (m_usingIMEFallback) {
			// IMU is back and was likely re-zeroed by recalibrating, rebase it onto the fallback heading so yaw doesn't jump
			m_imuYawOffset = normalizeAngle(m_lastGoodYaw - imuResult.yaw);
			m_usingIMEFallback = false;
		}
		double yaw = normalizeAngle(imuResult.yaw + m_imuYawOffset);

		// save reference so IME fallback continues from here without a jump
		if (!std::isnan(encHeading)) {
			m_imuRefYaw = yaw;
			m_encRefHeading = encHeading;
		}
		m_lastGoodYaw = yaw;
		m_mutex.give();

		return {yaw, OdomError::NONE};
	}

	// keep using the encoders while the IMU recalibrates after a reconnect
	bool recalibratingAfterDisconnect = imuResult.error == OdomError::IMU_CALIBRATING && m_usingIMEFallback;

	if (imuResult.error == OdomError::IMU_DISCONNECTED || recalibratingAfterDisconnect) {
		m_usingIMEFallback = true;

		YawResult imeResult = getIMEYaw(imuResult.error);
		if (imeResult.error == OdomError::NONE) {
			m_mutex.take();
			m_lastGoodYaw = imeResult.yaw;
			m_mutex.give();
		}
		return imeResult;
	}

	// calibrating / communication error: report it, don't guess
	return imuResult;
}

WheelTravelResult Odometry::getOdomWheelTravel(void) {
	if (!m_initialized) {
		return {{0.0, 0.0}, OdomError::NONE};
	}

    // Get current centidegree position of tracking wheels
	double currParallel = parallelTrackingWheel.get_position();
	double currPerpendicular = perpendicularTrackingWheel.get_position();

	// Returns error for problem with either wheel
	if (currParallel == PROS_ERR || currPerpendicular == PROS_ERR) {
		return {{0.0, 0.0}, OdomError::TRACKING_WHEEL_ERROR};
	}

    // Get delta between current and last frame 
	double dTicksL = currParallel - m_prevParallel; 
	double dTicksS = currPerpendicular - m_prevPerpendicular;

	// Convert centidegrees to degrees and find distance travelled by wheel
	double delParallel = (dTicksL / 36000.0) * m_config.parallelWheelDiameter * std::numbers::pi; 
	double delPerpendicular = (dTicksS / 36000.0) * m_config.perpendicularWheelDiameter * std::numbers::pi; 

    // Save current position as previous
	m_prevParallel = currParallel;
	m_prevPerpendicular = currPerpendicular;

	return {{delParallel, delPerpendicular}, OdomError::NONE};
}

VelocityResult Odometry::getParallelVel() {
	double initialVelocity = parallelTrackingWheel.get_velocity();

	if (m_config.useMotorEncoders){

		double leftVelocity = averageMotorGroupVelocities(leftMotors);
		double rightVelocity = averageMotorGroupVelocities(rightMotors);

		if (leftVelocity == PROS_ERR_F || rightVelocity == PROS_ERR_F) {
			return {0.0, OdomError::TRACKING_WHEEL_ERROR};
		}

		// motor velocities are in RPM
		double averageRpm = (leftVelocity + rightVelocity) / 2;

		double velocity  = averageRpm * DRIVE_GEAR_RATIO * (m_config.driveWheelDiameter / 2) * (2 * std::numbers::pi / 60);
		
		return {velocity, OdomError::NONE};
	}

	if (initialVelocity == PROS_ERR) {
		return {0.0, OdomError::TRACKING_WHEEL_ERROR};
	}

	// rotation sensor velocity is in centidegrees per second
	double deg_s = initialVelocity / 100.0;

	double velocity = deg_s * (std::numbers::pi / 180) * (m_config.parallelWheelDiameter / 2);

	return {velocity, OdomError::NONE};
}

WheelTravelResult Odometry::getDriveEncoderTravel(void) {
	if (!m_initialized) {
		return {{0, 0}, OdomError::NONE};
	}

    double currLeft = averageMotorGroupPosition(leftMotors);
    double currRight = averageMotorGroupPosition(rightMotors);

	if (std::isnan(currLeft) || std::isnan(currRight)) {
		return {{0, 0}, OdomError::TRACKING_WHEEL_ERROR};
	}

	if (std::isnan(m_prevLeft) || std::isnan(m_prevRight)) {
		m_prevLeft = currLeft;
		m_prevRight = currRight;
		return {{0, 0}, OdomError::NONE};
	}

    double dLeft = currLeft - m_prevLeft;
    double dRight = currRight - m_prevRight;

    double leftInches = (dLeft / 360.0) *DRIVE_GEAR_RATIO  * m_config.driveWheelDiameter * std::numbers::pi;
    double rightInches = (dRight / 360.0) * DRIVE_GEAR_RATIO * m_config.driveWheelDiameter * std::numbers::pi;

    m_prevLeft = currLeft;
    m_prevRight = currRight;

    double forwardInches = (leftInches + rightInches) / 2.0;
    return {{forwardInches, 0.0}, OdomError::NONE};
}

double Odometry::averageMotorGroupPosition(const pros::MotorGroup& group) {
    std::vector<double> positions = group.get_position_all();

    if (positions.empty()) {
        return NAN;
    }

    double sum = 0.0;
    int valid = 0;

    for (double pos : positions) {
        if (pos == PROS_ERR_F || !std::isfinite(pos)) {
            continue;
        }
        sum += pos;
        valid++;
    }

    if (valid == 0) {
        return NAN;
    }

    const double average = sum / static_cast<double>(valid);
    return average;
}

double Odometry::averageMotorGroupVelocities(const pros::MotorGroup& group){
	std::vector<double> velocities = group.get_actual_velocity_all();
	if(velocities.empty()){
		return PROS_ERR_F;
	}

	double sum = 0.0;

	for(auto vel : velocities){
		if(vel == PROS_ERR_F){
			return PROS_ERR_F;
		}
		sum += vel;
	}
	return sum / static_cast<double>(velocities.size());
}

// Background odom loop, started once in initialize(). Runs every 10 ms on a fixed schedule
// (delay_until), independent of PathFollower::step(), so pose stays current in every mode.
void Odometry::odomTask() {
	std::uint32_t now = pros::millis();
	while (true) {
		odom.updatePose();
		pros::Task::delay_until(&now, 10);
	}
}

Odometry odom({
	PARALLEL_TRACKING_WHEEL_DIAMETER,
	PERPENDICULAR_TRACKING_WHEEL_DIAMETER,
	PARALLEL_TRACKING_WHEEL_OFFSET,
	PERPENDICULAR_TRACKING_WHEEL_OFFSET,
	true,
	DRIVE_WHEEL_DIAMETER_INCHES
});
