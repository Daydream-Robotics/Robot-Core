#include "daydream/motion/odometry.hpp"
#include "daydream/config/constants.h"
#include "daydream/config/subsystems.hpp"
#include "daydream/utils/helpers.hpp"
#include <cmath>
#include <numbers>
#include <vector>
#include <optional>

struct MotorGroupPositionSample {
	double average{0.0};
	std::vector<std::optional<double>> motorPositions;
};

static std::optional<MotorGroupPositionSample>
readMotorGroupSample(const pros::MotorGroup& group) {

	std::vector<double> positions = group.get_position_all();

	if (positions.empty()) {
		return std::nullopt;
	}

	MotorGroupPositionSample sample;
	double sum = 0.0;
	size_t validCount = 0;

	for (double pos : positions) {

		if (!std::isfinite(pos)) {
			sample.motorPositions.push_back(std::nullopt);
			continue;
		}

		sample.motorPositions.push_back(pos);
		sum += pos;
		validCount++;
	}

	if (validCount == 0) {
		return std::nullopt;
	}

	sample.average = sum / static_cast<double>(validCount);

	return sample;
}

static std::optional<double> averageMotorDelta(
	const std::vector<std::optional<double>>& previous,
	const std::vector<std::optional<double>>& current) {

		// The motor groups must have matching sizes
		if (previous.size() != current.size()) {
			return std::nullopt;
		}

		double sum = 0.0;
		size_t validCount = 0;

		for (size_t i = 0; i < current.size(); i++) {
			// Only compare motors with two valid readings
			if (!previous[i].has_value() || !current[i].has_value()) {
				continue;
			}

			sum += current[i].value() - previous[i].value();
			validCount++;
		}

		// No motors could be compared
		if (validCount == 0) {
			return std::nullopt;
		}

		return sum / static_cast<double>(validCount);
	}

static std::optional<double> averageMotorGroupPosition(const pros::MotorGroup& group) {
    std::vector<double> positions = group.get_position_all();
    if (positions.empty()) {
        return std::nullopt;
    }

    double sum = 0.0;
	size_t validCount = 0;

    for (double pos : positions) {

		// Skip invalid motor readings
        if (!std::isfinite(pos)) {
			continue;
		}

		sum += pos;
		validCount++;
    }

	// No usable motors
	if (validCount == 0) {
		return std::nullopt;
	}

    return sum / static_cast<double>(validCount);
}

Odometry::Odometry(OdomConfig config) : m_config(config) {}


void Odometry::updatePose(void) {
	// Read motor encoders every update as IMU backup
	DriveTravelResult motorResult = getDriveEncoderTravel();

	if (m_config.useMotorEncoders &&
		motorResult.error != OdomError::NONE) {

		static uint32_t lastMotorFailurePrint = 0;
		const uint32_t now = pros::millis();

		if (now - lastMotorFailurePrint >= 500) {
			pros::lcd::print(0, "[Update Pose] Motor Encoder Error!");
			lastMotorFailurePrint = now;
		}

		m_lastOdomError = OdomError::MOTOR_ENCODER_ERROR;
		return;
	}

	// Check tracking wheels for if they are the movement source
	WheelTravelResult wheelResult;

	if (!m_config.useMotorEncoders) {
		wheelResult = getOdomWheelTravel();

		if (wheelResult.error != OdomError::NONE) {
			m_lastOdomError = wheelResult.error;
			pros::lcd::print(0, "[Update Pose] Tracking Wheel Error!");
			return;
		}
	}

	double encoderHeading = 0.0;

	bool encoderHeadingAvailable =
		motorResult.error == OdomError::NONE &&
		m_driveEncodersReady &&
		std::isfinite(m_config.driveTrackWidth) &&
		m_config.driveTrackWidth > 0.0;

	if (encoderHeadingAvailable) {
		encoderHeading =
			(motorResult.leftInches - motorResult.rightInches)
			/ m_config.driveTrackWidth;
	}

	YawResult yawResult = getIMUYaw();

	if (yawResult.error != OdomError::NONE) {

		// IMU may have lost its original heading reference
		if (yawResult.error != OdomError::IMU_COMMUNICATION_ERROR) {
			m_imuNeedsRebase = true;
		}

		static uint32_t lastImuFailurePrint = 0;
		const uint32_t now = pros::millis();
		if (now - lastImuFailurePrint >= 500) {
			pros::lcd::print(0, "[Update Pose] IMU Failure! %lf", yawResult.yaw);
			lastImuFailurePrint = now;
		}
		if (!encoderHeadingAvailable && m_initialized) {
			m_lastOdomError = yawResult.error;
			return;
		}
	}

	double theta_rad;

	if (yawResult.error == OdomError::NONE) {
		theta_rad = normalizeAngle(yawResult.yaw);

		if (m_usingEncoderHeading && m_initialized) {
			// Estimate movement during this final encoder interval
			double recoveryDelta = encoderHeadingAvailable
				? encoderHeading
				: 0.0;

			double estimatedTheta =
				normalizeAngle(m_prevTheta + recoveryDelta);

			if (m_imuNeedsRebase) {
				m_mutex.take();

				m_headingOffset = normalizeAngle(
					m_headingOffset + normalizeAngle(estimatedTheta - theta_rad)
				);

				m_mutex.give();
			}

			// Preserve the actual rotation during this update
			m_prevTheta = normalizeAngle(theta_rad - recoveryDelta);

			m_usingEncoderHeading = false;
		}

		m_imuNeedsRebase = false;

	} else {
		m_usingEncoderHeading = true;
		theta_rad = normalizeAngle(m_prevTheta + encoderHeading);
	}
	
	if (!m_initialized) {
		m_prevTheta = theta_rad;
		m_lastRawTheta = theta_rad;

		if (m_config.useMotorEncoders) {
			auto leftSample = readMotorGroupSample(leftMotors);
			auto rightSample = readMotorGroupSample(rightMotors);

			if (!leftSample.has_value() || !rightSample.has_value()) {
				static uint32_t lastInitMotorFailurePrint = 0;
				const uint32_t now = pros::millis();

				if (now - lastInitMotorFailurePrint >= 500) {
					pros::lcd::print(0, "[Update Pose] Motor Encoder Error!");
					lastInitMotorFailurePrint = now;
				}

				m_lastOdomError = OdomError::MOTOR_ENCODER_ERROR;
				return;
			}

			// Save existing group averages
			m_prevLeft = leftSample->average;
			m_prevRight = rightSample->average;

			// Save each motor's individual starting position
			m_prevLeftMotorPositions = leftSample->motorPositions;
			m_prevRightMotorPositions = rightSample->motorPositions;

			m_driveEncodersReady = true;

		} else {
			m_prevParallel = parallelTrackingWheel.get_position();
			m_prevPerpendicular = perpendicularTrackingWheel.get_position();

			if (m_prevParallel == PROS_ERR || m_prevPerpendicular == PROS_ERR) {
				m_lastOdomError = OdomError::TRACKING_WHEEL_ERROR;
				return;
			}

			auto leftSample = readMotorGroupSample(leftMotors);
			auto rightSample = readMotorGroupSample(rightMotors);

			if (leftSample.has_value() && rightSample.has_value()) {
				m_prevLeft = leftSample->average;
				m_prevRight = rightSample->average;

				m_prevLeftMotorPositions = leftSample->motorPositions;
				m_prevRightMotorPositions = rightSample->motorPositions;

				m_driveEncodersReady = true;
			} else {
				m_prevLeftMotorPositions.clear();
				m_prevRightMotorPositions.clear();
				m_driveEncodersReady = false;
			}
		}

		if (yawResult.error != OdomError::NONE &&
			(!m_driveEncodersReady ||
			!std::isfinite(m_config.driveTrackWidth) ||
			m_config.driveTrackWidth <= 0.0)) {

			pros::lcd::print(0, "[Update Pose] No valid heading source!");
			m_lastOdomError = yawResult.error;
			return;
		}

		m_usingEncoderHeading = (yawResult.error != OdomError::NONE);
		m_lastOdomError = m_usingEncoderHeading ? OdomError::IMU_FALLBACK : OdomError::NONE;
		m_initialized = true;
		return;
	}

	// Calculate distance travelled by either drive encoders or tracking wheels
	WheelLengths arcs;
	
	if (m_config.useMotorEncoders) {
		double forwardInches = (motorResult.leftInches + motorResult.rightInches) / 2.0;
		arcs = {forwardInches, 0.0};
	}	
	else {
		arcs = wheelResult.travel;
	}
	
	// Determine change in heading 
	double del_theta = normalizeAngle(theta_rad - m_prevTheta);

	// Determine change in local x and in local y
	double dx_local = arcs.parallel;
	double dy_local = arcs.perpendicular;

	if (!m_config.useMotorEncoders) {
		dx_local -= del_theta * m_config.parallelTrackingWheelOffset;
		dy_local += del_theta * m_config.perpendicularTrackingWheelOffset;
	}

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

	// Save encoder positions only after updating the pose
	if (motorResult.error == OdomError::NONE && m_driveEncodersReady) {
		m_prevLeft = motorResult.currentLeftPosition;
		m_prevRight = motorResult.currentRightPosition;

		// Save each motor's position for the next update
		m_prevLeftMotorPositions = motorResult.currentLeftMotorPositions;
		m_prevRightMotorPositions = motorResult.currentRightMotorPositions;
	}

	// Save tracking wheel positions only after updating the pose
	if (!m_config.useMotorEncoders) {
		m_prevParallel = wheelResult.currentParallelPosition;
		m_prevPerpendicular = wheelResult.currentPerpendicularPosition;
	}

	m_prevTheta = theta_rad;

	m_lastOdomError = m_usingEncoderHeading ? OdomError::IMU_FALLBACK : OdomError::NONE;

	// !IMPORTANT!: Likely not task safe
	// print to the controller every 100 ms 
	// static uint32_t lastPrintTime = 0;
	// if (pros::millis() - lastPrintTime > 100) {
	// 	controller.print(0, 0, "X:%.1f Y:%.1f", m_currentPosition.x, m_currentPosition.y);
	// 	lastPrintTime = pros::millis();
	// }
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

YawResult Odometry::getYaw(void) {
	YawResult imuResult = getIMUYaw();

	if (imuResult.error == OdomError::NONE || !m_usingEncoderHeading) {
		return imuResult;
	}

	// IMU is unavailable, use the encoder heading updatePose() is tracking
	m_mutex.take();
	double encoderYaw = m_lastRawTheta;
	m_mutex.give();

	return {encoderYaw, OdomError::NONE};
}

YawResult Odometry::getIMUYaw(void) {

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

	// angle is returned from -pi to pi
	return {-yaw_rad, OdomError::NONE};
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

	return {{delParallel, delPerpendicular}, OdomError::NONE, currParallel, currPerpendicular};
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

DriveTravelResult Odometry::getDriveEncoderTravel(void) {
	if (!m_initialized) {
		return {0.0, 0.0, OdomError::NONE};
	}

    auto leftSample = readMotorGroupSample(leftMotors);
    auto rightSample = readMotorGroupSample(rightMotors);

	if (!leftSample.has_value() || !rightSample.has_value()) {
		return {0.0, 0.0, OdomError::MOTOR_ENCODER_ERROR};
	}

	double currLeft = leftSample->average;
	double currRight = rightSample->average;

	if (!m_driveEncodersReady) {
		m_prevLeft = currLeft;
		m_prevRight = currRight;

		// Establish individual motor baselines
		m_prevLeftMotorPositions = leftSample->motorPositions;
		m_prevRightMotorPositions = rightSample->motorPositions;

		m_driveEncodersReady = true;
		return {0.0, 0.0, OdomError::NONE, currLeft, currRight, leftSample->motorPositions, rightSample->motorPositions};
	}

    auto leftDelta = averageMotorDelta(
		m_prevLeftMotorPositions,
		leftSample->motorPositions
	);

	auto rightDelta = averageMotorDelta(
		m_prevRightMotorPositions,
		rightSample->motorPositions
	);

	if (!leftDelta.has_value() || !rightDelta.has_value()) {
		// No comparable motor readings on at least one side
		// Re-establish baselines on next update
		m_driveEncodersReady = false;

		return {0.0, 0.0, OdomError::MOTOR_ENCODER_ERROR};
	}

	double dLeft = leftDelta.value();
	double dRight = rightDelta.value();

    double leftInches = (dLeft / 360.0) *DRIVE_GEAR_RATIO  * m_config.driveWheelDiameter * std::numbers::pi;
    double rightInches = (dRight / 360.0) * DRIVE_GEAR_RATIO * m_config.driveWheelDiameter * std::numbers::pi;

    return {leftInches, rightInches, OdomError::NONE, currLeft, currRight, leftSample->motorPositions, rightSample->motorPositions};
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

// ! TAI: move this to the new tuning file
void Odometry::calculateTrackingWheelOffsets() {

	// Calibration settings
	const int TURN_SPEED = 120;
	const double TARGET_ROTATION = 4.0 * std::numbers::pi;

	// Variables for calibration timeout
	const uint32_t START_TIME = pros::millis();
	const uint32_t TIMEOUT = 30000; // 30 seconds total for both spins

	// Variables for running sums
	double sumParallelTheta = 0.0;
	double sumPerpendicularTheta = 0.0;
	double sumThetaSquared = 0.0;

	for (int direction : {1, -1}) {

		// Record starting tracking wheel positions
		double startParallel = parallelTrackingWheel.get_position();
		double startPerpendicular = perpendicularTrackingWheel.get_position();

		// Check for tracking wheel sensor errors
		if (startParallel == PROS_ERR || startPerpendicular == PROS_ERR) {
			pros::lcd::print(0, "Calibration: Tracking wheel error!");
			return;
		}

		// Set up IMU rotation tracking
		YawResult initialYaw = getIMUYaw();

		if (initialYaw.error != OdomError::NONE) {
			pros::lcd::print(0, "Calibration: IMU error!");
			return;
		}

		double previousYaw = initialYaw.yaw;
		double totalRotation = 0.0;

		leftMotors.move_velocity(TURN_SPEED * direction);
		rightMotors.move_velocity(-TURN_SPEED * direction);

		while (std::abs(totalRotation) < TARGET_ROTATION) {

			// Stop calibration if the time limit is exceeded
			if (pros::millis() - START_TIME >= TIMEOUT) {
				leftMotors.move_velocity(0);
				rightMotors.move_velocity(0);
				pros::lcd::print(0, "Calibration timeout!");
				return;
			}

			YawResult result = getIMUYaw();

			if (result.error != OdomError::NONE) {
				leftMotors.move_velocity(0);
				rightMotors.move_velocity(0);
				return;
			}

			double currentYaw = result.yaw;

			double deltaYaw = angleDiffRad(currentYaw, previousYaw);

			totalRotation += deltaYaw;
			previousYaw = currentYaw;

			pros::delay(10);
		}

		// Stop the robot
		leftMotors.move_velocity(0);
		rightMotors.move_velocity(0);

		// Allow robot to settle before calculation
		pros::delay(500);

		// Account for any additional rotation while settling
		YawResult finalYaw = getIMUYaw();

		if (finalYaw.error != OdomError::NONE) {
			pros::lcd::print(0, "Calibration: IMU error!");
			return;
		}

		double finalDeltaYaw = angleDiffRad(finalYaw.yaw, previousYaw);

		totalRotation += finalDeltaYaw;

		// Get final tracking wheel positions
		double endParallel = parallelTrackingWheel.get_position();
		double endPerpendicular = perpendicularTrackingWheel.get_position();

		if (endParallel == PROS_ERR || endPerpendicular == PROS_ERR) {
			pros::lcd::print(0, "Calibration: Tracking wheel error!");
			return;
		}

		// Calculate change in sensor positions
		double parallelDelta = endParallel - startParallel;
		double perpendicularDelta = endPerpendicular - startPerpendicular;

		// Convert sensor positions to inches
		double parallelDistance = (parallelDelta / 36000.0) * std::numbers::pi * m_config.parallelWheelDiameter;
		double perpendicularDistance = (perpendicularDelta / 36000.0) * std::numbers::pi * m_config.perpendicularWheelDiameter;

		// Prevent division by invalid/near-zero rotation
		if (!std::isfinite(totalRotation) || std::abs(totalRotation) < 1e-6 ||
			!std::isfinite(parallelDistance) || !std::isfinite(perpendicularDistance)) {
			pros::lcd::print(0, "Calibration: Invalid Measurements");
			return;
		}

		// Add measurements from both spin directions
		sumParallelTheta += parallelDistance * totalRotation;
		sumPerpendicularTheta += perpendicularDistance * totalRotation;
		sumThetaSquared += totalRotation * totalRotation;

	}

	// Validate accumulated measurements before calculating offsets
	if (!std::isfinite(sumThetaSquared) || sumThetaSquared < 1e-9 ||
		!std::isfinite(sumParallelTheta) || !std::isfinite(sumPerpendicularTheta)) {
		pros::lcd::print(0, "Calibration: Invalid Measurements");
		return;
	}

	// Calculate signed tracking wheel offsets
	double parallelOffset =  sumParallelTheta / sumThetaSquared;
	double perpendicularOffset = -sumPerpendicularTheta / sumThetaSquared;

	// Display calculated tracking wheel offsets (testing)
	pros::lcd::print(0, "Parallel Offset: %.3f in", parallelOffset);
	pros::lcd::print(1, "Perpendicular Offset: %.3f in", perpendicularOffset);
}

Odometry odom({
	PARALLEL_TRACKING_WHEEL_DIAMETER,
	PERPENDICULAR_TRACKING_WHEEL_DIAMETER,
	PARALLEL_TRACKING_WHEEL_OFFSET,
	PERPENDICULAR_TRACKING_WHEEL_OFFSET,
	true,
	DRIVE_WHEEL_DIAMETER_INCHES,
	TRACK_WIDTH
});
