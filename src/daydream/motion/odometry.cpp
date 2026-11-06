#include "daydream/motion/odometry.hpp"
#include "daydream/config/constants.h"
#include "daydream/config/subsystems.hpp"
#include "daydream/utils/helpers.hpp"
#include <cmath>
#include <numbers>
#include <vector>

// Average position (deg) of the motors in a group that read successfully.
// Returns PROS_ERR_F if none did.
static double getPositionAllSanitizedAvg(const pros::MotorGroup& group) {
	std::vector<double> positions = group.get_position_all();

	double sum = 0.0;
	int size = 0;
	for (double& pos : positions) {
		if (pos != PROS_ERR_F) {
			sum += pos;
			size++;
		}
	}

	if (size == 0) {
		return PROS_ERR_F;
	}
	return sum / size;
}

// Distance a tracking wheel moved since the last frame, in inches. Updates prev to the current reading.
// A failed read stores PROS_ERR in prev, so the first good read after a reconnect only re-seeds it (0 travel).
static double trackingWheelTravel(double curr, double& prev, double diameter) {
	if (curr == PROS_ERR || prev == PROS_ERR) {
		prev = curr;
		return 0.0;
	}
	// rotation sensor position is in centidegrees
	double travel = ((curr - prev) / 36000.0) * diameter * std::numbers::pi;
	prev = curr;
	return travel;
}

Odometry::Odometry(OdomConfig config) : m_config(config) {}

void Odometry::updatePose(void) {
	YawResult yawResult = getYaw();

	if (yawResult.error != OdomError::NONE) {
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

		m_prevLeft = getPositionAllSanitizedAvg(leftMotors);
		m_prevRight = getPositionAllSanitizedAvg(rightMotors);

		// a failed read seeds PROS_ERR, so that wheel re-seeds on its first good read
		m_prevParallel = parallelTrackingWheel.get_position();
		m_prevPerpendicular = perpendicularTrackingWheel.get_position();

		// tracking wheels unless both are down
		m_config.useMotorEncoders = (m_prevParallel == PROS_ERR && m_prevPerpendicular == PROS_ERR);

		m_initialized = true;
		return;
	}

	// IME travel is read every frame, so its previous readings are current whenever a fallback needs them
	WheelLengths imeTravel = getDriveEncoderTravel();

	// Calculate distance travelled by either drive encoders or tracking wheels
	WheelLengths arcs;
	// whether each axis was measured by its tracking wheel, so its offset correction applies
	bool parallelFromWheel = false;
	bool perpendicularFromWheel = false;
	// tracking wheel error to show on the LCD this frame, if any
	const char* wheelError = nullptr;
	if (m_config.useMotorEncoders) {
		arcs = imeTravel;

		// switch back once either tracking wheel responds. seeding prev with this reading
		// (PROS_ERR for a wheel still down) makes the next frame's travel start from here
		double currParallel = parallelTrackingWheel.get_position();
		double currPerpendicular = perpendicularTrackingWheel.get_position();
		if (currParallel != PROS_ERR || currPerpendicular != PROS_ERR) {
			m_prevParallel = currParallel;
			m_prevPerpendicular = currPerpendicular;
			m_config.useMotorEncoders = false;
		} else {
			wheelError = "[Update Pose] Tracking Wheel Error!";
		}
	} else {
		WheelTravelResult wheelResult = getOdomWheelTravel(imeTravel.parallel);

		switch (wheelResult.error) {
			case OdomError::NONE:
				arcs = wheelResult.travel;
				parallelFromWheel = true;
				perpendicularFromWheel = true;
				break;
			case OdomError::PARALLEL_WHEEL_ERROR:
				wheelError = "[Update Pose] Parallel Wheel Error!";
				arcs = wheelResult.travel;
				perpendicularFromWheel = true;
				break;
			case OdomError::PERPENDICULAR_WHEEL_ERROR:
				wheelError = "[Update Pose] Perpendicular Wheel Error!";
				arcs = wheelResult.travel;
				parallelFromWheel = true;
				break;
			default:
				// both wheels down: IMEs until one comes back
				wheelError = "[Update Pose] Tracking Wheel Error!";
				m_config.useMotorEncoders = true;
				arcs = imeTravel;
				break;
		}
	}

	if (wheelError) {
		static uint32_t lastWheelFailurePrint = 0;
		const uint32_t now = pros::millis();
		if (now - lastWheelFailurePrint >= 500) {
			pros::lcd::print(0, "%s", wheelError);
			lastWheelFailurePrint = now;
		}
	}

	// Determine change in heading
	double del_theta = normalizeAngle(theta_rad - m_prevTheta);

	// Determine change in local x and in local y
	// offset corrections remove a tracking wheel's own rotation, so they only apply to travel that wheel measured
	// (IME forward travel is already at the drive center, and a missing perpendicular wheel's sideways is taken as 0)
	double dx_local = arcs.parallel;
	double dy_local = arcs.perpendicular;
	if (parallelFromWheel) {
		dx_local -= del_theta * m_config.parallelTrackingWheelOffset;
	}
	if (perpendicularFromWheel) {
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

	m_prevTheta = theta_rad;

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

YawResult Odometry::getYaw(void) {

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

WheelTravelResult Odometry::getOdomWheelTravel(double imeForward) {
	if (!m_initialized) {
		return {{0.0, 0.0}, OdomError::NONE};
	}

    // Get current centidegree position of tracking wheels
	double currParallel = parallelTrackingWheel.get_position();
	double currPerpendicular = perpendicularTrackingWheel.get_position();

	// travel since last frame. also saves each reading (or PROS_ERR) as previous
	double delParallel = trackingWheelTravel(currParallel, m_prevParallel, m_config.parallelWheelDiameter);
	double delPerpendicular = trackingWheelTravel(currPerpendicular, m_prevPerpendicular, m_config.perpendicularWheelDiameter);

	// both down: updatePose() switches to the IMEs
	if (currParallel == PROS_ERR && currPerpendicular == PROS_ERR) {
		return {{0.0, 0.0}, OdomError::TRACKING_WHEEL_ERROR};
	}

	// perpendicular down: parallel wheel forward, no sideways
	if (currPerpendicular == PROS_ERR) {
		return {{delParallel, 0.0}, OdomError::PERPENDICULAR_WHEEL_ERROR};
	}

	// parallel down: IMEs forward, perpendicular wheel sideways
	if (currParallel == PROS_ERR) {
		return {{imeForward, delPerpendicular}, OdomError::PARALLEL_WHEEL_ERROR};
	}

	return {{delParallel, delPerpendicular}, OdomError::NONE};
}

VelocityResult Odometry::getParallelVel() {
	double initialVelocity = parallelTrackingWheel.get_velocity();

	// IMEs when odometry is on them, or when the parallel wheel is down
	if (m_config.useMotorEncoders || initialVelocity == PROS_ERR){

		double leftVelocity = averageMotorGroupVelocities(leftMotors);
		double rightVelocity = averageMotorGroupVelocities(rightMotors);

		if (leftVelocity == PROS_ERR_F || rightVelocity == PROS_ERR_F) {
			return {0.0, OdomError::PARALLEL_WHEEL_ERROR};
		}

		// motor velocities are in RPM
		double averageRpm = (leftVelocity + rightVelocity) / 2;

		double velocity  = averageRpm * DRIVE_GEAR_RATIO * (m_config.driveWheelDiameter / 2) * (2 * std::numbers::pi / 60);
		
		return {velocity, OdomError::NONE};
	}

	// rotation sensor velocity is in centidegrees per second
	double deg_s = initialVelocity / 100.0;

	double velocity = deg_s * (std::numbers::pi / 180) * (m_config.parallelWheelDiameter / 2);

	return {velocity, OdomError::NONE};
}

WheelLengths Odometry::getDriveEncoderTravel(void) {
	if (!m_initialized) {
		return {0, 0};
	}

    double currLeft = getPositionAllSanitizedAvg(leftMotors);
    double currRight = getPositionAllSanitizedAvg(rightMotors);

    // a side only has travel when it read this frame and last frame. a failed read stores PROS_ERR_F
    // in prev, so the side's first good read after that only re-seeds it
    bool leftValid = currLeft != PROS_ERR_F && m_prevLeft != PROS_ERR_F;
    bool rightValid = currRight != PROS_ERR_F && m_prevRight != PROS_ERR_F;

    double leftInches = leftValid ? ((currLeft - m_prevLeft) / 360.0) * DRIVE_GEAR_RATIO * m_config.driveWheelDiameter * std::numbers::pi : 0.0;
    double rightInches = rightValid ? ((currRight - m_prevRight) / 360.0) * DRIVE_GEAR_RATIO * m_config.driveWheelDiameter * std::numbers::pi : 0.0;

    m_prevLeft = currLeft;
    m_prevRight = currRight;

    // average both sides, or use the one side that has travel (off by the turn while turning, but better than halving it)
    double forwardInches = 0.0;
    if (leftValid && rightValid) {
        forwardInches = (leftInches + rightInches) / 2.0;
    } 
	else if (leftValid) {
        forwardInches = leftInches;
    } 
	else if (rightValid) {
        forwardInches = rightInches;
    }
    return {forwardInches, 0.0};
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
