#include "autonomous.hpp"
#include "subsystems.h"

// Utility class for smoothing heading changes
class HeadingFilter {
	private:
		double alpha;
		double heading;
		bool initialized = false;

	public:
		HeadingFilter(double alpha) : alpha(alpha) {}

		double update(double raw) {
			if (!initialized) {
				heading = raw;
				initialized = true;
				return heading;
			}

			heading += alpha * angleDiffDeg(raw, heading);
			return heading;
		}

		void reset() {
			initialized = false;
		}
};


// TODO: Tune PID parameters
Autonomous::Autonomous() 
	: distancePID(0.0, 0.0, 0.0, 0.0), 
	headingPID(0.0, 0.0, 0.0, 0.0),
	turnPID(0.0, 0.0, 0.0, 0.0) {
		leftMotors.set_brake_mode_all(pros::E_MOTOR_BRAKE_HOLD);
		rightMotors.set_brake_mode_all(pros::E_MOTOR_BRAKE_HOLD);
	}

void Autonomous::turnTo(double targetHeading) {
	turnPID.reset();
    turnPID.setTarget(targetHeading);

	// TODO: Tune exit conditions
    turnPID.exit_condition_set(
        0.2, 150,     // small error (deg), time (ms)
        2.0, 300,     // big error (deg), time
        200,          // velocity settle time
        4000          // timeout
    );

	// TODO: Tune alpha
    HeadingFilter headingFilter(0.3);

	// Initialize clocking
	using clock = std::chrono::steady_clock;
    auto lastTime = clock::now();

	while (true) {
		// Heading calculation
		double rawHeading = getYaw();

		if (rawHeading < 0) {
			// TODO: Add more verbose error handling
			return;
		}

		// Determine PID correction using smoothed heading
		double filteredHeading = headingFilter.update(rawHeading - 180);
		double correction = headingPID.compute(filteredHeading);

		// Compute turnSpeed based on correction
		double turnSpeed = std::clamp(
            std::fabs(correction),
            2.0,
            65.0
        );

        turnSpeed = std::copysign(turnSpeed, correction);

		leftMotors.move_velocity(turnSpeed);
        rightMotors.move_velocity(-turnSpeed);

		if (turnPID.exit_condition(100) != PID::RUNNING) // TODO: get Angular Velocity
            break;

		pros::delay(10);
	}

	leftMotors.move_velocity(0);
    rightMotors.move_velocity(0);
    pros::delay(250);

}

void Autonomous::travel(double distance, double speed, double targetHeading, double timer_s) {
	// Set PID targets
	distancePID.setTarget(distance);
	headingPID.setTarget(targetHeading);

	// TODO: Tune Exit Conditions
	distancePID.exit_condition_set(
		0.5, 200,	// small error (in), (ms)
		2.0, 400,	// big error (in),(ms)
		200,		// velocity settling time (ms)
		timer_s * 1000 // timeout (ms)
	);

	// TODO: Tune Exit Conditions
	headingPID.exit_condition_set(
        1.0, 150,   // small error (deg), (ms)
        3.0, 300,	// big error (deg), (ms)
        200,		// velocity error (deg), (ms)
        timer_s * 1000
    );

	// TODO: Tune alpha [0, 1] [more smooth, less smooth]
	HeadingFilter headingFilter(0.35);

	Position start(pos_x, pos_y);

	double prevForward = 0.0;

	using clock = std::chrono::steady_clock;
	auto startTime = clock::now();
	auto lastTime = startTime;

	// Find heading and direction
	double headingRad = convertDegToRad(targetHeading);
    double direction = (distance >= 0.0) ? 1.0 : -1.0;

    Position headingUnit {
        direction * -std::cos(headingRad),
        direction * -std::sin(headingRad)
    };

	while (true) {
		// Find time step value
		auto now = clock::now();
		double dt = std::chrono::duration<double>(now - lastTime).count();
		if (dt <= 0.0) dt = 1e-3;
		lastTime = now;

		// Update current position and orientation
		updatePose();

		// Find distance traveled
		Position delta { pos_x - start.x, pos_y - start.y };
		double traveled = delta.x * headingUnit.x + delta.y * headingUnit.y;
		double remaining = std::fabs(distance) - std::fabs(traveled);

		// Hard STOP
		if (remaining < STOPTHRESHOLD) {
			break;
		}

		// Compute max target speed
		double speedScale = computeDecelScale(remaining, distance);
		double targetForward = speed * direction * speedScale;

		// Smooth forward momentum
		double forward = accelLimit(prevForward, targetForward, dt, accelLimitRate);
		prevForward = forward;

		// Heading calculation
		double rawHeading = getYaw();

		if (rawHeading < 0) {
			// TODO: Add more verbose error handling
			return;
		}

		// Determine PID correction using smoothed heading
		double filteredHeading = headingFilter.update(rawHeading - 180);
		double correction = headingPID.compute(filteredHeading);

		// Takeoff ramping
		double motionTime = std::chrono::duration<double>(now - startTime).count();
        double ramp = std::min(1.0, motionTime / takeoffRampTime);
        correction *= ramp;

		// Compute motor velocities
		double leftVel  = forward + correction;
        double rightVel = forward - correction;

        leftMotors.move_velocity(leftVel);
        rightMotors.move_velocity(rightVel);


		// Break if PID exit condition is met
		if (distancePID.exit_condition(100) != PID::RUNNING) // TODO: get linear velocity
            break;

        if (headingPID.exit_condition(100) != PID::RUNNING) // TODO: get angular velocity
            break;

		pros::delay(10);

	}

	leftMotors.move_velocity(0);
    rightMotors.move_velocity(0);
    pros::delay(250);


}

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

// Determine deceleration speed scaling
static double computeDecelScale(double remaining, double totalDistance) {
    double decelDistance = std::max(0.2, std::fabs(totalDistance) * 0.15);

    if (remaining >= decelDistance)
        return 1.0;

    double x = std::clamp(remaining / decelDistance, 0.0, 1.0);

    // Step smoothing
    double smooth = x * x * (3.0 - 2.0 * x);

	// Return speed scaling
    return std::clamp(smooth, 0.2, 1.0);
}

// Limit acceleration takeoff
static double accelLimit(double prev, double target, double dt, double accelLimit) {
    double maxDelta = accelLimit * dt;
    double delta = target - prev;

    if (delta > maxDelta) delta = maxDelta;
    if (delta < -maxDelta) delta = -maxDelta;

    return prev + delta;
}
