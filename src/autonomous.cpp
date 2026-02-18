#include "autonomous.hpp"
#include "subsystems.h"

// ====== Helper functions ======
namespace {
	// Normalize angle between [-180, 180]
	double normalizeAngle(double a) {
		return std::atan2(std::sin(a), std::cos(a));
	}
	
	// Return Euclidean distance btwn points p1 and p2
	double getDistance(Position p1, Position p2) {
		return std::sqrt(std::pow((p2.x - p1.x), 2) + std::pow((p2.y - p1.y), 2));
	}
	
	// Convert Degrees to Radians
	double convertDegToRad(double degree) {
		return degree * (std::numbers::pi / 180.0);
	}
	
	// Covert Radians to Degrees
	double convertRadToDeg(double rad) {
		return rad * (180.0 / M_PI);
	}
	
	// Subtract angles with [-180, 180] wrapping
	double angleDiffDeg(double a, double b) {
		double c = a - b;
		while (c > 180.0) c -= 360.0;
		while (c <= -180.0) c += 360.0;
		return c;
	}
	
	// Determine deceleration speed scaling
	double computeDecelScale(double remaining, double totalDistance) {
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
	double accelLimit(double prev, double target, double dt, double accelLimit) {
		double maxDelta = accelLimit * dt;
		double delta = target - prev;
	
		if (delta > maxDelta) delta = maxDelta;
		if (delta < -maxDelta) delta = -maxDelta;
	
		return prev + delta;
	}
	
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
}


// TODO: Tune PID parameters
Autonomous::Autonomous() 
	: distancePID(4.25, 0.0, 0.0, 0.0), 
	headingPID(0.004, 0.01, 0.0, 0.0, true),
	turnPID(1.2, 0.1, 0.001, 15.0, true) {
		leftMotors.set_brake_mode_all(pros::E_MOTOR_BRAKE_HOLD);
		rightMotors.set_brake_mode_all(pros::E_MOTOR_BRAKE_HOLD);
		
	}

void Autonomous::turnTo(double targetHeading) {
	turnPID.reset();
    turnPID.setTarget(targetHeading);

	pros::lcd::print(0, "Turning to %lf degrees", targetHeading);

	// TODO: Tune exit conditions
    turnPID.exit_condition_set(
        0.2, 250,     // small error (deg), time (ms)
        1.0, 3000,     // big error (deg), time
        200,          // velocity settle time
        0          // timeout
    );

	// TODO: Tune alpha
    HeadingFilter headingFilter(1);

	// Initialize clocking
	using clock = std::chrono::steady_clock;
    auto lastTime = clock::now();

	while (true) {
		// Heading calculation
		double rawHeading = getYaw();
		updatePose();

		if (rawHeading < 0) {
			pros::lcd::print(0, "IMU Failure!");
			// TODO: Add more verbose error handling
			return;
		}

		// Determine PID correction using smoothed heading
		double filteredHeading = headingFilter.update(rawHeading - 180);
		double correction = turnPID.compute(filteredHeading);
		//pros::lcd::print(1, "Correction: %lf", correction);

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

	pros::lcd::print(2, "Exited Turn Loop");

	leftMotors.move_velocity(0);
    rightMotors.move_velocity(0);
    pros::delay(250);

	updatePose();
}

void Autonomous::travel(double distance, double speed, double targetHeading, double timer_s) {

    auto clamp = [](double v, double lo, double hi) {
        return (v < lo) ? lo : (v > hi) ? hi : v;
    };

    auto normalizeDeg = [](double a) {
        while (a >= 180.0) a -= 360.0;
        while (a < -180.0) a += 360.0;
        return a;
    };

    distancePID.setTarget(distance);
    distancePID.exit_condition_set(
        0.5, 400,
        2.0, 1200,
        200,
        timer_s * 1000
    );

    headingPID.setTarget(0.0);

    Position start(pos_x, pos_y);

	double headingRad = convertDegToRad(targetHeading);
	Position headingUnit {
		std::sin(headingRad),
		-std::cos(headingRad)
	};

    while (true) {
        updatePose();

        // Compute traveled distance along heading vector
        Position delta { pos_x - start.x, pos_y - start.y };
        double traveled = delta.x * headingUnit.x + delta.y * headingUnit.y;

        double v = distancePID.compute(traveled);
        v = clamp(v, -speed, speed);

        // Heading error
        double rawHeading = getYaw();
        if (rawHeading < 0) {
            pros::lcd::print(0, "IMU Failure!");
            return;
        }

        double headingError = normalizeDeg(targetHeading - rawHeading);

        // Heading correction
        double omega = headingPID.compute(headingError);

        // Differential drive
        double left  = v + omega;
        double right = v - omega;

        // Magnitude Scaling
        double maxMag = std::max(std::fabs(left), std::fabs(right));
        if (maxMag > speed) {
            double scale = speed / maxMag;
            left  *= scale;
            right *= scale;
        }

        leftMotors.move_velocity(left);
        rightMotors.move_velocity(right);

		// Exit if any exit condition is met. 100 set to prevent velocity timeout for now
        if (distancePID.exit_condition(100) != PID::RUNNING)
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

	const double yaw = getYaw();

	if (yaw < 0) {
		pros::lcd::print(0, "IMU Failure!");
		return;
	}

	static double prevTheta = convertDegToRad(yaw - 180);

	// Get orientation from IMU
	double theta = convertDegToRad(yaw - 180);
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

	 controller.print(0, 0, "O: %.2lf\n", convertRadToDeg(theta));
	//controller.print(0, 0, "X: %.2f Y: %.2f\n", pos_x, pos_y);
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

