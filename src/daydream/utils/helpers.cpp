#include "daydream/config/constants.h"
#include "daydream/utils/helpers.hpp"
#include "main.h"
#include <cmath>
#include <numbers>
#include <algorithm>

// ====== Helper functions ======

/**
 * @brief Normalize angle between [-pi, +pi]
 * @returns Angle in radians between -pi and pi
 */
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
    return rad * (180.0 / std::numbers::pi);
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
	return std::clamp(smooth, 0.0, 1.0);
}

// Limit acceleration takeoff
double accelLimit(double prev, double target, double dt, double accel_limit) {
	double maxDelta = accel_limit * dt;
	double delta = target - prev;

	if (delta > maxDelta) delta = maxDelta;
	if (delta < -maxDelta) delta = -maxDelta;

	return prev + delta;
}

HeadingFilter::HeadingFilter(double alpha) : alpha(alpha) {}

double HeadingFilter::update(double raw) {
    if (!initialized) {
        heading = raw;
        initialized = true;
        return heading;
    }

    heading += alpha * angleDiffDeg(raw, heading);
    return heading;
}

void HeadingFilter::reset() {
    initialized = false;
}


double calcDistBetweenPoints(Position pt1, Position pt2) {
    return std::hypot(pt1.x - pt2.x, pt1.y - pt2.y);
}



//========= MCL Helpers =========//

//helper function to sanatize distance sensor reading
double sanitize_distance_reading(long val) {
    return (val == 9999 || val == PROS_ERR) ? NAN : static_cast<double>(val);
}

//helper function to sanatize other values
double sanitize_numeric_reading(double val) {
    return (val == PROS_ERR || val == PROS_ERR_F) ? NAN : val;
}

//helper function to take in tracking enum and return string
const char* odom_mode_name(odom_mode mode) {
    switch (mode) {
        case odom_mode::TWO_TRACKING: return "TWO_TRACKING"; break;
        case odom_mode::ONE_TRACKING: return "ONE_TRACKING"; break;
        case odom_mode::DRIVE_ENCODERS: return "DRIVE_ENCODERS"; break;
        default: return "UNKNOWN";
    }
}

//helper function to take in unit enum and return string
const char* motor_units_name(pros::MotorUnits units) {
    switch (units) {
        case pros::MotorUnits::degrees: return "deg"; break;
        case pros::MotorUnits::rotations: return "rot"; break;
        case pros::MotorUnits::counts: return "counts"; break;
        default: return "invalid"; 
    }
}

//converts motor position to wheel inches
double motor_position_to_wheel_inches(double raw_position, pros::MotorUnits units) {
    if (std::isnan(raw_position)) {
        return NAN;
    }
    switch (units) {
        case pros::MotorUnits::degrees:
            return raw_position * DRIVE_IN_PER_DEG * DRIVE_GEAR_RATIO;
            break;
        case pros::MotorUnits::rotations:
            return raw_position * DRIVE_WHEEL_CIRCUMFERENCE_IN * DRIVE_GEAR_RATIO;
            break;
        default:
            return NAN;
    }
}

//
double drive_motor_inches(const std::optional<pros::Motor>& motor) {
    if (!motor) {
        return NAN;
    }
    const double raw_position = sanitize_numeric_reading(motor->get_position());
    return motor_position_to_wheel_inches(raw_position, motor->get_encoder_units());
}

double tracking_wheel_inches(const std::optional<pros::Rotation>& sensor, double scale) {
    if (!sensor) {
        return NAN;
    }
    const double raw_centideg = sanitize_numeric_reading(sensor->get_position());
    return std::isnan(raw_centideg) ? NAN : raw_centideg * scale;
}