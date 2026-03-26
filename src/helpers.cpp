#include "constants.h"
#include "helpers.hpp"
#include <cmath>
#include <numbers>
#include <algorithm>

// ====== Helper functions ======

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