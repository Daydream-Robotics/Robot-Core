#pragma once

#include "odometry.hpp" // Needed for the Position struct

// Normalize angle between [-180, 180] 
double normalizeAngle(double a);

// Return Euclidean distance btwn points p1 and p2
double getDistance(Position p1, Position p2);

// Convert Degrees to Radians
double convertDegToRad(double degree);

// Covert Radians to Degrees
double convertRadToDeg(double rad);

// Subtract angles with [-180, 180] wrapping
double angleDiffDeg(double a, double b);

// Subtract angles with [-pi, +pi] wrapping
double angleDiffRad(double a, double b);

// Determine deceleration speed scaling
double computeDecelScale(double remaining, double totalDistance);

// Limit acceleration takeoff
double accelLimit(double prev, double target, double dt, double accel_limit);

// Utility class for smoothing heading changes
class HeadingFilter {
	private:
		double alpha;
		double heading = 0.0;
		bool initialized = false;

	public:
		HeadingFilter(double alpha);
		double update(double raw);
		void reset();
};


double calcDistBetweenPoints(Position pt1, Position pt2);