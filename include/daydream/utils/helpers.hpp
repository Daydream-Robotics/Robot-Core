#pragma once

#include "daydream/motion/odometry.hpp" // Needed for the Position struct
#include "main.h"

// Normalize angle between [-pi,pi] 
double normalizeAngle(double a);

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
double accelLimit(double prev, double target, double dt, double maxAccel);

// Utility class for smoothing heading changes
class HeadingFilter {
	private:
		double m_alpha;
		double m_heading = 0.0;
		bool m_initialized = false;

	public:
		HeadingFilter(double alpha);
		double update(double raw);
		void reset();
};


double calcDistBetweenPoints(Position pt1, Position pt2);

//measure effective drivetrain track width L (in) using IMEs + IMU
double measureTrackWidth(int iterations = 10);
