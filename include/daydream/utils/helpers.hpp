#pragma once

#include "daydream/motion/odometry.hpp" // Needed for the Position struct
#include "main.h"

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


// ! INTEGRATION TEAM: HI, also this is stuff I had as MCL helper functions, I had it before in a file called sensors.cpp/hpp but idk where you want it now

//========= MCL Helpers =========//

//distance sensor objects for all 4 directions
struct RangeSensors {
	pros::Distance front;
	pros::Distance left;
	pros::Distance back;
	pros::Distance right;
};

//odom mode
enum class OdomMode {
	TWO_TRACKING,
	ONE_TRACKING,
	DRIVE_ENCODERS
};

struct OdomSensors {
	//odom mode
	OdomMode mode;

	//rotation sensors
	std::optional<pros::Rotation> parallelTracking;
	std::optional<pros::Rotation> perpendicularTracking;

	//IMEs
	std::optional<pros::Motor> driveLeft;
	std::optional<pros::Motor> driveRight;

	//imus
	pros::Imu imuOne;
	std::optional<pros::Imu> imuTwo;
};

//distance sensor readings in inches (supports named and array access)
struct RangeReadings {
	union {
		struct { double front, left, back, right; };
		double direction[4];
	};
};

//odometry sensor readings
struct OdomReadings {
	double parallelTracking;
	double perpendicularTracking;
	double heading;
};

//previous odom readings
struct PrevOdom {
	double parallelTracking;
	double perpendicularTracking;
	double heading;
	double leftDrive;
	double rightDrive;
};

//calibration factors for distance sensors
struct RangeCalibration {
	union {
		struct { double front, left, back, right; };
		double direction[4];
	};
};

//calibration factors for odometry sensors
struct OdomCalibration {
	double parallelTrackingTpi;
	double perpendicularTrackingTpi;
	double imuOneScale;
	double imuTwoScale;
};

//physical offsets for distance sensors
struct RangeOffset {
	union {
		struct { double front, left, back, right; };
		double direction[4];
	};
};

//physical offsets for tracking wheels
struct OdomOffset {
	double parallelTracking;
	double perpendicularTracking;
};

//distance sensor objects
extern RangeSensors distanceSensors;
//odometry sensor objects
extern OdomSensors chassisOdomSensors;
//current odometry readings
extern OdomReadings odomValues;
//tracking wheel offsets from robot center
extern OdomOffset trackingOffset;
//current distance sensor readings
extern RangeReadings distanceReadings;

//updates odometry sensor readings
void updateOdomSensors();
//updates distance sensor readings
void updateRangeSensors();
//prints raw + converted range sensor values for unit checking
void printRangeSensorDebug();
//prints raw + converted odom sensor values for sign/unit checking
void printOdomSensorDebug();
