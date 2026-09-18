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

//========= MCL Helpers =========//

//distance sensor objects for all 4 directions
struct range_sensors {
	pros::Distance front;
	pros::Distance left;
	pros::Distance back;
	pros::Distance right;
};

//odom mode
enum class odom_mode {
	TWO_TRACKING,
	ONE_TRACKING,
	DRIVE_ENCODERS
};

struct odom_sensors {
	//odom mode
	odom_mode mode;

	//rotation sensors
	std::optional<pros::Rotation> parallel_tracking;
	std::optional<pros::Rotation> perpendicular_tracking;

	//IMEs
	std::optional<pros::Motor> drive_left;
	std::optional<pros::Motor> drive_right;

	//imus
	pros::Imu imu_one;
	std::optional<pros::Imu> imu_two;
};

	//distance sensor readings in inches (supports named and array access)
    struct range_readings {
        union {
            struct { double front, left, back, right; };
            double direction[4];
        };
    };

    //odometry sensor readings
    struct odom_readings {
        double parallel_tracking;
        double perpendicular_tracking;
        double heading;
    };

	//previous odom readings
    struct prev_odom {
		double parallel_tracking;
		double perpendicular_tracking;
		double heading;
		double left_drive;
		double right_drive;
	};

    //calibration factors for distance sensors
    struct range_calibration {
        union {
            struct { double front, left, back, right; };
            double direction[4];
        };
    };

    //calibration factors for odometry sensors
    struct odom_calibration {
        double parallel_tracking_tpi;
        double perpendicular_tracking_tpi;
        double imu_one_scale;
        double imu_two_scale;
    };

    //physical offsets for distance sensors
    struct range_offset {
        union {
            struct { double front, left, back, right; };
            double direction[4];
        };
    };

    //physical offsets for tracking wheels
    struct odom_offset {
        double parallel_tracking;
        double perpendicular_tracking;
    };