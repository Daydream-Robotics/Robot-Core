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
double accelLimit(double prev, double target, double dt, double maxAccel) {
	double maxDelta = maxAccel * dt;
	double delta = target - prev;

	if (delta > maxDelta) delta = maxDelta;
	if (delta < -maxDelta) delta = -maxDelta;

	return prev + delta;
}

HeadingFilter::HeadingFilter(double alpha) : m_alpha(alpha) {}

double HeadingFilter::update(double raw) {
    if (!m_initialized) {
        m_heading = raw;
        m_initialized = true;
        return m_heading;
    }

    m_heading += m_alpha * angleDiffDeg(raw, m_heading);
    return m_heading;
}

void HeadingFilter::reset() {
    m_initialized = false;
}


double calcDistBetweenPoints(Position pt1, Position pt2) {
    return std::hypot(pt1.x - pt2.x, pt1.y - pt2.y);
}


// ! INTEGRATION TEAM: HI, also this is stuff I had as MCL helper functions, I had it before in a file called sensors.cpp/hpp but idk where you want it now
// ! This also has some odom logic that you guys don't have, ie falling back to IMES and sanatizing
//========= MCL Helpers =========//

//helper function to sanatize distance sensor reading
double sanitizeDistanceReading(long val) {
    return (val == 9999 || val == PROS_ERR) ? NAN : static_cast<double>(val);
}

//helper function to sanatize other values
double sanitizeNumericReading(double val) {
    return (val == PROS_ERR || val == PROS_ERR_F) ? NAN : val;
}

//helper function to take in tracking enum and return string
const char* odomModeName(OdomMode mode) {
    switch (mode) {
        case OdomMode::TWO_TRACKING: return "TWO_TRACKING"; 
        case OdomMode::ONE_TRACKING: return "ONE_TRACKING"; 
        case OdomMode::DRIVE_ENCODERS: return "DRIVE_ENCODERS";
        default: return "UNKNOWN";
    }
}

//helper function to take in unit enum and return string
const char* motorUnitsName(pros::MotorUnits units) {
    switch (units) {
        case pros::MotorUnits::degrees: return "deg";
        case pros::MotorUnits::rotations: return "rot";
        case pros::MotorUnits::counts: return "counts";
        default: return "invalid"; 
    }
}

//converts motor position into the linear distance the wheel has traveled compared to 0 position of encoder
double motorPositionToWheelInches(double rawPosition, pros::MotorUnits units) {
    if (std::isnan(rawPosition)) {
        return NAN;
    }
    switch (units) {
        case pros::MotorUnits::degrees:
            return rawPosition * DRIVE_IN_PER_DEG * DRIVE_GEAR_RATIO;
        case pros::MotorUnits::rotations:
            return rawPosition * DRIVE_WHEEL_CIRCUMFERENCE_IN * DRIVE_GEAR_RATIO;
        default:
            return NAN;
    }
}

//returns the drive motor's accumulated wheel travel in inches, or NAN if unavailable.
double driveMotorInches(const std::optional<pros::Motor>& motor) {
    if (!motor) {
        return NAN;
    }
    const double rawPosition = sanitizeNumericReading(motor->get_position());
    return motorPositionToWheelInches(rawPosition, motor->get_encoder_units());
}

//converts a tracking wheel rotation sensor reading to wheel travel in inches.
double trackingWheelInches(const std::optional<pros::Rotation>& sensor, double scale) {
    if (!sensor) {
        return NAN;
    }
    const double rawCentideg = sanitizeNumericReading(sensor->get_position());
    return std::isnan(rawCentideg) ? NAN : rawCentideg * scale;
}

//rangle sensors for MCL
RangeSensors distanceSensors = {
    pros::Distance(3),  // front
    pros::Distance(2),  // left
    pros::Distance(8),  // back
    pros::Distance(9)}; // right

// struct for odom with backups
OdomSensors chassisOdomSensors = {
    OdomMode::DRIVE_ENCODERS,
    std::nullopt,
    std::nullopt,
    pros::Motor(-11, pros::MotorGears::blue, pros::MotorUnits::degrees),
    pros::Motor(18, pros::MotorGears::blue, pros::MotorUnits::degrees),
    pros::Imu(21),
    std::nullopt
};   

//vars to hold distance and odom readings
RangeReadings distanceReadings;
OdomReadings odomValues;

//values to calibrate distance sensors
// ! should be moved to constants but does depend on the structs
RangeCalibration distanceCalibration = {0.9896,0.9896,0.9896,0.9816};
// Odometry scale factors:
// - NEW_2 omniwheel: 2.0" diameter, circumference = 6.283185"
// - Rotation sensor: 36000 centidegrees/revolution
// - TPI = 36000 / 6.283185 = 5729.578 centidegrees/inch
// - IMU scale: degrees to radians = π/180 = 0.017453293
OdomCalibration odomScale = {
    1.0 / 5729.578,  // parallel tracking TPI (inches per centidegree)
    1.0 / 5729.578,  // perpendicular tracking TPI (inches per centidegree)
    0.017453293,     // IMU scale (radians per degree)
    1.0              // IMU two scale (unused)
};

// odom tracking wheel offsets offset
OdomOffset trackingOffset = {0.5, 0.75};  // parallel=0.5", perpendicular=0.75"

//update range sensor readings, (store, sanatize, convert to in, apply calibration)
void updateRangeSensors() {
    distanceReadings.front = sanitizeDistanceReading(distanceSensors.front.get_distance()) * MM_TO_IN * distanceCalibration.front;
    distanceReadings.left = sanitizeDistanceReading(distanceSensors.left.get_distance()) * MM_TO_IN * distanceCalibration.left;
    distanceReadings.back = sanitizeDistanceReading(distanceSensors.back.get_distance()) * MM_TO_IN * distanceCalibration.back;
    distanceReadings.right = sanitizeDistanceReading(distanceSensors.right.get_distance()) * MM_TO_IN * distanceCalibration.right;
}

// ! update odom sensors, ie return the parallel and perpindicular local distance, either using tracking wheels or IMES
void updateOdomSensors() {
    double parallel = NAN;
    double perpendicular = NAN;

    // Case 1: parallel tracking wheel exists
    if (chassisOdomSensors.parallelTracking) {
        parallel = sanitizeNumericReading(chassisOdomSensors.parallelTracking->get_position())
            * odomScale.parallelTrackingTpi;
    }
    // Case 2: use drivetrain encoders
    else if (chassisOdomSensors.driveLeft && chassisOdomSensors.driveRight) {
        const double left = driveMotorInches(chassisOdomSensors.driveLeft);
        const double right = driveMotorInches(chassisOdomSensors.driveRight);
        // forward displacement from differential drive
        parallel = (left + right) * 0.5;
    }

    // perpendicular tracking wheel
    if (chassisOdomSensors.perpendicularTracking) {
        perpendicular = sanitizeNumericReading(chassisOdomSensors.perpendicularTracking->get_position())
            * odomScale.perpendicularTrackingTpi;
    }
    else {
        // no strafe measurement
        perpendicular = 0.0;
    }

    const double heading = sanitizeNumericReading(chassisOdomSensors.imuOne.get_rotation()) *
        IMU_RAD_PER_DEG;

    odomValues = {parallel, perpendicular, heading};
}

//debug printing for range sensors
void printRangeSensorDebug() {
    const double frontMm = sanitizeDistanceReading(distanceSensors.front.get_distance());
    const double leftMm = sanitizeDistanceReading(distanceSensors.left.get_distance());
    const double backMm = sanitizeDistanceReading(distanceSensors.back.get_distance());
    const double rightMm = sanitizeDistanceReading(distanceSensors.right.get_distance());

    updateRangeSensors();

    printf("=== RANGE SENSOR DEBUG ===\n");
    printf("Units: raw=mm, converted=inches, MM_TO_IN=%.6f\n", MM_TO_IN);
    printf("Front: raw=%.2f mm, cal=%.4f, out=%.4f in\n", frontMm, distanceCalibration.front, distanceReadings.front);
    printf("Left:  raw=%.2f mm, cal=%.4f, out=%.4f in\n", leftMm, distanceCalibration.left, distanceReadings.left);
    printf("Back:  raw=%.2f mm, cal=%.4f, out=%.4f in\n", backMm, distanceCalibration.back, distanceReadings.back);
    printf("Right: raw=%.2f mm, cal=%.4f, out=%.4f in\n", rightMm, distanceCalibration.right, distanceReadings.right);
    printf("==========================\n");
}

//debug print for odom sensors
void printOdomSensorDebug() {
    const double leftRaw = chassisOdomSensors.driveLeft
        ? sanitizeNumericReading(chassisOdomSensors.driveLeft->get_position()) : NAN;
    const double rightRaw = chassisOdomSensors.driveRight
        ? sanitizeNumericReading(chassisOdomSensors.driveRight->get_position()) : NAN;
    const pros::MotorUnits leftUnits = chassisOdomSensors.driveLeft
        ? chassisOdomSensors.driveLeft->get_encoder_units() : pros::MotorUnits::invalid;
    const pros::MotorUnits rightUnits = chassisOdomSensors.driveRight
        ? chassisOdomSensors.driveRight->get_encoder_units() : pros::MotorUnits::invalid;
    const double leftInches = driveMotorInches(chassisOdomSensors.driveLeft);
    const double rightInches = driveMotorInches(chassisOdomSensors.driveRight);

    const double parallelRawCentideg = chassisOdomSensors.parallelTracking
        ? sanitizeNumericReading(chassisOdomSensors.parallelTracking->get_position()) : NAN;
    const double perpRawCentideg = chassisOdomSensors.perpendicularTracking
        ? sanitizeNumericReading(chassisOdomSensors.perpendicularTracking->get_position()) : NAN;
    const double parallelInches = trackingWheelInches(chassisOdomSensors.parallelTracking, odomScale.parallelTrackingTpi);
    const double perpInches = trackingWheelInches(chassisOdomSensors.perpendicularTracking, odomScale.perpendicularTrackingTpi);

    const double imuHeadingDeg = sanitizeNumericReading(chassisOdomSensors.imuOne.get_heading());
    const double imuRotationDeg = sanitizeNumericReading(chassisOdomSensors.imuOne.get_rotation());

    updateOdomSensors();

    printf("=== ODOM SENSOR DEBUG ===\n");
    printf("Mode: %s\n", odomModeName(chassisOdomSensors.mode));
    printf("Drive encoder conversion: wheel_diam=%.3f in, wheel_circ=%.6f in, ext_ratio=%.6f\n",
            DRIVE_WHEEL_DIAMETER_INCHES, DRIVE_WHEEL_CIRCUMFERENCE_IN, DRIVE_GEAR_RATIO);
    printf("Left drive:  raw=%.4f %s, conv=%.4f in\n", leftRaw, motorUnitsName(leftUnits), leftInches);
    printf("Right drive: raw=%.4f %s, conv=%.4f in\n", rightRaw, motorUnitsName(rightUnits), rightInches);
    printf("Drive avg parallel=%.4f in, left-right diff=%.4f in\n",
            (leftInches + rightInches) * 0.5, leftInches - rightInches);
    printf("Parallel tracker: raw=%.4f cdeg, conv=%.4f in\n", parallelRawCentideg, parallelInches);
    printf("Perp tracker:     raw=%.4f cdeg, conv=%.4f in\n", perpRawCentideg, perpInches);
    printf("IMU heading=%.4f deg (wrapped), rotation=%.4f deg, rotation=%.6f rad\n",
            imuHeadingDeg, imuRotationDeg, imuRotationDeg * IMU_RAD_PER_DEG);
    printf("Computed odom_values: parallel=%.4f in, perp=%.4f in, heading=%.6f rad\n",
            odomValues.parallelTracking, odomValues.perpendicularTracking, odomValues.heading);
    printf("=========================\n");
}
