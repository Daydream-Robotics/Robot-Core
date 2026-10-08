#include "daydream/config/constants.h"
#include "daydream/motion/odometry.hpp"
#include "daydream/utils/helpers.hpp"
#include "daydream/config/subsystems.hpp"
#include "daydream/autonomous/autonomous.hpp"
#include <cmath>
#include <numbers>
#include <algorithm>
#include <vector>

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

// Subtract angles with [-pi, pi] wrapping
double angleDiffDeg(double a, double b) {
    double c = a - b;
    while (c > std::numbers::pi) c -= 2 * std::numbers::pi;
    while (c <= -std::numbers::pi) c += 2 * std::numbers::pi;
    return c;
}

double angleDiffRad(double a, double b) {
    double c = a - b;
    while (c > M_PI) c -= 2 * M_PI;
    while (c < -M_PI) c += 2 * M_PI;
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

/**
 * @brief Measure the effective track width (L) of the drivetrain
 *
 * Turns the robot in place +/-90 deg several times and stores how far
 * each drive side traveled (with IMES) and how much the heading changed (IMU).
 *
 * When turning in place, each side moves along a circle around the robot's center:
 *     dLeft - dRight = L * dTheta
 * Each trial gives an estimate of L where all trials are combined with a least-squares fit:
 *     L = sum((dLeft - dRight) * dTheta) / sum(dTheta^2)
 *
 * @param iterations Number of turns to average over (alternates direction each turn)
 * @returns Effective track width L in inches (0.0 on IMU failure)
 */
double measureTrackWidth(int iterations){
    //local instance of auton so we can use turnTo / turnPID
    Autonomous auton; 

    //running sums for the least-squares fit of (dLeft - dRight) = L * dTheta
    double sumDiffTimesTheta = 0.0; //sum of (dLeft - dRight) * dTheta
    double sumThetaSquared = 0.0;   //sum of dTheta^2

    //average drive side travel in inches (same math as Odometry::getDriveEncoderTravel, which is private)
    auto sideInches = [](const pros::MotorGroup& group) {
            //average position (deg) across every motor on this side
            std::vector<double> positions = group.get_position_all();
            double sum = 0.0;
            for (double pos : positions) {
                sum += pos;
            }
            double deg = positions.empty() ? 0.0 : sum / positions.size();

            //motor deg to wheel rotations to inches of wheel circumference
            return (deg / 360.0) * DRIVE_GEAR_RATIO * DRIVE_WHEEL_DIAMETER_INCHES * std::numbers::pi;
    };

    //go throught the iterations
    for (int i = 0; i < iterations; ++i) {
        //zero the drive encoders so this trial's travel starts at 0
        leftMotors.tare_position_all();
        rightMotors.tare_position_all();
        pros::delay(100);

        //heading before the turn (rad)
        YawResult start = odom.getYaw();
        if (start.error != OdomError::NONE) {
                pros::lcd::print(0, "[TrackWidth] IMU Failure!");
                return 0.0;
        }

        //alternate +90 / -90 deg each trial so bias cancels out
        double delta = (i % 2 == 0) ? std::numbers::pi / 2 : -std::numbers::pi / 2;
        auton.turnTo(normalizeAngle(start.yaw + delta));
        //let the robot fully settle
        pros::delay(250); 

        //heading after the turn (rad)
        YawResult end = odom.getYaw();
        if (end.error != OdomError::NONE) {
                pros::lcd::print(0, "[TrackWidth] IMU Failure!");
                return 0.0;
        }

        //signed heading change, wrapped to [-pi, pi]
        double dTheta = angleDiffRad(end.yaw, start.yaw);

        //distance each side traveled during the turn (in) with opposite signs when turning in place
        double dLeft  = sideInches(leftMotors);
        double dRight = sideInches(rightMotors);

        //add this trial to the least-squares sums
        sumDiffTimesTheta += (dLeft - dRight) * dTheta;
        sumThetaSquared += dTheta * dTheta;

        //single-trial estimate of L, just for debugging
        double trialL = (dLeft - dRight) / dTheta;
        printf("trial %d: dLeft=%.3f dRight=%.3f dTheta=%.4f rad -> L=%.3f in\n",
                i, dLeft, dRight, dTheta, trialL);

        //show progress on the brain screen
        pros::lcd::print(1, "Trial %d/%d", i + 1, iterations);
        pros::lcd::print(2, "dLeft=%.2f dRight=%.2f", dLeft, dRight);
        pros::lcd::print(3, "dTh=%.3f rad L=%.3f", dTheta, trialL);
        if (sumThetaSquared > 1e-9) {
            pros::lcd::print(4, "Running L=%.4f in", sumDiffTimesTheta / sumThetaSquared);
        }

    }

    //final least-squares estimate
    double trackWidth = (sumThetaSquared > 1e-9) ? sumDiffTimesTheta / sumThetaSquared : 0.0;
    printf("Effective track width L: %.4f in\n", trackWidth);
    pros::lcd::print(5, "FINAL L=%.4f in", trackWidth);
    return trackWidth;
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
