#include "autonomous.hpp"
#include "subsystems.hpp"
#include "constants.h"
#include "helpers.hpp"
#include <cmath>
#include <numbers>


	

// TODO: Tune PID parameters
Autonomous::Autonomous() 
	: distancePID(DISTANCE_KP, DISTANCE_KI, DISTANCE_KD, DISTANCE_KI_THRESHOLD),  // 5.0, 2.0, 0.0, 1.0
	headingPID(HEADING_KP, HEADING_KI, HEADING_KD, HEADING_KI_THRESHOLD), // 0.002, 0.0, 0.0, 0.0
	turnPID(TURN_KP, TURN_KI, TURN_KD, TURN_KI_THRESHOLD) { // 1.22, 0.000, 0.063875, 180
		leftMotors.set_brake_mode_all(pros::E_MOTOR_BRAKE_HOLD);
		rightMotors.set_brake_mode_all(pros::E_MOTOR_BRAKE_HOLD);
	}

void Autonomous::turnTo(double targetHeading) {
	turnPID.reset();
    turnPID.setTarget(targetHeading);

	// pros::lcd::print(0, "Turning to %lf degrees", targetHeading);

	// TODO: Tune exit conditions
    turnPID.exit_condition_set(
        0.3, 75,     // small error (deg), time (ms)
        0.5, 100,     // big error (deg), time
       0.5 ,1000000,          // velocity settle time
        0          // timeout
    );

	// TODO: Tune alpha
    HeadingFilter headingFilter(0.3);

		// Initialize clocking
	using clock = std::chrono::steady_clock;
    auto lastTime = clock::now();
	double prevHeading = odom.getYaw();

	while (true) {
		// Heading calculation
		double rawHeading = odom.getYaw();
		odom.updatePose();

		if (rawHeading < 180.0) {
			pros::lcd::print(0, "[TurnTo] IMU Failure! YAW: %lf", rawHeading);
			// TODO: Add more verbose error handling
			return;
		}

		// Calculate angular velocity
		auto now = clock::now();
		std::chrono::duration<double> dt_dur = now - lastTime;
		double dt = dt_dur.count();
        if (dt < 0.001) dt = 0.001; // Prevent division by zero
		lastTime = now;
		
		// Determine PID correction using smoothed heading
		double filteredHeading = headingFilter.update(rawHeading);
		double correction = turnPID.compute(filteredHeading,true);
		//pros::lcd::print(1, "Correction: %lf", correction);
		
		// Compute turnSpeed based on correction
		double turnSpeed = std::clamp(
			std::fabs(correction),
            1.0, // 2
            100.0 // 70
        );
		
        turnSpeed = std::copysign(turnSpeed, correction);
		
		leftMotors.move_velocity(turnSpeed);
        rightMotors.move_velocity(-turnSpeed);
		
		double currentVelocity = (std::fabs(angleDiffDeg(targetHeading, filteredHeading)) < 1.0) ? angleDiffDeg(rawHeading, prevHeading) / dt : 999.0;
		if (turnPID.exit_condition(currentVelocity) != PID::RUNNING)
			break;
		prevHeading = rawHeading;
		
		pros::delay(10);
	}

	// pros::lcd::print(2, "Exited Turn Loop");

	leftMotors.move_velocity(0);
    rightMotors.move_velocity(0);
    pros::delay(10);
}


double Autonomous::travel(double distance, double speed, double targetHeading, double timer_s) {
	double traveled = 0;

    auto clamp = [](double v, double lo, double hi) {
        return (v < lo) ? lo : (v > hi) ? hi : v;
    };

    auto normalizeDeg = [](double a) {
        while (a >= 180.0) a -= 360.0;
        while (a < -180.0) a += 360.0;
        return a;
    };
	// int count = 0;
    distancePID.setTarget(distance);
    distancePID.exit_condition_set(
        0.1, 10,
        0.4, 30,
        0.5, 200000000,
        timer_s * 1000
    );

    headingPID.setTarget(0.0);

    Position start = odom.getPosition();
    double direction = (distance >= 0.0) ? 1.0 : -1.0;

	double headingRad = convertDegToRad(targetHeading);
	Position headingUnit {
		std::cos(headingRad),
		std::sin(headingRad)
	};

    double prevVelocity = 0.0;
    double prevDistance = 0.0;

	using clock = std::chrono::steady_clock;
    auto startTime = clock::now();
	auto lastTime = clock::now();

    while (true) {
		// get elapesed time since last loop
		auto now = clock::now();
		std::chrono::duration<double> dt_dur = now - lastTime;
		double dt = dt_dur.count();
        if (dt < 0.001) dt = 0.001; // Prevent division by zero
		lastTime = now;

        odom.updatePose();
		
		// if (pos_x < -1) {
		// 	pros::lcd::print(7, "OUT OF BOUNDS!");
		// 	break;
		// }

		// controller.print(0,0, "%.2f, %.2f", pos_x, pos_y);
        // Compute traveled distance along heading vector
        Position delta { odom.getPosX() - start.x, odom.getPosY() - start.y };
		// printf("X: %.2f, Y: %.2f\n", odom.pos_x, odom.pos_y);

		traveled = delta.x * headingUnit.x + delta.y * headingUnit.y;

        double v = distancePID.compute(traveled);
        v = clamp(v, -speed, speed);

		// limiter during accel
		double currentAccelLimit = accelLimitRate;
        if (std::fabs(v) < std::fabs(prevVelocity)) {
			currentAccelLimit = 500.0; // Allow faster deceleration
        }
		v = accelLimit(prevVelocity, v, dt, currentAccelLimit);
        prevVelocity = v;

        // Heading error
        double rawHeading = odom.getYaw();
        if (rawHeading < 180.0) {
            pros::lcd::print(0, "[Travel] IMU Failure!");
            break;
        }

        double headingError = normalizeDeg(targetHeading - rawHeading);

        // Heading correction
		// double omega = headingPID.compute(headingError) * (std::fabs(v) / speed);
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

		// Exit if any exit condition is met.
		// Ignore velocity exit for the first second to allow robot to accelerate
		auto elapsed_ms = std::chrono::duration_cast<std::chrono::milliseconds>(now - startTime).count();
		double currVel = (elapsed_ms > 1000) ? (traveled - prevDistance) / dt : 999.0;
		PID::ExitState exitState = distancePID.exit_condition(currVel);
        if (exitState != PID::RUNNING){
			// print exit condition
			pros::lcd::print(0,"Exit Condition Meet");
			printf("Exit Condition: %d\n", exitState);
            break;
		}
        prevDistance = traveled;
		//   controller.print(0,0, "%.2f", rawHeading);
		// count++;
        pros::delay(10);
    }

    leftMotors.move_velocity(0);
    rightMotors.move_velocity(0);
    pros::delay(10);
	return traveled;
}

bool Autonomous::travelToPoint(double targetX, double targetY, double maxSpeed, bool reverse, int timer) {
	odom.updatePose();
	Position start = odom.getPosition();
	double dx = targetX - start.x;
	double dy = targetY - start.y;
	
	double distance = std::hypot(dx, dy); //euclidean distance from start to end point	
	double targetHeading = std::atan2(dy, dx) * 180.0  / std::numbers::pi;
	
	if (reverse) {
		targetHeading += 180;
		if (targetHeading > 180) targetHeading -= 360;
		
		distance = -distance;
	}

	turnTo(targetHeading);
	travel(distance, maxSpeed, targetHeading, timer);

	pros::delay(10);

	return true;
}