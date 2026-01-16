#include "main.h"
#include "odometry.h"
#include "constants.h"
#include "subsystems.h"

#include "stdlib.h"

double pos_x = 0;
double pos_y = 0;
double prevTheta;
double theta = 0;
double heading;
double optimized_angle;
double temp_angle;


void turn_pid(double target, double weightAdjustment) {
	int correctCount = 0;
	heading = get_yaw_quaternion();
	double turn_error = 0, turn_total_error = 0, turn_derivative = 0, turn_prev_error = 0, turn_PID = 0;
	
	while (correctCount <= 10) {
		
		heading = get_yaw_quaternion() - 180;
		optimized_angle = target - heading;

		if (optimized_angle > 180) optimized_angle -= 360;
		else if (optimized_angle < -180) optimized_angle += 360;
		if (optimized_angle == 180) optimized_angle = 179.99;


		// proportion
		turn_error = optimized_angle;

		// integral
		turn_total_error += turn_error;

		// derivative
		turn_derivative = turn_error - turn_prev_error;
		
		// get prev error for next instance
		turn_prev_error = turn_error;

		turn_PID = ((TURN_KP * turn_error) / 360);
		turn_PID += (TURN_KD * turn_derivative);
		if (abs(turn_total_error) < 2000) {
			turn_PID += (TURN_KI * turn_total_error);
		}

		int turnSpeed = turn_PID * 65;
	
		if(abs(optimized_angle) > 0.2) {
			turnSpeed = std::clamp(std::abs(turnSpeed), 2, 65);

			leftMotors.move_velocity((int)copysign(turnSpeed, turn_PID));
			rightMotors.move_velocity(-(int)copysign(turnSpeed, turn_PID));

		}
		
		if(abs(optimized_angle) <= 0.2) {
			correctCount++;
		}

		pros::delay(10);
	}

	leftMotors.move(0);
	rightMotors.move(0);

	pros::delay(250);
	
}

void move_pid(Position target, int speed, double weightAdjustment, int backwards, int timer) {
	int correctCount = 0;
	double move_error = 0, move_total_error = 0, move_derivative = 0, move_prev_error= 0, move_PID = 0;
	
	// Alter target to be relative position as origin
	update_position_and_angle();
	double del_x = target.x - pos_x;
	double del_y = target.y - pos_y;

	if (backwards) {
		del_x *= -0.999;
		del_y *= -0.999;
	}

	// Get angle to target
	double target_heading = std::atan2(del_y, del_x);
    
	const auto start_time = std::chrono::steady_clock::now();
	const auto timer_duration = std::chrono::milliseconds(timer);

	if (!backwards){
		// turn to the specified angle
		turn_pid(convertRadToDeg(target_heading), 0);
	}

	pros::lcd::print(1, "Left Turn_PID");
	
	while (correctCount <= 2) {

		update_position_and_angle();
		double dist = getDistance({ pos_x, pos_y }, target);

		if (timer > 0 && ((std::chrono::steady_clock::now() - start_time) > timer_duration)) {
			return;
		}

		// proportion
		move_error = dist;

		// integral
		move_total_error += move_error;

		// derivative
		move_derivative = move_error - move_prev_error;
		
		// get prev error for next instance
		move_prev_error = move_error;

		move_PID = (MOVE_KP * move_error) * (backwards ? -1 : 1);
		move_PID += copysign(0.12 + (weightAdjustment * 0.05), move_PID);
	
		if(abs(move_PID) >= 0.5) {
			leftMotors.move(copysign(speed, move_PID));
			rightMotors.move(copysign(speed, move_PID));
		} else if(dist > 0.15) {
			leftMotors.move(move_PID * speed);
			rightMotors.move(move_PID * speed);
		} 
		
		if(dist <= 4) {
			correctCount++;
		}

		pros::delay(10);
	}

	leftMotors.move(0);
	rightMotors.move(0);

	pros::delay(250);
	
}

void move_dist_pid(double targetDistance, int speed, int timer, bool backwards) {
	int correctCount = 0;
	double move_error = 0, move_total_error = 0, move_derivative = 0, move_prev_error= 0, move_PID = 0;
	
	// Alter target to be relative position as origin
	update_position_and_angle();
	Position init_pos(pos_x, pos_y);

	const auto start_time = std::chrono::steady_clock::now();
	const auto timer_duration = std::chrono::milliseconds(timer);

	while (correctCount <= 5) {

		update_position_and_angle();
		double dist = getDistance(init_pos, { pos_x, pos_y });

		
		if (timer > 0 && ((std::chrono::steady_clock::now() - start_time) > timer_duration)) {
			leftMotors.move_velocity(STOP);
			rightMotors.move_velocity(STOP);

			pros::delay(250);
			return;
		}
		

		// proportion
		move_error = std::abs(targetDistance - dist);

		// integral
		move_total_error += move_error;

		// derivative
		move_derivative = move_error - move_prev_error;
		
		// get prev error for next instance
		move_prev_error = move_error;

		move_PID = (MOVE_KP * move_error);
		move_PID += copysign(0.12, move_PID);
	
		if(std::abs(move_PID) >= 0.5) {

			leftMotors.move_velocity(backwards ? -speed : speed);
			rightMotors.move_velocity(backwards ? -speed : speed);

		} else if(dist > 0.15) {

			leftMotors.move_velocity(backwards ? -speed : speed);
			rightMotors.move_velocity(backwards ? -speed : speed);
		} 
		
		if(std::abs(targetDistance - dist) <= 1) {
			correctCount++;
		}

		pros::delay(10);
	}

	leftMotors.move_velocity(STOP);
	rightMotors.move_velocity(STOP);

	pros::delay(250);

}

double get_yaw_quaternion() {
	pros::quaternion_s_t qt = imu.get_quaternion();

	//error fetching quat, retry
	if (qt.w == PROS_ERR_F) {
		qt = imu.get_quaternion();
		if (qt.w == PROS_ERR_F) {
			// pros::lcd::set_text(5, "ERROR: IMU Quaternion Fetch Failed");
			return -1.0;
		}
	}

	//convert quat to yaw
	double yaw = atan2(2 * ((qt.w * qt.z) + (qt.x * qt.y)), 1 - (2 * ((qt.y * qt.y) + (qt.z * qt.z)))); //yaw formula = atan2(2(wz + xy), 1 - 2(y^2 + z^2))

	//returns yaw converted from rad to deg; angle is returned from -180 to 180 (+ 180 for [0, 360])
	return ((yaw * (180 / M_PI)) + 180);
}

double normalizeAngle(double a) {
    // robust normalize using atan2(sin,cos)
    return std::atan2(std::sin(a), std::cos(a));
}

void update_position_and_angle() {
	// Calculate distance travelled by each tracking wheel
	ArcLengths arcs = get_wheel_travel();

	// Get orientation from IMU
	theta = convertDegToRad(get_yaw_quaternion() - 180);
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
}


ArcLengths get_wheel_travel() {

    // Get current centidegree position of tracking wheels
	int currParallel = parallelTrackingWheel.get_position();
	int currPerpendicular = perpendicularTrackingWheel.get_position();

	// Convert centidegrees to degrees and find distance travelled by wheel
	double delParallel = (currParallel / 36000.0) * PARALLEL_TRACKING_WHEEL_DIAMETER * std::numbers::pi; 
	double delPerpendicular = (currPerpendicular / 36000.0) * PERPENDICULAR_TRACKING_WHEEL_DIAMETER * std::numbers::pi; 

	// Create a structure of lengths
	ArcLengths del(delParallel, delPerpendicular);

	// Reset tracking for next measurements
	parallelTrackingWheel.reset_position();
	perpendicularTrackingWheel.reset_position();

	return del;
}

double convertDegToRad (double degree) {
	return degree * (std::numbers::pi / 180.0);
}

double convertRadToDeg (double rad) {
	return rad * (180.0 / M_PI);
}

double getDistance (Position p1, Position p2) {
	return std::sqrt(std::pow((p2.x - p1.x), 2) + std::pow((p2.y - p1.y), 2));
}

double angleDiffDeg(double a, double b) {
	double c = a - b;
	while (c > 180.0) c -= 360.0;
	while (c <= -180.0) c += 360.0;
	return c;
}

void travelDistanceWithHeading(double distance, double speed, double target_heading, int timer) {

	// PID controls
	double integrator = 0.0;
	double prev_error = 0.0;

	// Initialization takeoff variables
	double prev_forward = 0.0;
	double forward_accel_limit = 200.0; 
	double takeoff_ramp_time = 0.35;    // seconds to ramp-in heading correction
	double heading_ema_alpha = 0.35;    // EMA smoothing factor for heading (0..1), lower = smoother
	double heading_deadband = 0.5;      // degrees: ignore tiny heading errors at start
	double smoothed_heading = 0.0;
	bool smoothed_heading_initialized = false;

	// Initialize speed variables
	const double max_speed = std::fabs(speed) * 1.5 + 1.0;
	const double min_speed = -max_speed;

	// Deceleration parameters
	const double decel_distance = std::max(0.2, std::fabs(distance) * 0.15);
	const double stop_threshold = 0.5; // inches

	// Clocking
	using clock = std::chrono::steady_clock;
	auto last_t = clock::now();

	// Initialize timer if it exists
	const auto start_time = last_t;
	const auto timer_duration = std::chrono::milliseconds(timer);

	// Initial position
	Position start(pos_x, pos_y);
	Position current = start;

	double remaining = std::fabs(distance);
	double direction = (distance >= 0) ? 1.0 : -1.0;

	while (true) {
		auto now = clock::now();

		std::chrono::duration<double> elapsed_since_motion_start = now - start_time;
		double motion_t = elapsed_since_motion_start.count();
		
		// Check if timer has completed
		if (timer > 0 && (now - start_time) > timer_duration) {
			leftMotors.move_velocity(0);
			rightMotors.move_velocity(0);
			pros::delay(250);
			return;
		}
		
		// Calculate infintesimal time
		std::chrono::duration<double> elapsed = now - last_t;
		double dt = elapsed.count();
		// Clamping for small numbers
		if (dt <= 0) dt = 1e-3;
		last_t = now;

		update_position_and_angle();
		double traveled = getDistance(start, { pos_x, pos_y });
		remaining = std::fabs(distance) - traveled;
		// Reached destination
		if (remaining <= stop_threshold) break;

		double raw_heading = get_yaw_quaternion() - 180;
	
		if (!smoothed_heading_initialized) {
			smoothed_heading = raw_heading;
			smoothed_heading_initialized = true;
		} else {
			smoothed_heading = heading_ema_alpha * raw_heading + (1.0 - heading_ema_alpha) * smoothed_heading;
		}

		double heading_error = angleDiffDeg(target_heading, smoothed_heading);

		if (std::fabs(heading_error) < heading_deadband) {
			heading_error = 0.0;
		}

		// PID control
		integrator += heading_error * dt;
		integrator = std::clamp(integrator, -MOVE_HEADING_INTEGRATOR_LIMIT, MOVE_HEADING_INTEGRATOR_LIMIT);
		double deriv = angleDiffDeg(heading_error, prev_error) / dt;
		prev_error = heading_error;
		double raw_corr = MOVE_HEADING_KP * heading_error + MOVE_HEADING_KD * deriv + MOVE_HEADING_KI * integrator;

		double corr_ramp_factor = 1.0;
		if (motion_t < takeoff_ramp_time) {
			corr_ramp_factor = motion_t / takeoff_ramp_time; // 0 -> 1 linearly
		}
		double corr = raw_corr * corr_ramp_factor;

		// Scale down speed as we near target
		double speed_scale = 1.0;
		if (remaining < decel_distance) {
			speed_scale = std::clamp(remaining / decel_distance, 0.2, 1.0);
		}

		double target_forward = speed * direction * speed_scale;
		double max_delta = forward_accel_limit * dt;
		double forward_delta = target_forward - prev_forward;
		if (forward_delta > max_delta) forward_delta = max_delta;
		if (forward_delta < -max_delta) forward_delta = -max_delta;
		double forward = prev_forward + forward_delta;
		prev_forward = forward;

		// Determine wheel speeds
		double left_vel = forward + corr;
		double right_vel = forward - corr;
		left_vel = std::clamp(left_vel, min_speed, max_speed);
		right_vel = std::clamp(right_vel, min_speed, max_speed);

		leftMotors.move_velocity(left_vel);
		rightMotors.move_velocity(right_vel);

		pros::delay(10);
	}

	leftMotors.move_velocity(0);
	rightMotors.move_velocity(0);
	pros::delay(250);
	return;
}