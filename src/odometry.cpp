#include "main.h"
#include "odometry.h"
#include "constants.h"
#include "subsystems.h"

#include "stdlib.h"


pros::Rotation LTWheel(LEFT_TRACKING_WHEEL_PORT);
pros::Rotation RTWheel(RIGHT_TRACKING_WHEEL_PORT);
pros::Rotation BTWheel(BACK_TRACKING_WHEEL_PORT);

pros::IMU imu(IMU_PORT);

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
	// pros::lcd::print(1, "Heading = %lf, Target = %lf", heading, target);
	double turn_error = 0, turn_total_error = 0, turn_derivative = 0, turn_prev_error = 0, turn_PID = 0;
	
	while (correctCount <= 10) {
		
		heading = get_yaw_quaternion() - 180;
		optimized_angle = target - heading;
			
		//pros::lcd::print(1, "Heading = %lf, Target = %lf", heading, target);

		// if(target > heading){
		// 	temp_angle = -360 - heading + target;
		// 	if(abs(optimized_angle) > abs(temp_angle))
		// 		optimized_angle = temp_angle;
		// } else {
		// 	temp_angle = 360 - heading + target;
		// 	if(abs(optimized_angle) > abs(temp_angle))
		// 		optimized_angle = temp_angle;
		// }

		if (optimized_angle > 180) optimized_angle -= 360;
		else if (optimized_angle < -180) optimized_angle += 360;
		if (optimized_angle == 180) optimized_angle = 179.99;


		// proportion
		turn_error = optimized_angle;

		//pros::lcd::print(3, "Optimized = %lf", optimized_angle);

		// integral
		turn_total_error += turn_error;

		// derivative
		turn_derivative = turn_error - turn_prev_error;
		
		// get prev error for next instance
		turn_prev_error = turn_error;

		turn_PID = ((TURN_KP * turn_error) / 360);
		//turn_PID += copysign(0.12 + (weightAdjustment * 0.05), turn_PID);
		turn_PID += (TURN_KD * turn_derivative);
		if (abs(turn_total_error) < 2000) {
			turn_PID += (TURN_KI * turn_total_error);
		}

		//pros::lcd::print(2, "PID = %lf", turn_PID);

		int turnSpeed = turn_PID * 50;
	
		if(abs(optimized_angle) > 0.2) {
			turnSpeed = std::clamp(std::abs(turnSpeed), 2, 50);
			// leftMotors.move((int)copysign(turnSpeed, turn_PID));
			// rightMotors.move(-(int)copysign(turnSpeed, turn_PID));	

			leftMotors.move_velocity((int)copysign(turnSpeed, turn_PID));
			rightMotors.move_velocity(-(int)copysign(turnSpeed, turn_PID));

		}
		// pros::lcd::print(4, "Statement: %d", abs(optimized_angle) > 0.2);
		
		if(abs(optimized_angle) <= 0.2) {
			correctCount++;
		}

		//pros::lcd::print(4, "Correct Count: %d", correctCount);
		// pros::lcd::print(5, "Cur angle: %lf", heading);
		// pros::lcd::print(6, "Turnspeed: %d", turnSpeed);
		//pros::lcd::print(7, "Optimized angle: %lf", optimized_angle);
		// pros::lcd::print(7, "Move value: %d", (int)copysign(turnSpeed, turn_PID));
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
	const auto timer_duration = std::chrono::seconds(timer);

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

void move_dist_pid(double targetDistance, int speed, int timer) {
	int correctCount = 0;
	double move_error = 0, move_total_error = 0, move_derivative = 0, move_prev_error= 0, move_PID = 0;
	
	// Alter target to be relative position as origin
	update_position_and_angle();
	Position init_pos(pos_x, pos_y);

	const auto start_time = std::chrono::steady_clock::now();
	const auto timer_duration = std::chrono::seconds(timer);

	while (correctCount <= 5) {

		update_position_and_angle();
		double dist = getDistance(init_pos, { pos_x, pos_y });

		
		if (timer > 0 && ((std::chrono::steady_clock::now() - start_time) > timer_duration)) {
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
			// leftMotors.move(copysign(speed, move_PID));
			// rightMotors.move(copysign(speed, move_PID));

			leftMotors.move_velocity(speed);
			rightMotors.move_velocity(speed);

		} else if(dist > 0.15) {

			leftMotors.move_velocity(speed);
			rightMotors.move_velocity(speed);

			// leftMotors.move(move_PID * speed);
			// rightMotors.move(move_PID * speed);
		} 
		
		if(std::abs(targetDistance - dist) <= 1) {
			correctCount++;
		}

		pros::delay(10);
	}

	leftMotors.move(0);
	rightMotors.move(0);

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
	double dx_local = (arcs.left + arcs.right) / 2.0;
	double dy_local = arcs.back - (del_theta * BACK_TRACKING_WHEEL_DISTANCE);

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
	int currLeft = LTWheel.get_position();
	int currRight = RTWheel.get_position();
	int currBack = BTWheel.get_position();

	// Convert centidegrees to degrees and find distance travelled by wheel
	double del_L = (currLeft / 36000.0) * TRACKING_WHEEL_DIAMETER * std::numbers::pi; 
	double del_R = (currRight / 36000.0) * TRACKING_WHEEL_DIAMETER * std::numbers::pi; 
	double del_B = (currBack / 36000.0) * BACK_TRACKING_WHEEL_DIAMETER * std::numbers::pi;

	// Create a structure of lengths
	ArcLengths del(del_L, del_R, del_B);

	// Reset tracking for next measurements
	LTWheel.reset_position();
	RTWheel.reset_position();
	BTWheel.reset_position();

	return del;
}

double compute_heading_change(ArcLengths arcs) {
	// Calculate heading in radians
	return (arcs.right - arcs.left) / (double) (RIGHT_TRACKING_WHEEL_DISTANCE + LEFT_TRACKING_WHEEL_DISTANCE);
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
