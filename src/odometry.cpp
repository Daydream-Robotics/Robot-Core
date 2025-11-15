#include "main.h"


// Get raw yaw/quat value
double getYawQuaternion() {
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

void turnPID(double target, double weightAdjustment = 0) {
	int correctCount = 0;
	heading = getYawQuaternion();
	// pros::lcd::print(1, "Heading = %lf, Target = %lf", heading, target);
	
	while (correctCount <= 10) {
		
		heading = getYawQuaternion() - 180;
		optimized_angle = target - heading;
			
		pros::lcd::print(1, "Heading = %lf, Target = %lf", heading, target);

		if(target > heading){
			temp_angle = ((360 - target) + heading) * -1;
			if(abs(optimized_angle) > abs(temp_angle))
				optimized_angle = temp_angle;
		}
		else {
			temp_angle = (360 - heading) + target;
			if(abs(optimized_angle) > abs(temp_angle))
				optimized_angle = temp_angle;
		}

		// proportion
		turn_error = optimized_angle;

		pros::lcd::print(3, "Optimized = %lf", optimized_angle);

		// integral
		turn_total_error += turn_error;

		// derivative
		turn_derivative = turn_error - turn_prev_error;
		
		// get prev error for next instance
		turn_prev_error = turn_error;

		turn_PID = ((turn_kP * turn_error) / 360);
		turn_PID += copysign(0.12 + (weightAdjustment * 0.05), turn_PID); // want to remove

		pros::lcd::print(2, "PID = %lf", turn_PID);
	
		if(abs(optimized_angle) > 0.15) {
			leftMotors.move(turn_PID * 127);
			rightMotors.move(-turn_PID * 127);
		} else if(abs(turn_PID) <= 0.4) {
			leftMotors.move(copysign(0.4, turn_PID * 127));
			rightMotors.move(copysign(0.4, -turn_PID * 127));
		} else {
			leftMotors.move(0);
			rightMotors.move(0);
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