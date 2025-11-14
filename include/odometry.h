/*
 * \file odometry.h
 *
 *  Contains function declarations and constants for odometry
 */

#ifndef ODOMETRY_H
#define ODOMETRY_H

// Longitudinal position (inches)
extern double pos_x;

// Lateral position (inches)
extern double pos_y;

// Orientation (radians) (0 - 2pi)
extern double theta;

// Previous Orientation (radians) (0 - 2pi)
extern double prevTheta;

// Tracking Wheels
extern pros::Rotation LTWheel;
extern pros::Rotation RTWheel;
extern pros::Rotation BTWheel;

// IMU Sensor
extern pros::IMU imu;


// Struct for holding lengths travelled by wheels
typedef struct ArcLengths {
    double left;
    double right;
    double back;
} ArcLengths;

// Calculate and update global position and orientation variables
void update_position_and_angle(void);

// Get arc lengths travelled and update encoder values
ArcLengths get_wheel_travel(void);

// Return change in heading based on arc lengths travelled
double compute_heading_change(ArcLengths arcs);

// Convert degrees to radians
double convertDegToRad(double degrees);

#endif