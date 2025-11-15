/*
 * \file odometry.h
 *
 *  Contains function declarations and constants for odometry
 */

#ifndef ODOMETRY_H
#define ODOMETRY_H

#include <cmath>

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

typedef struct Position {
    double x;
    double y;
} Position;

// Calculate and update global position and orientation variables
void update_position_and_angle(void);

// Normalize the angle btwn pi and -pi
double normalizeAngle(double a);

// Get arc lengths travelled and update encoder values
ArcLengths get_wheel_travel(void);

// Return change in heading based on arc lengths travelled
double compute_heading_change(ArcLengths arcs);

// Convert degrees to radians
double convertDegToRad(double degrees);

// Convert radians to degrees
double convertRadToDeg (double rad);

// Find distance between two positions
double getDistance (Position p1, Position p2);

// Turn to specified target angle
void turn_pid(double target, double weightAdjustment);

// Move to specified position
void move_pid(Position target, double weightAdjustment);

// Get yaw from imu
double get_yaw_quaternion();

#endif