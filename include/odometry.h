// /*
//  * \file odometry.h
//  *
//  *  Contains function declarations and constants for odometry
//  */

// #ifndef ODOMETRY_H
// #define ODOMETRY_H

// #include <cmath>
// #include <chrono>

// // Struct for holding lengths travelled by wheels
// typedef struct ArcLengths {
//     double parallel;
//     double perpendicular;
// } ArcLengths;

// typedef struct Position {
//     double x;
//     double y;
// } Position;

// // Calculate and update global position and orientation variables
// void update_position_and_angle(void);

// // Normalize the angle btwn pi and -pi
// double normalizeAngle(double a);

// // Get arc lengths travelled and update encoder values
// ArcLengths get_wheel_travel(void);

// // Return change in heading based on arc lengths travelled
// double compute_heading_change(ArcLengths arcs);

// // Convert degrees to radians
// double convertDegToRad(double degrees);

// // Convert radians to degrees
// double convertRadToDeg (double rad);

// // Find distance between two positions
// double getDistance (Position p1, Position p2);

// // Turn to specified target angle
// void turn_pid(double target, double weightAdjustment);

// // Move to specified position (if backward == 1, then reverse instead) (if timer == -1, then there is no timer)
// void move_pid(Position target, int speed, double weightAdjustment, int backwards, int timer);

// // Move a specified distance
// void move_dist_pid(double targetDistance, int speed, int timer, bool backwards);

// // Get yaw from imu
// double get_yaw_quaternion();

// // Move a specified distance and direction
// void travelDistanceWithHeading(double distance, double speed, double target_heading, int timer);

// // Helper: shortest difference a - b normalized
// double angleDiffDeg(double a, double b);

// #endif