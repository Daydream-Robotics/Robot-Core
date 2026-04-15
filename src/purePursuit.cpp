#include "purePursuit.hpp"
#include "helpers.hpp"
#include <cmath>
#include "odometry.hpp"
#include "subsystems.h"
#include "arclengthSplining.hpp"
#include "main.h"

PurePursuit::PurePursuit(ALS_Path& als_path) 
    : velocityPID(PurPur_KP, PurPur_KI, PurPur_KD, 0.0), als_path(als_path) {
}


bool PurePursuit::step() {
    // Safety check to prevent a data abort if the path is empty/invalid
    if (!als_path.isValid() || als_path.getSamples().empty()) {
        leftMotors.move_velocity(0);
        rightMotors.move_velocity(0);
        return true;
    }

    odom.updatePose();
    
    double cur_x = odom.pos_x;
    double cur_y = odom.pos_y;
    double cur_heading_deg = odom.getYaw() - 180.0; // Remove the 180 degree offset from getYaw()
    
    double distFromEnd = als_path.getTotalLength() - als_path.getSamples()[als_path.findClosestSampleIndex({cur_x, cur_y}, 0, -1)].s;

    if (distFromEnd < END_TOLERANCE) {
        leftMotors.move_velocity(0);
        rightMotors.move_velocity(0);
        return true;
    }

    // update dynamic lookahead
    lookAheadDist = getLookaheadDist();

    // get target point coords local to the robot
    Position targetPoint = als_path.returnLookaheadPoint({cur_x, cur_y}, lookAheadDist);
    Position robotFrameTargetPt = convertPtToRobotFrame(targetPoint);

    // Using actual distance to target ensures accurate curvature regardless of point sparsity
    double curvature = calculateCurvature(robotFrameTargetPt);

    // Throttle base velocity based on the actual lookahead curvature
    int base_vel = getBaseVelocity(curvature);

    // set up pid here later

    int left_vel = base_vel + (curvature * base_vel * TURN_RATE); // Positive curvature means target is to the RIGHT, so left wheel goes faster
    int right_vel = base_vel - (curvature * base_vel * TURN_RATE);

    
    if (stepCounter % 10 == 0) {
        double current_vel = odom.getParallelVel();
        pros::lcd::print(5, "Velocity: %.2lf in/s", current_vel);
        pros::lcd::print(6, "LX %.2lf, LY %.2lf", robotFrameTargetPt.x, robotFrameTargetPt.y);
        pros::lcd::print(1, "Cur: %lf", curvature);
        pros::lcd::print(2, "VEL: %d", base_vel);

        printf("[PP] Pos:(%.2f, %.2f) H:%.2f | Vel:%.2f | LookAhead:%.2f | TgtGlobal:(%.2f, %.2f) TgtLocal:(%.2f, %.2f) | Curv:%.4f | Vels: B:%d L:%d R:%d\n",
               cur_x, cur_y, cur_heading_deg, current_vel, lookAheadDist, targetPoint.x, targetPoint.y,
               robotFrameTargetPt.x, robotFrameTargetPt.y, curvature, base_vel, left_vel, right_vel);
    }
    stepCounter++;

    rightMotors.move_velocity(right_vel);
    leftMotors.move_velocity(left_vel);

    return false;
}


// calculates the curvature of a point within the robot frame
double PurePursuit::calculateCurvature(Position robotFrameTargetPt) {
    double actual_dist = calcDistBetweenPoints({0, 0}, robotFrameTargetPt);
    
    double curvature = 0.0;
    if (actual_dist > 0.01) {
        curvature = (2.0 * robotFrameTargetPt.y) / (actual_dist * actual_dist); // Use lateral distance (Y) instead of forward distance (X)
    }

    return curvature;
}


// converts a target point to a point local to the robot's frame of reference
Position PurePursuit::convertPtToRobotFrame(Position targetPoint) {
    double robot_x = odom.pos_x;
    double robot_y = odom.pos_y;
    double robot_heading_deg = odom.getYaw() - 180.0; // Remove the 180 degree offset from getYaw()
    
    double dx = targetPoint.x - robot_x;
    double dy = targetPoint.y - robot_y;

    // localize displacements relative to orientation of robot
    double robot_heading_rad = convertDegToRad(robot_heading_deg);
    double localX = dx * cos(robot_heading_rad) + dy * sin(robot_heading_rad);
    double localY = dx * (-sin(robot_heading_rad)) + dy * cos(robot_heading_rad);

    return {localX, localY};
}


// gets the dynamic lookahead distance based off of the forward velocity of the robot.
double PurePursuit::getLookaheadDist() {
    double vel = std::abs(odom.getParallelVel());
    // pros::lcd::print(5, "Velocity: %lf in/s", vel); // Commented out to prevent LVGL crashes
    double dynamicLookahead = std::clamp(LOOKAHEAD_SECONDS * vel, MIN_LOOKAHEAD_DIST, MAX_LOOKAHEAD_DIST);
    // printf("Dynamic Lookahead: %lf\n", dynamicLookahead);
    return dynamicLookahead;
}


int PurePursuit::getBaseVelocity(double curvature) {
    int base_vel = MAX_BASE_VEL / (1 + (std::abs(curvature) * SPEED_ADJUSTMENT_CONST));
    return std::clamp(base_vel, MIN_BASE_VEL, MAX_BASE_VEL);
}