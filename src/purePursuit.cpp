#include "purePursuit.hpp"
#include "helpers.hpp"
#include <cmath>
#include "odometry.hpp"
#include "subsystems.h"
#include "arclengthSplining.hpp"
#include "main.h"
#include "sd_card_logging.hpp"

PurePursuit::PurePursuit(ALS_Path& als_path) 
    : m_velocityPID(PurPur_KP, PurPur_KI, PurPur_KD, 0.0), m_als_path(als_path) {
    m_ghostPoint = updateGhostPoint();
}

bool PurePursuit::step() {
    // Safety check to prevent a data abort if the path is empty/invalid
    if (!m_als_path.isValid() || m_als_path.getSamples().empty()) {
        leftMotors.move_velocity(0);
        rightMotors.move_velocity(0);
        return true;
    }

    odom.updatePose();
    
    double cur_x = odom.pos_x;
    double cur_y = odom.pos_y;
    double cur_heading_deg = odom.getYaw() - 180.0; // Remove the 180 degree offset from getYaw()
    
    // double current_s = als_path.getSamples()[als_path.findClosestSampleIndex({cur_x, cur_y}, 0, -1)].s;
    m_lastPassedPtIdx = m_als_path.findClosestSampleIndex({cur_x, cur_y}, m_lastPassedPtIdx, -1);
    Sample curSample = m_als_path.getSamples()[m_lastPassedPtIdx];
    double current_s = curSample.s;
    
    m_distFromEnd = m_als_path.getTotalLength() - current_s;

    if (m_distFromEnd < END_TOLERANCE) {
        leftMotors.move_velocity(0);
        rightMotors.move_velocity(0);
        return true;
    }

    // update dynamic lookahead
    m_lookAheadDist = getLookaheadDist();

    // get target point coords local to the robot
    Position targetPoint = m_als_path.returnLookaheadPoint({cur_x, cur_y}, m_lookAheadDist);
    
    // if dist to end is less than lookahead, use ghost point to prevent aggressive braking
    if (m_distFromEnd < m_lookAheadDist) {
        targetPoint = m_ghostPoint;
    }
    Position robotFrameTargetPt = convertPtToRobotFrame(targetPoint);

    // Using actual distance to target ensures accurate curvature regardless of point sparsity
    double steeringCurvature = calculateCurvature(robotFrameTargetPt);

    // Look ahead along the path to find the sharpest upcoming curve
    double pathMaxCurvature = m_als_path.getMaxAbsCurvatureInRange(current_s, current_s + m_lookAheadDist);

    // Throttle base velocity based on the sharpest upcoming curve
    int base_vel = getBaseVelocity(pathMaxCurvature);
    bool sharpTurn = (pathMaxCurvature * m_lookAheadDist) > (M_PI / 4.0);
    if (!sharpTurn) {
        base_vel =std::copysign(base_vel, robotFrameTargetPt.x); // If target is behind, reverse direction
    }
    
    
    //copysign is used for determining if robot is going fwds or backwards
    // !IMPORTANT! If this doesnt work, flip the sign of curvature
    double left_target = base_vel + (steeringCurvature * base_vel * TURN_RATE); // Positive curvature means target is to the RIGHT, so left wheel goes faster
    double right_target = base_vel - (steeringCurvature * base_vel * TURN_RATE);

    // Maintain the turn ratio if the requested velocity exceeds the motor's physical limit
    double max_req = std::max(std::abs(left_target), std::abs(right_target));
    if (max_req > 200.0) {
        left_target = left_target * (200.0 / max_req);
        right_target = right_target * (200.0 / max_req);
    }

    int left_vel = static_cast<int>(left_target);
    int right_vel = static_cast<int>(right_target);

    
    if (m_stepCounter % 10 == 0) {
        double current_vel = odom.getParallelVel();
        pros::lcd::print(5, "Velocity: %.2lf in/s", current_vel);
        pros::lcd::print(6, "LX %.2lf, LY %.2lf", robotFrameTargetPt.x, robotFrameTargetPt.y);
        pros::lcd::print(1, "Cur: %lf", steeringCurvature);
        pros::lcd::print(2, "VEL: %d", base_vel);

        m_lastPassedPtIdx;
        double distFromLine = std::hypot(curSample.x - cur_x, curSample.y - cur_y);
        m_totalDistOff += distFromLine;
        printf("[PP] Pos:(%.2f, %.2f) H:%.2f | Vel:%.2f | LookAhead:%.2f | TgtGlobal:(%.2f, %.2f) TgtLocal:(%.2f, %.2f) | Curv:%.4f | Vels: B:%d L:%d R:%d | OffAtStep: %.4f | AvgDistOff: %.4f\n",
               cur_x, cur_y, cur_heading_deg, current_vel, m_lookAheadDist, targetPoint.x, targetPoint.y,
               robotFrameTargetPt.x, robotFrameTargetPt.y, steeringCurvature, base_vel, left_vel, right_vel, distFromLine, m_totalDistOff/m_stepCounter);
    }
    m_stepCounter++;


    rightMotors.move_velocity(right_vel);
    leftMotors.move_velocity(left_vel);

    return false;
}


// calculates the curvature of a point within the robot frame
double PurePursuit::calculateCurvature(Position robotFrameTargetPt) {
    double actual_dist = calcDistBetweenPoints({0, 0}, robotFrameTargetPt);
    
    double curvature = 0.0;
    if (actual_dist < 8.0) {
        actual_dist = 8.0;
    }

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
    int min_base_adjusted = MIN_BASE_VEL;

    // Further reduce speed if we're close to the end of the path to prevent overshooting
    if (m_distFromEnd < END_SLOWDOWN_THRESH) {
        base_vel = static_cast<int>(base_vel * (m_distFromEnd / END_SLOWDOWN_THRESH));
        min_base_adjusted = 0; // Allow full stop when within the slowdown threshold
    }

    return std::clamp(base_vel, min_base_adjusted, MAX_BASE_VEL);
}

Position PurePursuit::updateGhostPoint() {
    std::vector<Sample> samples = m_als_path.getSamples();

    Sample lastPoint = samples.back();
    
    double heading = lastPoint.heading;
    
    double ux = std::cos(convertDegToRad(heading));
    double uy = std::sin(convertDegToRad(heading));

    Position ghostPoint = {lastPoint.x + END_GHOST_CAST * ux, lastPoint.y + END_GHOST_CAST * uy};

    return ghostPoint;
}