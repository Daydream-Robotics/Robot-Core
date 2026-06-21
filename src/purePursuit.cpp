#include "purePursuit.hpp"
#include "helpers.hpp"
#include <cmath>
#include "odometry.hpp"
#include "subsystems.hpp"
#include "arclengthSplining.hpp"
#include "main.h"
#include "sd_card_logging.hpp"

PurePursuitController::PurePursuitController() {}


void PurePursuitController::reset() {
    m_totalDistOff = 0;
    m_stepCounter = 0;
}


WheelVelocities PurePursuitController::compute(const Pose& currentPose, const ALS_Path& als_path, std::size_t& closestSampleIdx, PathFlag flag) {

    Sample currentSample = als_path.getSamples()[closestSampleIdx];

    // update dynamic lookahead
    m_lookAheadDist = getLookaheadDist();
    
    Position targetPoint;

    // create virtual pose
    Pose virtualPose = currentPose;
    if (flag == PathFlag::REVERSE) {
        virtualPose.theta = angleDiffRad(currentPose.theta + M_PI, 0);
    }

    // if dist to end is less than lookahead, use ghost point to prevent aggressive braking
    m_distFromEnd = als_path.getTotalLength() - currentSample.s;
    if (m_distFromEnd < m_lookAheadDist) {
        // create ghost point
        Sample lastPoint = als_path.getSamples().back();
        targetPoint = {
            lastPoint.x + END_GHOST_CAST * std::cos(lastPoint.heading),
            lastPoint.y + END_GHOST_CAST * std::sin(lastPoint.heading)
        };
    } else {
        // get global target point coords
        double targetS = currentSample.s + m_lookAheadDist;
        if (targetS > als_path.getTotalLength()) {
            targetS = als_path.getTotalLength();
        }
        Waypoint targetWP = als_path.getPointAtArcLength(targetS);
        targetPoint = {targetWP.x, targetWP.y};
    }

    // translate coordinates to robot frame
    Position robotFrameTargetPt = convertPtToRobotFrame(targetPoint, virtualPose);

    // get the curvature to the target point
    double steeringCurvature = calculateCurvature(robotFrameTargetPt);

    // Look ahead along the path to find the sharpest upcoming curve
    double pathMaxCurvature = als_path.getMaxAbsCurvatureInRange(currentSample.s, currentSample.s + m_lookAheadDist);

    // Throttle base velocity based on the sharpest upcoming curve
    // Direction is selected externally for the whole path.
    // int base_vel = static_cast<int>(getBaseVelocity(pathMaxCurvature, m_speedAdjustmentConst) * velocityDirection);
    int base_vel = static_cast<int>(getBaseVelocity(pathMaxCurvature));

    if (flag == PathFlag::REVERSE) {
        base_vel = -base_vel;
    }
    
    // !IMPORTANT! If reverse tracking steers the wrong way, flip the sign of curvature
    double turn_vel = steeringCurvature * std::abs(base_vel) * TURN_RATE;
    double left_target = base_vel - turn_vel;
    double right_target = base_vel + turn_vel;

    // Maintain the turn ratio if the requested velocity exceeds the motor's physical limit
    double max_req = std::max(std::abs(left_target), std::abs(right_target));
    if (max_req > 600.0) {
        left_target = left_target * (600.0 / max_req);
        right_target = right_target * (600.0 / max_req);
    }


    if (m_stepCounter % 10 == 0) {
        double current_vel = odom.getParallelVel();
        // pros::lcd::print(5, "Velocity: %.2lf in/s", current_vel);
        // pros::lcd::print(6, "LX %.2lf, LY %.2lf", robotFrameTargetPt.x, robotFrameTargetPt.y);
        // pros::lcd::print(1, "Cur: %lf", steeringCurvature);
        // pros::lcd::print(2, "VEL: %d", base_vel);

        double distFromLine = std::hypot(currentSample.x - virtualPose.x, currentSample.y - virtualPose.y);
        m_totalDistOff += std::abs(distFromLine);
        // printf("[PP] Pos:(%.2f, %.2f) H:%.2f | Vel:%.2f | LookAhead:%.2f | Dir:%.0f | TgtGlobal:(%.2f, %.2f) TgtLocal:(%.2f, %.2f) | Curv:%.4f | Vels: B:%d L:%d R:%d | OffAtStep: %.4f | AvgDistOff: %.4f\n",
        //        cur_x, cur_y, cur_heading_deg, current_vel, m_lookAheadDist, velocityDirection, targetPoint.x, targetPoint.y,
        //        robotFrameTargetPt.x, robotFrameTargetPt.y, steeringCurvature, base_vel, left_vel, right_vel, distFromLine, m_totalDistOff / (m_stepCounter + 1));
    }
    m_stepCounter++;

    return WheelVelocities{left_target, right_target};
}


// calculates the curvature of a point within the robot frame
double PurePursuitController::calculateCurvature(Position robotFrameTargetPt) {
    double actual_dist = calcDistBetweenPoints({0, 0}, robotFrameTargetPt);
    
    double curvature = 0.0;
    if (actual_dist < 8.0) {
        actual_dist = 8.0;
    }
    curvature = (2.0 * robotFrameTargetPt.y) / (actual_dist * actual_dist); // Use lateral distance (Y) instead of forward distance (X)

    return curvature;
}


// converts a target point to a point local to the robot's frame of reference
Position PurePursuitController::convertPtToRobotFrame(Position targetPoint, const Pose& currentPose) {
    double dx = targetPoint.x - currentPose.x;
    double dy = targetPoint.y - currentPose.y;
    double robot_heading_rad = currentPose.theta;

    // localize displacements relative to orientation of robot
    double localX = dx * cos(robot_heading_rad) + dy * sin(robot_heading_rad);
    double localY = dx * (-sin(robot_heading_rad)) + dy * cos(robot_heading_rad);

    return {localX, localY};
}


// gets the dynamic lookahead distance based off of the forward velocity of the robot.
double PurePursuitController::getLookaheadDist() {
    double vel = std::abs(odom.getParallelVel());
    // pros::lcd::print(5, "Velocity: %lf in/s", vel); // Commented out to prevent LVGL crashes
    double dynamicLookahead = std::clamp(LOOKAHEAD_SECONDS * vel, MIN_LOOKAHEAD_DIST, MAX_LOOKAHEAD_DIST);
    // printf("Dynamic Lookahead: %lf\n", dynamicLookahead);
    return dynamicLookahead;
}


int PurePursuitController::getBaseVelocity(double curvature) {
    int max_base_vel_adjusted = MAX_BASE_VEL * m_speedMultiplier;
    int base_vel = max_base_vel_adjusted / (1 + (std::abs(curvature) * SPEED_ADJUSTMENT_CONST));
    int min_base_adjusted = MIN_BASE_VEL;

    // Further reduce speed if we're close to the end of the path to prevent overshooting
    if (m_distFromEnd < END_SLOWDOWN_THRESH) {
        base_vel = static_cast<int>(base_vel * (m_distFromEnd / END_SLOWDOWN_THRESH));
        min_base_adjusted = 0; // Allow full stop when within the slowdown threshold
    }

    return std::clamp(base_vel, min_base_adjusted, MAX_BASE_VEL);
}