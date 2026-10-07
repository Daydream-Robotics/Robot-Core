#include "daydream/motion/control/purePursuit.hpp"

#include <cmath>

#include "daydream/config/subsystems.hpp"
#include "daydream/motion/odometry.hpp"
#include "daydream/motion/pathing/arclengthSplining.hpp"
#include "daydream/utils/helpers.hpp"
#include "daydream/utils/sd_card_logging.hpp"
#include "main.h"

PurePursuitController::PurePursuitController() {}


void PurePursuitController::reset() {
    m_totalDistOff = 0;
    m_stepCounter = 0;
}


WheelVelocities PurePursuitController::compute(
    const Pose& currentPose,
    const ALS_Path& als_path,
    std::size_t& closestSampleIdx,
    PathFlag flag
) {
    Sample currentSample = als_path.getSamples()[closestSampleIdx];

    // Update dynamic lookahead.
    m_lookAheadDist = getLookaheadDist();

    Position targetPoint;

    // Create virtual pose.
    Pose virtualPose = currentPose;
    if (flag == PathFlag::REVERSE) {
        virtualPose.theta = angleDiffRad(currentPose.theta + M_PI, 0);
    }

    // If distance to end is less than lookahead, use ghost point to prevent aggressive braking.
    m_distFromEnd = als_path.getTotalLength() - currentSample.s;

    if (m_distFromEnd < m_lookAheadDist) {
        // Create ghost point.
        Sample lastPoint = als_path.getSamples().back();

        targetPoint = {
            lastPoint.x + END_GHOST_CAST * std::cos(lastPoint.heading),
            lastPoint.y + END_GHOST_CAST * std::sin(lastPoint.heading)
        };
    } else {
        // Get global target point coordinates.
        double targetS = currentSample.s + m_lookAheadDist;

        if (targetS > als_path.getTotalLength()) {
            targetS = als_path.getTotalLength();
        }

        Waypoint targetWP = als_path.getPointAtArcLength(targetS);
        targetPoint = {targetWP.x, targetWP.y};
    }

    // Translate coordinates to the robot frame.
    Position robotFrameTargetPt = convertPtToRobotFrame(targetPoint, virtualPose);

    // Get the curvature to the target point.
    double steeringCurvature = calculateCurvature(robotFrameTargetPt);

    // Look ahead along the path to find the sharpest upcoming curve.
    double pathMaxCurvature = als_path.getMaxAbsCurvatureInRange(
        currentSample.s,
        currentSample.s + m_lookAheadDist
    );

    // Throttle base velocity based on the sharpest upcoming curve.
    // Direction is selected externally for the whole path.
    // int baseVel = static_cast<int>(
    //     getBaseVelocity(pathMaxCurvature, m_speedAdjustmentConst) * velocityDirection
    // );

    int baseVel = static_cast<int>(getBaseVelocity(pathMaxCurvature));

    if (flag == PathFlag::REVERSE) {
        baseVel = -baseVel;
    }

    // !IMPORTANT! If reverse tracking steers the wrong way, flip the sign of curvature.
    double turnVel = steeringCurvature * std::abs(baseVel) * TURN_RATE;
    double leftTarget = baseVel - turnVel;
    double rightTarget = baseVel + turnVel;

    // Maintain the turn ratio if the requested velocity exceeds the motor's physical limit.
    double maxReq = std::max(std::abs(leftTarget), std::abs(rightTarget));

    if (maxReq > 600.0) {
        leftTarget = leftTarget * (600.0 / maxReq);
        rightTarget = rightTarget * (600.0 / maxReq);
    }

    if (m_stepCounter % 10 == 0) {
        double currentVel = odom.getParallelVel();

        // pros::lcd::print(5, "Velocity: %.2lf in/s", currentVel);
        // pros::lcd::print(6, "LX %.2lf, LY %.2lf", robotFrameTargetPt.x, robotFrameTargetPt.y);
        // pros::lcd::print(1, "Cur: %lf", steeringCurvature);
        // pros::lcd::print(2, "VEL: %d", baseVel);

        double distFromLine = std::hypot(
            currentSample.x - virtualPose.x,
            currentSample.y - virtualPose.y
        );

        m_totalDistOff += std::abs(distFromLine);

        printf(
            "[PP] Pos:(%.2f, %.2f) H:%.2f | Vel:%.2f | LookAhead:%.2f | "
            "Dir:%.0f | TgtGlobal:(%.2f, %.2f) TgtLocal:(%.2f, %.2f) | "
            "Curv:%.4f | Vels: B:%d L:%.1f R:%.1f | OffAtStep: %.4f | "
            "AvgDistOff: %.4f\n",
            currentPose.x,
            currentPose.y,
            convertRadToDeg(currentPose.theta),
            currentVel,
            m_lookAheadDist,
            1.0,
            targetPoint.x,
            targetPoint.y,
            robotFrameTargetPt.x,
            robotFrameTargetPt.y,
            steeringCurvature,
            baseVel,
            leftTarget,
            rightTarget,
            distFromLine,
            m_totalDistOff / (m_stepCounter + 1)
        );
    }

    m_stepCounter++;

    return WheelVelocities{leftTarget, rightTarget};
}


/**
 * @brief Calculates the curvature from the robot to a target point.
 *
 * @param robotFrameTargetPt Target point in the robot's local coordinate frame.
 * @return Curvature required to reach the target point.
 */
double PurePursuitController::calculateCurvature(Position robotFrameTargetPt) {
    double actualDist = calcDistBetweenPoints({0, 0}, robotFrameTargetPt);

    double curvature = 0.0;

    if (actualDist < 8.0) {
        actualDist = 8.0;
    }

    curvature = (2.0 * robotFrameTargetPt.y) / (actualDist * actualDist);

    return curvature;
}


/**
 * @brief Converts a target point from global coordinates to the robot's local frame.
 *
 * @param targetPoint Target point in global field coordinates.
 * @param currentPose Current global pose of the robot.
 * @return Target point represented in the robot's local coordinate frame.
 */
Position PurePursuitController::convertPtToRobotFrame(
    Position targetPoint,
    const Pose& currentPose
) {
    double dx = targetPoint.x - currentPose.x;
    double dy = targetPoint.y - currentPose.y;
    double robotHeadingRad = currentPose.theta;

    // Localize displacements relative to the orientation of the robot.
    double localX = dx * std::cos(robotHeadingRad) + dy * std::sin(robotHeadingRad);
    double localY = dx * (-std::sin(robotHeadingRad)) + dy * std::cos(robotHeadingRad);

    return {localX, localY};
}


/**
 * @brief Calculates the dynamic lookahead distance based on the robot's velocity.
 *
 * @return Dynamic lookahead distance (inches).
 */
double PurePursuitController::getLookaheadDist() {
    double vel = std::abs(odom.getParallelVel());

    // pros::lcd::print(5, "Velocity: %lf in/s", vel);
    // Commented out to prevent LVGL crashes.

    double dynamicLookahead = std::clamp(
        LOOKAHEAD_SECONDS * vel,
        MIN_LOOKAHEAD_DIST,
        MAX_LOOKAHEAD_DIST
    );

    // printf("Dynamic Lookahead: %lf\n", dynamicLookahead);

    return dynamicLookahead;
}


/**
 * @brief Calculates the base velocity based on path curvature and distance from the end.
 *
 * The velocity is reduced for sharp curves and when approaching the end of the path.
 *
 * @param curvature Curvature of the upcoming section of the path.
 * @return Base motor velocity (RPM).
 */
int PurePursuitController::getBaseVelocity(double curvature) {
    int maxBaseVelAdjusted = MAX_BASE_VEL * m_speedMultiplier;
    int baseVel = maxBaseVelAdjusted /
                  (1 + (std::abs(curvature) * SPEED_ADJUSTMENT_CONST));

    int minBaseAdjusted = MIN_BASE_VEL;

    // Further reduce speed if we're close to the end of the path to prevent overshooting.
    if (m_distFromEnd < END_SLOWDOWN_THRESH) {
        baseVel = static_cast<int>(
            baseVel * (m_distFromEnd / END_SLOWDOWN_THRESH)
        );

        minBaseAdjusted = 0;  // Allow full stop when within the slowdown threshold.
    }

    return std::clamp(baseVel, minBaseAdjusted, MAX_BASE_VEL);
}