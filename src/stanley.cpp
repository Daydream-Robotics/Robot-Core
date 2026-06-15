#include "stanley.hpp"
#include "helpers.hpp"
#include "subsystems.hpp"
#include <cmath>
#include <algorithm>

void StanleyController::setPath(ALS_Path& path) {
    m_als_path       = &path;
    m_closestIdx = 0;
}

/*stanley controllr step function
    - direction should be set to -1 for backwards travel
    - direction is preset to 1 for fowards travel*/
bool StanleyController::step(int direction) {

    //checking if there is a vaild path to follow
    if (!m_als_path || !m_als_path->isValid() ||m_als_path->getSamples().empty()) {
        leftMotors.move_velocity(0);
        rightMotors.move_velocity(0);
        return true;
    }

    odom.updatePose();


    //sets the pose of the robot to a variable

    double x   = odom.getPosX();
    double y   = odom.getPosY();
    double yaw = odom.getYaw(); 
    printf("X: %.3f   Y: %.3f   Yaw: %.3f\n", x, y, yaw);

    //calls upon the closest point helper to find the closest point relivle to the robot's last upodated pose
    m_closestIdx = m_als_path->findClosestSampleIndex({x, y}, m_closestIdx);
    const Sample& nearest = m_als_path->getSamples()[m_closestIdx];

    double distFromEnd = m_als_path->getTotalLength() - nearest.s;
    printf("End Distance%.3f\n", distFromEnd);

    if (distFromEnd < STANLEY_END_TOL) {
        leftMotors.move_velocity(0);
        rightMotors.move_velocity(0);
        printf("REACHED END with a %.3f error\n", distFromEnd);
        return true;
    }
    


    double pathYawRad  = nearest.heading;
    double robotYawRad = convertDegToRad(yaw);

    double effectivePathYaw = (direction == 1) ? pathYawRad : normalizeAngle(pathYawRad + M_PI);
    double headingError = normalizeAngle(effectivePathYaw - robotYawRad);

    // CTE sign also flips when reversing
    double ex  = nearest.x - x;
    double ey  = nearest.y - y;
    double cte = direction * (-std::sin(effectivePathYaw) * ex + std::cos(effectivePathYaw) * ey);

    double speed = std::abs(odom.getParallelVel()); 
    printf("Speed: %.3f\n", speed);

    double steer = headingError + std::atan2(STANLEY_K * cte, speed + STANLEY_K_SOFT);
    printf("Steer Before Clamp: %.3f\n", steer);

    steer = std::clamp(steer, -STANLEY_MAX_STEER, STANLEY_MAX_STEER);
    printf("Steer After Clamp: %.3f\n", steer);

    int baseVel = STANLEY_MAX_VEL;
    if (distFromEnd < STANLEY_END_SLOW) {
        double scale = distFromEnd / STANLEY_END_SLOW;
        baseVel = static_cast<int>(baseVel * scale);
        baseVel = std::max(baseVel, STANLEY_MIN_VEL);
    }

    printf("Base Velocity: %.3f\n", baseVel);

    double leftVel  = direction * baseVel - steer * STANLEY_TURN_SCALE;
    double rightVel = direction * baseVel + steer * STANLEY_TURN_SCALE;
    
    printf("leftMotors Unscaled Velocity: %.3f\nrightMotors Unscaled Velocity: %.3f\n", leftVel, rightVel);
    
   
    double maxMag = std::max(std::abs(leftVel), std::abs(rightVel));
    if (maxMag > STANLEY_MAX_VEL) {
        double scale = STANLEY_MAX_VEL / maxMag;
        leftVel  *= scale;
        rightVel *= scale;
        printf("Scale: %.3f\nleftMotors Scaled Velocity: %.3f\nrightMotors Scaled Velocity: %.3f\n", scale, leftVel, rightVel);
    }

    else printf("No Velocity Scale\n");

    leftMotors.move_velocity(static_cast<int>(leftVel));
    rightMotors.move_velocity(static_cast<int>(rightVel));

    
    
    return false;
}