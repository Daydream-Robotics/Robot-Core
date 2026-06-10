#include "stanley.hpp"
#include "helpers.hpp"
#include "subsystems.hpp"
#include <cmath>
#include <algorithm>

void StanleyController::setPath(ALS_Path& path) {
    m_als_path       = &path;
    m_closestIdx = 0;
}


bool StanleyController::step() {
    if (!m_als_path || !m_als_path->isValid() ||m_als_path->getSamples().empty()) {
        leftMotors.move_velocity(0);
        rightMotors.move_velocity(0);
        return true;
    }

    odom.updatePose();

    double x   = odom.getPosX();
    double y   = odom.getPosY();
    double yaw = odom.getYaw(); 

    m_closestIdx = m_als_path->findClosestSampleIndex({x, y}, m_closestIdx);
    const Sample& nearest = m_als_path->getSamples()[m_closestIdx];

    double distFromEnd = m_als_path->getTotalLength() - nearest.s;
    if (distFromEnd < STANLEY_END_TOL) {
        leftMotors.move_velocity(0);
        rightMotors.move_velocity(0);
        return true;
    }


    double pathYawRad  = nearest.heading;
    double robotYawRad = convertDegToRad(yaw);

    
    double headingError = normalizeAngle(pathYawRad - robotYawRad);

    double ex  = nearest.x - x;
    double ey  = nearest.y - y;
    double cte = -std::sin(pathYawRad) * ex + std::cos(pathYawRad) * ey;

    double speed = std::abs(odom.getParallelVel()); 


    double steer = headingError + std::atan2(STANLEY_K * cte, speed + STANLEY_K_SOFT);
    steer = std::clamp(steer, -STANLEY_MAX_STEER, STANLEY_MAX_STEER);

    int baseVel = STANLEY_MAX_VEL;
    if (distFromEnd < STANDLEY_END_SLOW) {
        double scale = distFromEnd / STANDLEY_END_SLOW;
        baseVel = static_cast<int>(baseVel * scale);
        baseVel = std::max(baseVel, STANLEY_MIN_VEL);
    }

    double leftVel  = (baseVel - steer * STANLEY_TURN_SCALE);
    double rightVel = (baseVel + steer * STANLEY_TURN_SCALE);

   
    double maxMag = std::max(std::abs(leftVel), std::abs(rightVel));
    if (maxMag > STANLEY_MAX_VEL) {
        double scale = STANLEY_MAX_VEL / maxMag;
        leftVel  *= scale;
        rightVel *= scale;
    }

    leftMotors.move_velocity(static_cast<int>(leftVel));
    rightMotors.move_velocity(static_cast<int>(rightVel));

    return false;
}