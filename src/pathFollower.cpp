#include "pathFollower.hpp"
#include "subsystems.hpp"
#include "constants.h"

PathFollower::PathFollower(MotionController& controller)
     : m_controller(controller) {}

void PathFollower::setPath(ALS_Path& path) {
    m_path = &path;
    m_currentSampleIdx = 0;
    m_isFinished = false;
    if (m_path->isValid()) {
        m_distanceFromEnd = m_path->getTotalLength();
    }
}

bool PathFollower::step() {
    if (m_isFinished || !m_path || !m_path->isValid() || m_path->getSamples().empty()) {
        leftMotors.move_velocity(0);
        rightMotors.move_velocity(0);
        return true;
    }

    odom.updatePose();
    Pose currentPose = odom.getPose();

    m_currentSampleIdx = m_path->findClosestSampleIndex({currentPose.x, currentPose.y}, m_currentSampleIdx);

    if (m_currentSampleIdx >= m_path->getSamples().size()) {
        m_isFinished = true;
        return true;
    }

    Sample targetSample = m_path->getSamples()[m_currentSampleIdx];
    m_distanceFromEnd = m_path->getTotalLength() - targetSample.s;

    if (m_distanceFromEnd < END_TOLERANCE) {
        leftMotors.move_velocity(0);
        rightMotors.move_velocity(0);
        m_isFinished = true;
        return true;
    }

    WheelVelocities wheelVelocities = m_controller.compute(currentPose, targetSample);
    
    leftMotors.move_velocity(wheelVelocities.left);
    rightMotors.move_velocity(wheelVelocities.right);

    return false;
}