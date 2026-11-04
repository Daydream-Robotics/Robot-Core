#include "motion/pathFollower.hpp"
#include "config/constants.h"
#include <cstdio>
#include <utility>

PathFollower::PathFollower(MotionController& controller, SerialLink& link, PoseSource poseSource)
     : m_controller(controller), m_link(link), m_poseSource(std::move(poseSource)) {}

void PathFollower::setPath(const ALS_Path& path, PathFlag flag) {
    this->flag = flag;
    m_path = &path;
    m_currentSampleIdx = 0;
    m_isFinished = false;
    if (m_path->isValid()) {
        m_distanceFromEnd = m_path->getTotalLength();
    }
    m_controller.reset();
}

void PathFollower::stop() {
    m_link.setGroup(packets::LEFT_DRIVE_SLOT, packets::GroupMode::STOP, 0);
    m_link.setGroup(packets::RIGHT_DRIVE_SLOT, packets::GroupMode::STOP, 0);
}

void PathFollower::stopAndReset() {
    stop();
    m_controller.reset();
}

bool PathFollower::step() {
    if (m_isFinished || !m_path || !m_path->isValid() || m_path->getSamples().empty()) {
        stop();
        return true;
    }

    // measurements from the brain. without a live link there's nothing to control with
    const std::optional<SerialLink::Sample> sample = m_link.latestSample();
    if (!sample || !m_link.linkUp() || sample->omegaRadS.size() <= packets::RIGHT_DRIVE_SLOT) {
        stopAndReset();
        return false;
    }
    // the brain stopped the drive on its own (watchdog or restart), so the controller's last output wasn't applied
    if (sample->status & (packets::status::WATCHDOG | packets::status::BOOTED)) {
        m_controller.reset();
    }

    const std::optional<Pose> pose = m_poseSource();
    if (!pose) {
        stopAndReset();
        return false;
    }
    const Pose currentPose = *pose;

    m_currentSampleIdx = m_path->findClosestSampleIndex({currentPose.x, currentPose.y}, m_currentSampleIdx);

    if (m_currentSampleIdx >= m_path->getSamples().size()) {
        std::fprintf(stderr, "[PF-ERROR] m_currentSampleIdx (%zu) is out of bounds (size %zu)!\n", m_currentSampleIdx, m_path->getSamples().size());
        stop();
        m_isFinished = true;
        return true;
    }

    Sample targetSample = m_path->getSamples()[m_currentSampleIdx];
    m_distanceFromEnd = m_path->getTotalLength() - targetSample.s;

    if (m_distanceFromEnd < END_TOLERANCE) {
        stop();
        m_isFinished = true;
        return true;
    }

    // omegaRadS is at the motor output shaft, controllers work at the wheel
    DriveState drive{
        sample->omegaRadS[packets::LEFT_DRIVE_SLOT] * DRIVE_GEAR_RATIO,
        sample->omegaRadS[packets::RIGHT_DRIVE_SLOT] * DRIVE_GEAR_RATIO,
        sample->batteryMv / 1000.0
    };

    WheelVelocities wheelVelocities = m_controller.compute(currentPose, drive, *m_path, m_currentSampleIdx, flag);

    switch(wheelVelocities.input){
        case ControlMode::INPUT_VELOCITY:
            m_link.setGroup(packets::LEFT_DRIVE_SLOT, packets::GroupMode::VELOCITY, static_cast<float>(wheelVelocities.left));
            m_link.setGroup(packets::RIGHT_DRIVE_SLOT, packets::GroupMode::VELOCITY, static_cast<float>(wheelVelocities.right));
            break;
        case ControlMode::INPUT_VOLTAGE:
            m_link.setGroup(packets::LEFT_DRIVE_SLOT, packets::GroupMode::VOLTAGE, static_cast<float>(wheelVelocities.left));
            m_link.setGroup(packets::RIGHT_DRIVE_SLOT, packets::GroupMode::VOLTAGE, static_cast<float>(wheelVelocities.right));
            break;
        default:
            break;
    }

    return false;
}
