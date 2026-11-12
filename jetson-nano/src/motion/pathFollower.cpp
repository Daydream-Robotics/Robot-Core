#include "motion/pathFollower.hpp"
#include "config/constants.h"
#include "config/robotDrive.hpp"
#include <algorithm>
#include <chrono>
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
    for (std::size_t j = 0; j < packets::DRIVE_GROUPS; j++) {
        m_link.setGroup(packets::driveSlot(j), packets::GroupMode::STOP, 0);
    }
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
    if (!sample || !m_link.linkUp() || sample->omegaRadS.size() < packets::DRIVE_GROUPS) {
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
    // omegaRadS is at the motor output shaft, controllers work at the wheel, each group with its own gearing
    const int64_t nowUs = std::chrono::duration_cast<std::chrono::microseconds>(
        std::chrono::steady_clock::now().time_since_epoch()).count();
    DriveState drive;
    for (std::size_t j = 0; j < packets::DRIVE_GROUPS; j++) {
        drive.omega[j] = sample->omegaRadS[packets::driveSlot(j)] * driveGearRatio(j);
    }
    drive.batteryVolts = sample->batteryMv / 1000.0;
    drive.age = (nowUs - sample->tNanoUs) / 1e6;
    // the wheel speeds are from the brain's last state and the command applies a little after this step,
    // so the controller works from where the robot will be then
    Pose currentPose = *pose;
    m_controller.predict(currentPose, drive);

    // only search a short way ahead
    const std::vector<Sample>& samples = m_path->getSamples();
    const double searchLimit = samples[std::min(m_currentSampleIdx, samples.size() - 1)].s + PATH_SEARCH_AHEAD;
    std::size_t searchEnd = m_currentSampleIdx;
    while (searchEnd < samples.size() && samples[searchEnd].s <= searchLimit) {
        searchEnd++;
    }
    m_currentSampleIdx = m_path->findClosestSampleIndex({currentPose.x, currentPose.y}, m_currentSampleIdx, searchEnd);

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

    const DriveCommand command = m_controller.compute(currentPose, drive, *m_path, m_currentSampleIdx, flag);

    const packets::GroupMode mode = command.input == ControlMode::INPUT_VOLTAGE ? packets::GroupMode::VOLTAGE : packets::GroupMode::VELOCITY;
    for (std::size_t j = 0; j < packets::DRIVE_GROUPS; j++) {
        m_link.setGroup(packets::driveSlot(j), mode, static_cast<float>(command.value[j]));
    }

    return false;
}
