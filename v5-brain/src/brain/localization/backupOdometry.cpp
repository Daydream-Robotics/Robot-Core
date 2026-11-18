/** @file backupOdometry.cpp
 *  @brief Adapt SensorService tracking-wheel and IMU deltas to shared odometry.
 */

#include "brain/localization/backupOdometry.hpp"

#include <numbers>

namespace brain {

BackupOdometry::BackupOdometry(shared::Odometry& odometry) : m_odometry(odometry) {}

void BackupOdometry::setEnabled(bool enabled) {
    if (m_enabled && !enabled && m_hasReadySample) m_valid = false;
    m_enabled = enabled;
}

bool BackupOdometry::enabled() const { return m_enabled; }
bool BackupOdometry::valid() const { return m_valid && m_hasReadySample; }

void BackupOdometry::update(const SensorSnapshot& sensors) {
    if (!m_enabled) return;
    if (!sensors.headingDeltaValid || sensors.trackingWheelCount < 2 ||
        !sensors.trackingWheels[0].deltaValid || !sensors.trackingWheels[1].deltaValid ||
        (sensors.trackingWheels[0].faultFlags & packets::fault::MISSING) ||
        (sensors.trackingWheels[1].faultFlags & packets::fault::MISSING)) {
        if (m_hasReadySample) m_valid = false;
        return;
    }

    constexpr double CENTIDEGREES_TO_RADIANS = std::numbers::pi / 18000.0;
    m_hasReadySample = true;
    m_odometry.update({sensors.trackingWheels[0].distanceDelta,
                       sensors.trackingWheels[1].distanceDelta,
                       sensors.headingDelta * CENTIDEGREES_TO_RADIANS});
}

shared::Pose BackupOdometry::pose() const { return m_odometry.pose(); }

void BackupOdometry::reset(const shared::Pose& pose) {
    m_odometry.reset(pose);
    m_valid = true;
    m_hasReadySample = false;
}

} // namespace brain
