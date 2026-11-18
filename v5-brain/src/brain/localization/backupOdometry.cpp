/** @file backupOdometry.cpp
 *  @brief Adapt SensorService tracking-wheel and IMU deltas to shared odometry.
 */

#include "brain/localization/backupOdometry.hpp"

#include <numbers>

namespace brain {

BackupOdometry::BackupOdometry(shared::Odometry& odometry) : m_odometry(odometry) {}

void BackupOdometry::setEnabled(bool enabled) { m_enabled = enabled; }

bool BackupOdometry::enabled() const { return m_enabled; }

void BackupOdometry::update(const SensorSnapshot& sensors) {
    // SensorService stores wheel travel in hundredths of a length unit. The
    // wheels must be supplied in parallel, perpendicular order.
    if (!m_enabled || sensors.trackingWheelCount < 2 ||
        (sensors.trackingWheels[0].faultFlags & packets::fault::MISSING) ||
        (sensors.trackingWheels[1].faultFlags & packets::fault::MISSING)) {
        return;
    }

    constexpr double CENTIDEGREES_TO_RADIANS = std::numbers::pi / 18000.0;
    m_odometry.update({sensors.trackingWheels[0].delta / 100.0,
                       sensors.trackingWheels[1].delta / 100.0,
                       sensors.headingDelta * CENTIDEGREES_TO_RADIANS});
}

shared::Pose BackupOdometry::pose() const { return m_odometry.pose(); }

void BackupOdometry::reset(const shared::Pose& pose) {
    m_odometry.reset(pose);
}

} // namespace brain
