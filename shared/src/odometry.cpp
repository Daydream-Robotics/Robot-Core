/** @file odometry.cpp
 *  @brief Tracking-wheel integration using the integration branch's field frame.
 */

#include "odometry.hpp"

#include <cmath>
#include <numbers>

namespace shared {
namespace {
double normalizeAngle(double angle) {
    return std::atan2(std::sin(angle), std::cos(angle));
}
} // namespace

Odometry::Odometry(const RobotConfig& config) : m_config(config) {}

Pose Odometry::update(const OdometrySample& sample) {
    const double THETA = sample.headingDelta;
    const double CHORD = std::abs(THETA) > 1e-9 ? 2.0 * std::sin(THETA / 2.0) / THETA : 1.0;
    const double FORWARD = (sample.parallelDelta - THETA * m_config.parallelTrackingWheel.offset) * CHORD;
    const double LEFT = (sample.perpendicularDelta + THETA * m_config.perpendicularTrackingWheel.offset) * CHORD;
    const double MIDPOINT = std::numbers::pi / 2.0 - m_pose.heading + THETA / 2.0;
    m_pose.x += std::cos(MIDPOINT) * FORWARD - std::sin(MIDPOINT) * LEFT;
    m_pose.y += std::sin(MIDPOINT) * FORWARD + std::cos(MIDPOINT) * LEFT;
    m_pose.heading = normalizeAngle(m_pose.heading - THETA);
    return m_pose;
}

void Odometry::reset(const Pose& pose) { m_pose = pose; }
Pose Odometry::pose() const { return m_pose; }

} // namespace shared
