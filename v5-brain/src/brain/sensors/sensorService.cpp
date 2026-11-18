/**
 * @file sensorService.cpp
 * @brief Implementation of the V5 Brain physical sensor acquisition service.
 * @details Samples the IMU, PROS motor groups, and tracking wheels on each tick,
 *          computes per-cycle deltas in protocol units (centidegrees and rad/s),
 *          and populates @ref brain::SensorSnapshot.
 */

#include "brain/sensors/sensorService.hpp"

#include "brain/sensors/trackingWheel.hpp"
#include "pros/rtos.hpp"

#include <algorithm>
#include <cerrno>
#include <cmath>
#include <numbers>
#include <vector>

namespace brain {

namespace {

/// Conversion factor from revolutions per minute (RPM) to radians per second (rad/s).
constexpr double RPM_TO_RAD_PER_SEC = (2.0 * std::numbers::pi) / 60.0;

/// Conversion factor from degrees to protocol centidegrees (1 deg = 100 centidegrees).
constexpr double DEG_TO_CENTIDEGREES = 100.0;

/**
 * @brief Convert a floating-point degree value to rounded integer centidegrees.
 * @param degrees Angular value or delta in degrees.
 * @return Rounded value in centidegrees.
 */
std::int32_t toCentidegrees(double degrees) {
    return static_cast<std::int32_t>(std::llround(degrees * DEG_TO_CENTIDEGREES));
}

/**
 * @brief Inspect PROS fault flags across all motors in a motor group.
 * @param group Reference to the PROS MotorGroup to inspect.
 * @return Bitmask of @c packets::fault flags active on any motor in the group.
 * @todo Planned feature: Also map @c packets::fault::DRIVER and @c DRV_OVER_CURRENT if exposed
 *       by firmware/PROS fault bitmasks.
 */
std::uint8_t readMotorGroupFaults(pros::MotorGroup& group) {
    std::uint8_t flags = 0;

    errno = 0;
    const std::vector<std::int32_t> overTemp = group.is_over_temp_all();
    if (errno == ENODEV || overTemp.empty()) {
        return packets::fault::MISSING;
    }

    for (std::int32_t tempFlag : overTemp) {
        if (tempFlag != 0) {
            flags |= packets::fault::OVER_TEMP;
        }
    }

    const std::vector<std::int32_t> overCurrent = group.is_over_current_all();
    for (std::int32_t currentFlag : overCurrent) {
        if (currentFlag != 0) {
            flags |= packets::fault::OVER_CURRENT;
        }
    }

    return flags;
}

} // namespace

/**
 * @brief Construct a SensorService bound to existing hardware device references.
 * @param headingSensor IMU used to track heading changes.
 * @param motorGroups Non-owning array of motor-group pointers.
 * @param motorGroupCount Number of valid entries in @p motorGroups.
 * @param trackingWheels Non-owning array of tracking-wheel pointers.
 * @param trackingWheelCount Number of valid entries in @p trackingWheels.
 */
SensorService::SensorService(pros::IMU& headingSensor,
                             pros::MotorGroup* const* motorGroups,
                             std::size_t motorGroupCount,
                             TrackingWheel* const* trackingWheels,
                             std::size_t trackingWheelCount)
    : m_headingSensor(headingSensor),
      m_motorGroups(motorGroups),
      m_motorGroupCount(motorGroupCount),
      m_trackingWheels(trackingWheels),
      m_trackingWheelCount(trackingWheelCount) {}

/**
 * @brief Sample all configured sensors and return a coherent @ref SensorSnapshot.
 * @return Populated @ref SensorSnapshot for the current acquisition tick.
 * @todo Planned feature: Verify whether @c TrackingWheelTelemetry::delta should carry raw
 *       wheel rotation centidegrees directly from @c Encoder::getDelta() or linear distance
 *       scaled into centidegrees; currently converts @c TrackingWheel::deltaDistance().
 * @todo Planned feature: Populate @c TrackingWheelTelemetry::faultFlags once @c TrackingWheel
 *       exposes underlying @c Encoder::connected() status.
 */
SensorSnapshot SensorService::sample() {
    SensorSnapshot snapshot{};

    // 1. Sample IMU continuous rotation (PROS clockwise-positive -> protocol CCW-positive).
    errno = 0;
    const double currentHeadingDeg = m_headingSensor.get_rotation();
    if (errno == 0 && !m_headingSensor.is_calibrating() && std::isfinite(currentHeadingDeg)) {
        if (m_hasHeadingBaseline) {
            const double deltaHeadingDeg = -(currentHeadingDeg - m_lastHeadingDeg);
            snapshot.headingDelta = toCentidegrees(deltaHeadingDeg);
        } else {
            snapshot.headingDelta = 0;
            m_hasHeadingBaseline = true;
        }
        m_lastHeadingDeg = currentHeadingDeg;
    } else {
        snapshot.headingDelta = 0;
    }

    // 2. Sample configured motor groups up to protocol maximum.
    const std::size_t validMotorGroups =
        (m_motorGroups != nullptr)
            ? std::min<std::size_t>(m_motorGroupCount, packets::MAX_GROUPS)
            : 0U;
    snapshot.motorGroupCount = static_cast<std::uint8_t>(validMotorGroups);

    for (std::size_t i = 0; i < validMotorGroups; ++i) {
        pros::MotorGroup* group = m_motorGroups[i];
        MotorGroupTelemetry& telemetry = snapshot.motorGroups[i];

        if (group == nullptr) {
            telemetry.faultFlags = packets::fault::MISSING;
            continue;
        }

        // Ensure motor group reports positions in degrees so centidegree conversion is exact.
        group->set_encoder_units_all(pros::E_MOTOR_ENCODER_DEGREES);

        telemetry.faultFlags = readMotorGroupFaults(*group);
        if ((telemetry.faultFlags & packets::fault::MISSING) != 0) {
            telemetry.positionDelta = 0;
            telemetry.velocity = 0.0f;
            m_hasMotorBaseline[i] = false;
            continue;
        }

        // Average position across motors in the group (in degrees).
        const std::vector<double> positions = group->get_position_all();
        double avgPosDeg = 0.0;
        if (!positions.empty()) {
            for (double pos : positions) {
                avgPosDeg += pos;
            }
            avgPosDeg /= static_cast<double>(positions.size());
        }

        if (m_hasMotorBaseline[i]) {
            telemetry.positionDelta = toCentidegrees(avgPosDeg - m_lastMotorPosDeg[i]);
        } else {
            telemetry.positionDelta = 0;
            m_hasMotorBaseline[i] = true;
        }
        m_lastMotorPosDeg[i] = avgPosDeg;

        // Average output-shaft velocity across motors in the group (RPM -> rad/s).
        const std::vector<double> velocities = group->get_actual_velocity_all();
        double avgRpm = 0.0;
        if (!velocities.empty()) {
            for (double rpm : velocities) {
                avgRpm += rpm;
            }
            avgRpm /= static_cast<double>(velocities.size());
        }
        telemetry.velocity = static_cast<float>(avgRpm * RPM_TO_RAD_PER_SEC);
    }

    // 3. Sample configured tracking wheels up to protocol maximum.
    const std::size_t validTrackingWheels =
        (m_trackingWheels != nullptr)
            ? std::min<std::size_t>(m_trackingWheelCount, packets::MAX_ROTATIONS)
            : 0U;
    snapshot.trackingWheelCount = static_cast<std::uint8_t>(validTrackingWheels);

    for (std::size_t i = 0; i < validTrackingWheels; ++i) {
        TrackingWheel* wheel = m_trackingWheels[i];
        TrackingWheelTelemetry& telemetry = snapshot.trackingWheels[i];

        if (wheel == nullptr) {
            telemetry.faultFlags = packets::fault::MISSING;
            telemetry.delta = 0;
            continue;
        }

        wheel->update();
        // TODO: Confirm unit convention between TrackingWheel::deltaDistance() and protocol
        //       TrackingWheelTelemetry::delta (centidegrees of wheel rotation vs. scaled length).
        telemetry.delta = toCentidegrees(wheel->deltaDistance());
        telemetry.faultFlags = 0;
    }

    // 4. Capture microsecond timestamp after all device reads complete (per StateFixed::tUs spec).
    snapshot.sampledAt = pros::micros();

    return snapshot;
}

} // namespace brain

