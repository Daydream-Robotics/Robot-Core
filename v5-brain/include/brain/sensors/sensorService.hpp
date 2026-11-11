/**
 * @file sensorService.hpp
 * @brief Interface for sampling the Brain's configured physical sensors.
 * @details Aggregates readings from the inertial sensor (IMU), drivetrain/mechanism
 *          motor groups, and odometry tracking wheels into a single coherent
 *          @ref brain::SensorSnapshot per acquisition cycle.
 */

#pragma once

#include "brain/types.hpp"

#include "pros/imu.hpp"
#include "pros/motor_group.hpp"

#include <array>
#include <cstddef>
#include <cstdint>

namespace brain {

/// Forward declaration for tracking-wheel references stored by SensorService.
class TrackingWheel;

/**
 * @class SensorService
 * @brief Collects a coherent sensor snapshot from the Brain's devices.
 * @details Controller input is acquired separately by DriverInputReader and is not part
 *          of this service's responsibility.
 * @todo Planned feature: Add a @c reset() / @c tare() method to zero cached delta baselines
 *       when a new configuration is applied or autonomous starts.
 * @todo Planned feature: Expose per-device health/calibration status queries for diagnostics.
 */
class SensorService {
public:
    /**
     * @brief Create a sensor sampler over the configured PROS devices.
     * @param headingSensor IMU used to measure heading changes.
     * @param motorGroups Non-owning array of configured motor-group pointers.
     * @param motorGroupCount Number of entries in @p motorGroups.
     * @param trackingWheels Non-owning array of tracking-wheel pointers.
     * @param trackingWheelCount Number of entries in @p trackingWheels.
     * @note The referenced devices and pointer arrays must outlive this service.
     */
    SensorService(pros::IMU& headingSensor,
                  pros::MotorGroup* const* motorGroups,
                  std::size_t motorGroupCount,
                  TrackingWheel* const* trackingWheels,
                  std::size_t trackingWheelCount);

    /**
     * @brief Destroy the SensorService instance.
     * @note Does not destroy or close the underlying non-owning hardware devices.
     */
    ~SensorService() = default;

    // Prevent copying a service that tracks per-tick stateful delta baselines.
    SensorService(const SensorService&) = delete;
    SensorService& operator=(const SensorService&) = delete;

    /**
     * @brief Read the configured sensors and return their current snapshot.
     * @return Sensor data for one acquisition cycle; no controller data is read.
     */
    SensorSnapshot sample();

private:
    /// IMU used to acquire heading information; owned by the robot configuration.
    pros::IMU& headingSensor_;

    /// Non-owning array of motor groups whose telemetry is sampled.
    pros::MotorGroup* const* motorGroups_;

    /// Number of motor groups available through @ref motorGroups_.
    std::size_t motorGroupCount_;

    /// Non-owning array of tracking wheels whose telemetry is sampled.
    TrackingWheel* const* trackingWheels_;

    /// Number of tracking wheels available through @ref trackingWheels_.
    std::size_t trackingWheelCount_;

    /// Whether @ref lastHeadingDeg_ has been initialized with a valid IMU sample.
    bool hasHeadingBaseline_ = false;

    /// Previous IMU continuous rotation reading (in degrees) for delta calculation.
    double lastHeadingDeg_ = 0.0;

    /// Tracks whether each motor-group slot has a valid baseline position for deltas.
    std::array<bool, packets::MAX_GROUPS> hasMotorBaseline_{};

    /// Previous average encoder position (in degrees) per configured motor-group slot.
    std::array<double, packets::MAX_GROUPS> lastMotorPosDeg_{};
};

} // namespace brain

