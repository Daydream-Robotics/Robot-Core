/** @file sensorService.hpp
 *  @brief Interface for sampling the Brain's configured physical sensors.
 */

#pragma once

#include "brain/types.hpp"

#include "pros/imu.hpp"
#include "pros/motor_group.hpp"

#include <cstddef>

namespace brain {

/// Forward declaration for tracking-wheel references stored by SensorService.
class TrackingWheel;

/** @brief Collects a coherent sensor snapshot from the Brain's devices.
 *
 * Controller input is acquired separately by DriverInputReader and is not part
 * of this service's responsibility.
 */
class SensorService {
public:
    /** @brief Create a sensor sampler over the configured PROS devices.
     *  @param headingSensor IMU used to measure heading changes.
     *  @param motorGroups Non-owning array of configured motor-group pointers.
     *  @param motorGroupCount Number of entries in @p motorGroups.
     *  @param trackingWheels Non-owning array of tracking-wheel pointers.
     *  @param trackingWheelCount Number of entries in @p trackingWheels.
     *  @note The referenced devices and pointer arrays must outlive this service.
     */
    SensorService(pros::IMU& headingSensor,
                  pros::MotorGroup* const* motorGroups,
                  std::size_t motorGroupCount,
                  TrackingWheel* const* trackingWheels,
                  std::size_t trackingWheelCount);

    /** @brief Read the configured sensors and return their current snapshot.
     *  @return Sensor data for one acquisition cycle; no controller data is read.
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
};

} // namespace brain
