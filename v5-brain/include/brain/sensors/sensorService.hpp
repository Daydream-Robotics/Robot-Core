#pragma once

#include "brain/types.hpp"

#include "pros/imu.hpp"
#include "pros/motor_group.hpp"

#include <cstddef>

namespace brain {

class TrackingWheel;

class SensorService {
public:
    SensorService(pros::IMU& headingSensor,
                  pros::MotorGroup* const* motorGroups,
                  std::size_t motorGroupCount,
                  TrackingWheel* const* trackingWheels,
                  std::size_t trackingWheelCount);

    BrainState sample(const DriverInput& driverInput);

private:
    pros::IMU& headingSensor_;
    pros::MotorGroup* const* motorGroups_;
    std::size_t motorGroupCount_;
    TrackingWheel* const* trackingWheels_;
    std::size_t trackingWheelCount_;
};

} // namespace brain
