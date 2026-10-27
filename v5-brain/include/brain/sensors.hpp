#pragma once

#include "brain/types.hpp"
#include "robotConfig.hpp"

#include "pros/imu.hpp"
#include "pros/misc.hpp"
#include "pros/motor_group.hpp"
#include "pros/rotation.hpp"

#include <cstddef>
#include <cstdint>

namespace brain {

class Encoder {
public:
    explicit Encoder(std::int8_t smartPort, bool reversed = false);

    void update();
    double position() const;
    double delta() const;
    bool connected() const;
    void tare();

private:
    pros::Rotation device_;
};

class TrackingWheel {
public:
    TrackingWheel(Encoder& encoder, shared::TrackingWheelGeometry geometry);

    void update();
    double distance() const;
    double deltaDistance() const;

private:
    Encoder& encoder_;
    shared::TrackingWheelGeometry geometry_;
};

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

class DriverInputReader {
public:
    explicit DriverInputReader(pros::Controller& controller);

    DriverInput read() const;

private:
    pros::Controller& controller_;
};

} // namespace brain
