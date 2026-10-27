#pragma once

namespace shared {

struct TrackingWheelGeometry {
    double diameter;
    double offset;
};

// Robot measurements used by both the V5 Brain and Jetson Nano.
struct RobotConfig {
    double driveWheelDiameter;
    double driveGearRatio;
    double trackWidth;
    TrackingWheelGeometry parallelTrackingWheel;
    TrackingWheelGeometry perpendicularTrackingWheel;
};

} // namespace shared
