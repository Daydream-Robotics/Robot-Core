#pragma once

#include "robotConfig.hpp"

namespace shared {

struct Pose {
    double x;
    double y;
    double heading;
};

// Platform-neutral sensor input for the shared odometry calculation.
struct OdometrySample {
    double parallelDelta;
    double perpendicularDelta;
    double headingDelta;
};

class Odometry {
public:
    explicit Odometry(const RobotConfig& config);

    Pose update(const OdometrySample& sample);
    void reset(const Pose& pose);
    Pose pose() const;

private:
    RobotConfig config_;
    Pose pose_{};
};

} // namespace shared
