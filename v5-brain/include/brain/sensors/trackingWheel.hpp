#pragma once

#include "robotConfig.hpp"

namespace brain {

class Encoder;

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

} // namespace brain
