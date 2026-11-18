#include "localization/ekf/odomInputs.hpp"

#include "config/constants.h"

std::optional<OdomInputs> OdomInputAdapter::convert(const SerialLink::Sample& sample, const BNO085& imu) {
    // TODO: first sample, store tNanoUs and return nothing

    // TODO: dt = (sample.tNanoUs - *m_lastTUs) * 1e-6, real dt even if stateSeq skipped
    // TODO: dt <= 0 -> skip, dt > ~0.2 s -> link stalled, reset() and return nothing

    // TODO: tracking wheels, ds = wheelStepRad * (diameter / 2)
    //     PARALLEL_TRACKING_WHEEL_DIAMETER, PERPENDICULAR_TRACKING_WHEEL_DIAMETER
    // TODO: IMEs, ds = imeStepRad * DRIVE_GEAR_RATIO * (DRIVE_WHEEL_DIAMETER_INCHES / 2), one value per group
    //     confirm which group index is left and which is right
    // TODO: IMU, dTheta = imu.headingIncrement(previous tNanoUs, this tNanoUs), invalid if it returns nothing
    // TODO: don't output an IME heading, (dsRight - dsLeft) / TRACK_WIDTH is already in dsLeft and dsRight

    // TODO: validity per source: packets::fault::MISSING in wheelFlags / groupFlags, NaN,
    //     or more than ~2 in in 10 ms

    // TODO: store this tNanoUs as the previous one
    (void)sample;
    (void)imu;
    return std::nullopt;
}

void OdomInputAdapter::reset() {
    // TODO: forget the previous sample
}
