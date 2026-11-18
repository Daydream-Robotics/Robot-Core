#include "localization/ekf/odomInputs.hpp"

#include <cmath>

#include "config/constants.h"
#include "protocol/packets.hpp"

std::optional<OdomInputs> OdomInputAdapter::convert(const SerialLink::Sample& sample, const BNO085& imu) {
    // first sample, nothing to difference against yet
    if (!m_lastTUs) {
        m_lastTUs = sample.tNanoUs;
        return std::nullopt;
    }

    // start and end of this cycle, Nano steady_clock (us)
    const int64_t t0Us = *m_lastTUs;
    const int64_t t1Us = sample.tNanoUs;

    // real dt, a skipped stateSeq makes this a multiple of 10 ms
    const double dt = static_cast<double>(t1Us - t0Us) * 1e-6;

    // duplicate or out of order, keep the previous time
    if (dt <= 0.0) {
        return std::nullopt;
    }

    // link stalled, start over from this sample
    if (dt > ODOM_INPUT_STALL_DT) {
        m_lastTUs = t1Us;
        return std::nullopt;
    }

    // every source starts invalid at zero
    OdomInputs in;
    in.tUs = t1Us;
    in.dt = dt;

    // most travel any source could see this cycle
    const double maxDs = ODOM_INPUT_MAX_SPEED * dt;

    // one tracking wheel: rad -> in, valid if present, not missing, finite, and possible
    const auto wheel = [&](int slot, double diameter, double& ds, bool& valid) {
        // no reading for this slot yet
        if (slot >= static_cast<int>(sample.wheelStepRad.size()) ||
            slot >= static_cast<int>(sample.wheelFlags.size())) {
            return;
        }
        // arc length, angle times radius
        ds = sample.wheelStepRad[slot] * (diameter / 2.0);
        valid = !(sample.wheelFlags[slot] & packets::fault::MISSING) && std::isfinite(ds) && std::abs(ds) <= maxDs;
    };
    wheel(ODOM_PAR_WHEEL, PARALLEL_TRACKING_WHEEL_DIAMETER, in.dsPar, in.parValid);
    wheel(ODOM_PERP_WHEEL, PERPENDICULAR_TRACKING_WHEEL_DIAMETER, in.dsPerp, in.perpValid);

    // drive IMEs: output shaft rad -> wheel in, one value per group
    if (ODOM_LEFT_GROUP < static_cast<int>(sample.imeStepRad.size()) &&
        ODOM_RIGHT_GROUP < static_cast<int>(sample.imeStepRad.size()) &&
        ODOM_LEFT_GROUP < static_cast<int>(sample.groupFlags.size()) &&
        ODOM_RIGHT_GROUP < static_cast<int>(sample.groupFlags.size())) {
        // inches of wheel travel per radian of output shaft
        const double inPerRad = DRIVE_GEAR_RATIO * (DRIVE_WHEEL_DIAMETER_INCHES / 2.0);
        in.dsLeft = sample.imeStepRad[ODOM_LEFT_GROUP] * inPerRad;
        in.dsRight = sample.imeStepRad[ODOM_RIGHT_GROUP] * inPerRad;
        // each side present, finite, and possible
        const bool leftOk = !(sample.groupFlags[ODOM_LEFT_GROUP] & packets::fault::MISSING) &&
                            std::isfinite(in.dsLeft) && std::abs(in.dsLeft) <= maxDs;
        const bool rightOk = !(sample.groupFlags[ODOM_RIGHT_GROUP] & packets::fault::MISSING) &&
                             std::isfinite(in.dsRight) && std::abs(in.dsRight) <= maxDs;
        // the EKF uses both sides together
        in.imeValid = leftOk && rightOk;
    }

    // IMU heading change over exactly the same interval as the wheels
    const std::optional<double> dTheta = imu.headingIncrement(t0Us, t1Us);
    if (dTheta && std::isfinite(*dTheta) && std::abs(*dTheta) <= ODOM_INPUT_MAX_OMEGA * dt) {
        in.dTheta = *dTheta;
        in.imuValid = true;
    }

    // this cycle's end is the next cycle's start
    m_lastTUs = t1Us;
    return in;
}

void OdomInputAdapter::reset() {
    // the next sample is treated as the first
    m_lastTUs.reset();
}
