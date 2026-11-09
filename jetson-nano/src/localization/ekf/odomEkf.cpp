#include "localization/ekf/odomEkf.hpp"

#include <cmath>

#include "config/constants.h"
// TODO (#178): uncomment once odomInputs.hpp is merged in
// #include "localization/ekf/odomInputs.hpp"
#include "utils/mathHelpers.hpp"

// TODO (#178): delete this temporary copy of OdomInputs once odomInputs.hpp is merged in
struct OdomInputs {
    int64_t tUs = 0; // end of this cycle, Nano steady_clock (us)
    double dt = 0.0; // seconds since the previous OdomInputs

    double dsPar = 0.0; // parallel tracking wheel travel (in)
    double dsPerp = 0.0; // perpendicular tracking wheel travel (in)
    double dsLeft = 0.0; // left drive travel from IMEs (in)
    double dsRight = 0.0; // right drive travel from IMEs (in)
    double dTheta = 0.0; // IMU heading change, CCW+ (rad)

    bool parValid = false;
    bool perpValid = false;
    bool imeValid = false;
    bool imuValid = false;
};

void OdomEkf::step(const OdomInputs& in) {
    // TODO: skip the cycle if in.dt <= 0
    // TODO: predict(in.dt), then update(buildMeasurement(in)) if any row is valid
}

const OdomEkf::State& OdomEkf::state() const {
    return m_s;
}

const OdomEkf::Cov& OdomEkf::covariance() const {
    return m_P;
}

void OdomEkf::reset(const Pose& pose) {
    // TODO: set x, y, theta from pose, zero the velocities, reset P
}

void OdomEkf::predict(double dt) {
    // TODO: theta_mid = theta + omega * dt / 2
    // TODO: x += dt * (v_x * cos(theta_mid) - v_y * sin(theta_mid)), y += dt * (v_x * sin(theta_mid) + v_y * cos(theta_mid))
    // TODO: theta += omega * dt, wrapped with utils::wrapAngle, velocities unchanged
    // TODO: F = Jacobian of the lines above, P = F * P * F^T + Q(dt)
}

OdomEkf::Measurement OdomEkf::buildMeasurement(const OdomInputs& in) const {
    // TODO: one row per valid source, in the velocity columns (3, 4, 5):
    //   parallel wheel: dsPar = dt * (v_x + PARALLEL_TRACKING_WHEEL_OFFSET * omega)
    //   perpendicular wheel: dsPerp = dt * (v_y - PERPENDICULAR_TRACKING_WHEEL_OFFSET * omega)
    //   left IME: dsLeft = dt * (v_x - omega * TRACK_WIDTH / 2)
    //   right IME: dsRight = dt * (v_x + omega * TRACK_WIDTH / 2)
    //   IMU: dTheta = dt * omega
    // TODO: R, one variance per row
    return Measurement{};
}

void OdomEkf::update(const Measurement& m) {
    // TODO: y = z - H * s, S = H * P * H^T + R, K = P * H^T * S^-1
    // TODO: s += K * y, wrap theta
    // TODO: P = (I - K * H) * P * (I - K * H)^T + K * R * K^T (Joseph form)
}
