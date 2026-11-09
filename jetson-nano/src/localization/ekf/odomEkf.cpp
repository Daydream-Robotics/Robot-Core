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
    // skip a duplicate, out-of-order, or NaN dt
    if (in.dt <= 0.0 || std::isnan(in.dt)) {
        return;
    }

    // move the state forward and grow P
    predict(in.dt);

    Measurement measurement = buildMeasurement(in);
    // no valid source this cycle: coast on the prediction
    if (measurement.z.size() == 0) {
        return;
    }

    update(measurement);
}

const OdomEkf::State& OdomEkf::state() const {
    return m_s;
}

const OdomEkf::Cov& OdomEkf::covariance() const {
    return m_P;
}

void OdomEkf::reset(const Pose& pose) {
    // zero everything, so v_x, v_y, omega start at rest
    m_s.setZero();

    // new pose in the odometry frame, heading wrapped to (-pi, pi]
    m_s[0] = pose.x;
    m_s[1] = pose.y;
    m_s[2] = utils::wrapAngle(pose.theta);

    // drop the old covariance and correlations, start from the initial P
    m_P = initialP;
}

void OdomEkf::predict(double dt) {
    // old state
    const double theta = m_s[2];
    const double vX = m_s[3];
    const double vY = m_s[4];
    const double omega = m_s[5];

    // heading halfway through the cycle, makes the straight step match the arc
    const double thetaMid = theta + omega * dt / 2.0;
    const double cosThetaMid = std::cos(thetaMid);
    const double sinThetaMid = std::sin(thetaMid);

    // robot-frame velocity rotated into the odometry frame, times dt
    const double dx = dt * (vX * cosThetaMid - vY * sinThetaMid);
    const double dy = dt * (vX * sinThetaMid + vY * cosThetaMid);

    // F = d(new state) / d(old state), identity except where the pose depends on theta or the velocities
    Cov F = Cov::Identity();
    F(0, 2) = -dy; // dx/dtheta, a heading error swings the step sideways
    F(1, 2) = dx; // dy/dtheta
    F(0, 3) = dt * cosThetaMid; // dx/dvX, forward speed moves along the heading
    F(1, 3) = dt * sinThetaMid; // dy/dvX
    F(0, 4) = -dt * sinThetaMid; // dx/dvY, sideways speed moves perpendicular to the heading
    F(1, 4) = dt * cosThetaMid; // dy/dvY
    F(0, 5) = -dy * dt / 2.0; // dx/domega
    F(1, 5) = dx * dt / 2.0; // dy/domega
    F(2, 5) = dt; // dtheta/domega

    // process noise on the velocities only, the next predict carries it into the pose through F
    const double accelStep = ODOM_EKF_ACCEL_SIGMA * dt; // in/s the speed may change this cycle
    const double angAccelStep = ODOM_EKF_ANG_ACCEL_SIGMA * dt; // rad/s the turn rate may change this cycle

    Cov Q = Cov::Zero();
    Q(3, 3) = accelStep * accelStep;
    Q(4, 4) = accelStep * accelStep;
    Q(5, 5) = angAccelStep * angAccelStep;

    // new state, velocities stay constant
    m_s[0] += dx;
    m_s[1] += dy;
    m_s[2] = utils::wrapAngle(theta + omega * dt);

    // propagate the uncertainty and add this cycle's process noise
    m_P = F * m_P * F.transpose() + Q;

    // remove rounding asymmetry
    m_P = ((m_P + m_P.transpose()) / 2.0).eval();
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
