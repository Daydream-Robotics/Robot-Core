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
    // one row per valid source
    const int n = in.parValid + in.perpValid + 2 * in.imeValid + in.imuValid;
    if (n == 0) {
        return Measurement{};
    }

    // H columns 0 to 2 and R off-diagonals stay zero, every row below fills z, H columns 3 to 5, and R's diagonal
    Measurement m;
    m.z.resize(n);
    m.H.setZero(n, 6);
    m.R.setZero(n, n);

    const double dt = in.dt;
    // IME scrub grows with turn rate in either direction
    const double absOmega = std::abs(m_s[5]);

    int row = 0;

    // writes one row (measured value, H in the vX, vY, omega columns, sigma squared into R) and moves to the next
    const auto addRow = [&](double zVal, double hVx, double hVy, double hOmega, double sigma) {
        m.z(row) = zVal;
        m.H(row, 3) = hVx;
        m.H(row, 4) = hVy;
        m.H(row, 5) = hOmega;
        m.R(row, row) = sigma * sigma;
        ++row;
    };

    // parallel wheel: dt * (vX + offset * omega), offset positive to the right
    if (in.parValid) {
        const double hOmega = dt * PARALLEL_TRACKING_WHEEL_OFFSET;
        const double sigma = ODOM_EKF_TRACKING_SIGMA_PER_IN * std::abs(in.dsPar) + ODOM_EKF_TRACKING_SIGMA_FLOOR;
        addRow(in.dsPar, dt, 0.0, hOmega, sigma);
    }

    // perpendicular wheel: dt * (vY - offset * omega), offset positive behind
    if (in.perpValid) {
        const double hOmega = -dt * PERPENDICULAR_TRACKING_WHEEL_OFFSET;
        const double sigma = ODOM_EKF_TRACKING_SIGMA_PER_IN * std::abs(in.dsPerp) + ODOM_EKF_TRACKING_SIGMA_FLOOR;
        addRow(in.dsPerp, 0.0, dt, hOmega, sigma);
    }

    // drive IMEs: dt * (vX -/+ omega * track / 2), noisier and worse in turns
    if (in.imeValid) {
        const double hOmega = dt * TRACK_WIDTH / 2.0;
        const double turnSigma = ODOM_EKF_IME_SIGMA_PER_OMEGA * absOmega + ODOM_EKF_IME_SIGMA_FLOOR;
        const double sigmaLeft = ODOM_EKF_IME_SIGMA_PER_IN * std::abs(in.dsLeft) + turnSigma;
        const double sigmaRight = ODOM_EKF_IME_SIGMA_PER_IN * std::abs(in.dsRight) + turnSigma;
        addRow(in.dsLeft, dt, 0.0, -hOmega, sigmaLeft);
        addRow(in.dsRight, dt, 0.0, hOmega, sigmaRight);
    }

    // IMU: dt * omega, gyro noise grows with sqrt(dt) and leftover bias with dt
    if (in.imuValid) {
        const double sigma = ODOM_EKF_IMU_NOISE_DENSITY * std::sqrt(dt) + ODOM_EKF_IMU_BIAS_SIGMA * dt;
        addRow(in.dTheta, 0.0, 0.0, dt, sigma);
    }

    // a source counted but not added (or the reverse) would leave bad rows, skip the update instead
    if (row != n) {
        return Measurement{};
    }
    return m;
}

void OdomEkf::update(const Measurement& m) {
    // TODO: y = z - H * s, S = H * P * H^T + R, K = P * H^T * S^-1
    // TODO: s += K * y, wrap theta
    // TODO: P = (I - K * H) * P * (I - K * H)^T + K * R * K^T (Joseph form)
}
