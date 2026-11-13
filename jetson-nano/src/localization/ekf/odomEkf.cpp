#include "localization/ekf/odomEkf.hpp"

#include <array>
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
    if (in.dt <= 0.0 || !std::isfinite(in.dt)) {
        return;
    }

    // state at the start of this cycle, the increment is measured from here
    const State before = m_s;

    // start of this cycle, Nano steady_clock (us)
    const int64_t t0Us = in.tUs - std::llround(in.dt * 1e6);

    if (in.dt > ODOM_EKF_MAX_DT) {
        m_P.bottomRightCorner<3, 3>() = initialP.bottomRightCorner<3, 3>();
        m_P.topRightCorner<3, 3>().setZero();
        m_P.bottomLeftCorner<3, 3>().setZero();
    }

    // this cycle's covariance starts with the pose known exactly, only the velocity uncertainty carries in
    m_Pstep = m_P;
    m_Pstep.topRows<3>().setZero();
    m_Pstep.leftCols<3>().setZero();


    // move the state forward and grow P
    predict(in.dt);

    Measurement measurement = buildMeasurement(in);
    // no valid source this cycle: coast on the prediction
    if (measurement.z.size() != 0) {
        update(measurement);
    }

    // this cycle ends now
    m_tUs = in.tUs;

    publish(stepIncrement(before, t0Us));

}

const OdomEkf::State& OdomEkf::state() const {
    return m_s;
}

const OdomEkf::Cov& OdomEkf::covariance() const {
    return m_P;
}

void OdomEkf::reset(const Pose& pose) {
    if (!std::isfinite(pose.x) || !std::isfinite(pose.y) || !std::isfinite(pose.theta)) {
        return;
    }

    // zero everything, so v_x, v_y, omega start at rest
    m_s.setZero();

    // new pose in the odometry frame, heading wrapped to (-pi, pi]
    m_s[0] = pose.x;
    m_s[1] = pose.y;
    m_s[2] = utils::wrapAngle(pose.theta);

    // drop the old covariance and correlations, start from the initial P
    m_P = initialP;

    // TODO (#185): clear the increment buffer, the odometry frame jumped so old increments can't be composed across it
}

void OdomEkf::onPose(PoseCallback cb) {
    m_poseCallbacks.push_back(std::move(cb));
}

void OdomEkf::onIncrement(IncrementCallback cb) {
    m_incrementCallbacks.push_back(std::move(cb));
}

std::optional<PoseIncrement> OdomEkf::increment(int64_t t0Us, int64_t t1Us) const {
    // TODO (#185): reject t0Us >= t1Us, or a span older than the oldest buffered increment
    // TODO (#185): walk the ring buffer from oldest to newest, keeping increments inside [t0Us, t1Us]
    // TODO (#185): delta = compose(delta, inc.delta), cov = J_A cov J_Aᵀ + J_B inc.cov J_Bᵀ (se2::composeJacA / composeJacB)
    // TODO (#185): cycles treated as independent (no cross terms), an approximation
    return std::nullopt;
}

Twist OdomEkf::twist() const {
    const double v = m_s[3];
    const double omega = m_s[5];
    Eigen::Matrix2d cov;
    cov << m_P(3, 3), m_P(3, 5),
           m_P(5, 3), m_P(5, 5);
    return Twist{m_tUs, v, omega, cov};
}

PoseIncrement OdomEkf::stepIncrement(const State& before, int64_t t0Us) const {
    // pose at the start and end of this cycle, odometry frame
    const Pose p0{before[0], before[1], before[2]};
    const Pose p1{m_s[0], m_s[1], m_s[2]};

    // motion from p0 to p1 in p0's frame, theta wrapped
    const Pose delta = se2::between(p0, p1);

    // rotate x, y into p0's frame
    Eigen::Matrix3d J = Eigen::Matrix3d::Identity();
    J.topLeftCorner<2, 2>() = Eigen::Rotation2Dd(-before[2]).toRotationMatrix();
    const Eigen::Matrix3d cov = J * m_Pstep.topLeftCorner<3, 3>() * J.transpose();


    return PoseIncrement{t0Us, m_tUs, delta, cov};
}

void OdomEkf::publish(const PoseIncrement& inc) {
    // TODO (#185): write inc into m_increments at m_incrementHead, advance the head, grow m_incrementCount up to INCREMENT_HISTORY
    // TODO (#185): call every pose callback with StampedPose{m_tUs, pose of m_s, P.topLeftCorner<3, 3>()}
    // TODO (#185): call every increment callback with inc
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

    // process noise is a change in velocity this cycle
    const double accelStep = ODOM_EKF_ACCEL_SIGMA * dt; // in/s the speed may change this cycle
    const double angAccelStep = ODOM_EKF_ANG_ACCEL_SIGMA * dt; // rad/s the turn rate may change this cycle

    Eigen::Matrix3d Qv = Eigen::Matrix3d::Zero();
    Qv(0, 0) = accelStep * accelStep;
    Qv(1, 1) = accelStep * accelStep;
    Qv(2, 2) = angAccelStep * angAccelStep;

    // G maps a velocity change (vX, vY, omega) into the state, with the same rotation as the prediction
    Eigen::Matrix<double, 6, 3> G = Eigen::Matrix<double, 6, 3>::Zero();
    G(0, 0) = dt * cosThetaMid;
    G(0, 1) = -dt * sinThetaMid;
    G(1, 0) = dt * sinThetaMid;
    G(1, 1) = dt * cosThetaMid;
    G(2, 2) = dt;
    G(3, 0) = 1.0;
    G(4, 1) = 1.0;
    G(5, 2) = 1.0;


    // new state, velocities stay constant
    m_s[0] += dx;
    m_s[1] += dy;
    m_s[2] = utils::wrapAngle(theta + omega * dt);

    // propagate the uncertainty and add this cycle's process noise, in both velocity and pose
    m_P = F * m_P * F.transpose() + G * Qv * G.transpose();
    // same propagation for this cycle's covariance
    m_Pstep = F * m_Pstep * F.transpose() + G * Qv * G.transpose();

    // remove rounding asymmetry
    m_P = ((m_P + m_P.transpose()) / 2.0).eval();
    m_Pstep = ((m_Pstep + m_Pstep.transpose()) / 2.0).eval();

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
    // nothing to correct with
    if (m.z.size() == 0) {
        return;
    }

    // rows this cycle
    const int rows = static_cast<int>(m.z.size());
    // indices of rows that passed the gate
    std::array<int, MAX_ROWS> keptRows;
    // how many of keptRows are filled
    int keptCount = 0;

    // column i is P * H_i^T
    const Gain PHt = m_P * m.H.transpose();

    // drop any row whose disagreement with the prediction is too big to be noise
    for (int i = 0; i < rows; ++i) {
        // this row of H, 1 x 6, picks the states the source measures
        const auto hRow = m.H.row(i);
        // innovation, measured minus predicted
        const double nuI = m.z(i) - hRow.dot(m_s);
        // S(i, i), how big the innovation should be: state uncertainty plus sensor noise
        const double sII = hRow.dot(PHt.col(i)) + m.R(i, i);
        // innovation in its own standard deviations
        const double d2 = nuI * nuI / sII;
        if (d2 <= ODOM_EKF_ROW_GATE) {
            keptRows[keptCount++] = i;
        }
    }

    // every source disagreed, so coast on the prediction
    if (keptCount == 0) {
        return;
    }

    // the surviving rows, packed together
    MeasVec zKept(keptCount);
    MeasMat hKept(keptCount, 6);
    MeasCov rKept = MeasCov::Zero(keptCount, keptCount);
    Gain PHtKept(6, keptCount);

    // copy each kept row down, PHt's column is reused instead of recomputed
    for (int i = 0; i < keptCount; ++i) {
        const int src = keptRows[i];
        zKept(i) = m.z(src);
        hKept.row(i) = m.H.row(src);
        rKept(i, i) = m.R(src, src);
        PHtKept.col(i) = PHt.col(src);
    }

    // how far off the prediction was, one entry per kept row
    const MeasVec nu = zKept - hKept * m_s;
    // innovation covariance, off-diagonals say how much the rows see the same uncertainty
    const MeasCov S = hKept * PHtKept + rKept;
    // gain, 6 x keptCount, how far to move each state per unit of each row's innovation
    const Gain K = S.ldlt().solve(PHtKept.transpose()).transpose();

    // correct the state, then keep heading in (-pi, pi]
    m_s += K * nu;
    m_s[2] = utils::wrapAngle(m_s[2]);

    // Joseph form, a sum of two symmetric products
    const Cov ikh = Cov::Identity() - K * hKept;
    m_P = ikh * m_P * ikh.transpose() + K * rKept * K.transpose();
    // same correction for this cycle's covariance, with the gain from the full P
    m_Pstep = ikh * m_Pstep * ikh.transpose() + K * rKept * K.transpose();


    // remove rounding asymmetry
    m_P = ((m_P + m_P.transpose()) / 2.0).eval();
    m_Pstep = ((m_Pstep + m_Pstep.transpose()) / 2.0).eval();
}
