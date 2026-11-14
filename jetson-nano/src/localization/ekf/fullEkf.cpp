#include "localization/ekf/fullEkf.hpp"

#include <algorithm>
#include <cmath>
#include <mutex>

#include "config/constants.h"
#include "utils/mathHelpers.hpp"

FullEkf::FullEkf(const FullEkfNoise& noise) : m_noise(noise) {}

void FullEkf::predict(const PoseIncrement& inc) {
    // apply queued measurements first
    drainQueue();

    // skip an increment with a NaN or infinity
    if (!(std::isfinite(inc.delta.x) && std::isfinite(inc.delta.y) && std::isfinite(inc.delta.theta) && inc.cov.allFinite())) {
        return;
    }

    std::lock_guard<std::mutex> lock(m_stateMutex);

    // Jacobians at the old pose
    const Eigen::Matrix3d F = se2::composeJacA(m_x, inc.delta);
    const Eigen::Matrix3d G = se2::composeJacB(m_x);

    // move the pose by this cycle's increment
    m_x = se2::compose(m_x, inc.delta);

    // propagate the covariance
    m_P = F * m_P * F.transpose() + G * inc.cov * G.transpose() + m_noise.qExtra;
    // remove rounding asymmetry
    m_P = ((m_P + m_P.transpose()) / 2.0).eval();

    // time of the new pose
    m_tUs = inc.t1Us;
}

void FullEkf::submit(const PoseMeasurement& m) {
    // queue it, the EKF thread applies it on the next predict()
    std::lock_guard<std::mutex> lock(m_pendingMutex);
    m_pending.push_back(m);
}

StampedPose FullEkf::latest() const {
    // the EKF thread may be writing m_x and m_P, so wait for it to finish
    std::lock_guard<std::mutex> lock(m_stateMutex);
    // copy out while locked
    return StampedPose{m_tUs, m_x, m_P};
}

UpdateResult FullEkf::lastUpdate() const {
    // update() writes m_lastUpdate under the lock
    std::lock_guard<std::mutex> lock(m_stateMutex);
    return m_lastUpdate;
}

UpdateResult FullEkf::update(const PoseMeasurement& m) {
    // not accepted until it passes the gate
    UpdateResult result = {m.tUs, m.source};

    // skip a measurement with a NaN or infinity
    if (!(std::isfinite(m.pose.x) && std::isfinite(m.pose.y) && std::isfinite(m.pose.theta) && m.cov.allFinite())) {
        return result;
    }

    // innovation, measured minus predicted, heading wrapped
    Eigen::Vector3d nu;
    nu(0) = m.pose.x - m_x.x;
    nu(1) = m.pose.y - m_x.y;
    nu(2) = utils::wrapAngle(m.pose.theta - m_x.theta);

    // innovation covariance, filter uncertainty plus measurement noise
    const Eigen::Matrix3d S = m_P + m.cov;
    // factor S once for both solves
    const Eigen::LDLT<Eigen::Matrix3d> sLdlt = S.ldlt();

    // Mahalanobis distance squared
    const double d2 = nu.dot(sLdlt.solve(nu));
    result.nis = d2;

    // reject above the gate or if d² isn't a number
    if (d2 > m_noise.gate || !std::isfinite(d2)) {
        return result;
    }

    // Kalman gain, P S⁻¹
    const Eigen::Matrix3d K = sLdlt.solve(m_P).transpose();

    // move the pose by gain times innovation, heading wrapped
    const Eigen::Vector3d dx = K * nu;
    m_x.x += dx(0);
    m_x.y += dx(1);
    m_x.theta = utils::wrapAngle(m_x.theta + dx(2));

    // Joseph form covariance update
    const Eigen::Matrix3d ikh = Eigen::Matrix3d::Identity() - K;
    m_P = ikh * m_P * ikh.transpose() + K * m.cov * K.transpose();
    // remove rounding asymmetry
    m_P = ((m_P + m_P.transpose()) / 2.0).eval();

    // delayed measurements are applied as if current
    result.accepted = true;
    return result;
}

void FullEkf::drainQueue() {
    // TODO (#186): swap m_pending into a local vector under m_pendingMutex, so submit() isn't blocked while updating
    // TODO (#186): sort by tUs, then update() each under m_stateMutex
}
