#include "localization/ekf/fullEkf.hpp"

#include <algorithm>
#include <cmath>
#include <mutex>

#include "utils/mathHelpers.hpp"

FullEkf::FullEkf(const FullEkfNoise& noise) : m_noise(noise) {}


void FullEkf::reset(const Pose& pose, const Eigen::Matrix3d& cov) {
    // skip a pose or covariance with a NaN or infinity
    if (!(std::isfinite(pose.x) && std::isfinite(pose.y) && std::isfinite(pose.theta) && cov.allFinite())) {
        return;
    }

    // drop measurements queued before the reset
    {
        std::lock_guard<std::mutex> lock(m_pendingMutex);
        m_pending.clear();
    }

    std::lock_guard<std::mutex> lock(m_stateMutex);
    m_x = Pose{pose.x, pose.y, utils::wrapAngle(pose.theta)};
    m_P = ((cov + cov.transpose()) / 2.0).eval();
    m_rejectStreak = 0;
}


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

    // skip a measurement with a NaN or infinity, or a variance that isn't positive
    if (!(std::isfinite(m.pose.x) && std::isfinite(m.pose.y) && std::isfinite(m.pose.theta) && m.cov.allFinite())
        || !(m.cov.diagonal().array() > 0.0).all()) {
        return result;
    }


    // TODO (#187): m.tUs is older than m_x, apply it at m.tUs from the history and replay the increments since
    // innovation, measured minus predicted, heading wrapped
    Eigen::Vector3d nu;
    nu(0) = m.pose.x - m_x.x;
    nu(1) = m.pose.y - m_x.y;
    nu(2) = utils::wrapAngle(m.pose.theta - m_x.theta);

    // after too many rejections in a row, raise P so the gate reopens
    if (m_rejectStreak >= m_noise.recoveryRejects) {
        m_P = m_P.cwiseMax(m_noise.recoveryP);
        m_rejectStreak = 0;
    }

    // innovation covariance, filter uncertainty plus measurement noise
    const Eigen::Matrix3d S = m_P + m.cov;
    // factor S once for both solves
    const Eigen::LDLT<Eigen::Matrix3d> sLdlt = S.ldlt();
    // reject if S can't be factored or isn't positive definite
    if (sLdlt.info() != Eigen::Success || !sLdlt.isPositive()) {
        return result;
    }

    // Mahalanobis distance squared
    const double d2 = nu.dot(sLdlt.solve(nu));
    result.nis = d2;

    // reject if d² isn't a number or is negative
    if (!std::isfinite(d2) || d2 < 0.0) {
        return result;
    }

    // reject above the gate
    if (d2 > m_noise.gate) {
        ++m_rejectStreak;
        return result;
    }
    m_rejectStreak = 0;

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

    result.accepted = true;
    return result;
}

void FullEkf::drainQueue() {
    // take every queued measurement, m_pending is left empty
    std::vector<PoseMeasurement> batch;
    {
        std::lock_guard<std::mutex> lock(m_pendingMutex);
        batch.swap(m_pending);
    }

    // nothing queued this cycle
    if (batch.empty()) {
        return;
    }

    // oldest first
    std::sort(batch.begin(), batch.end(), [](const PoseMeasurement& a, const PoseMeasurement& b) {
        return a.tUs < b.tUs;
    });

    // apply each one and keep the latest result
    std::lock_guard<std::mutex> lock(m_stateMutex);
    for (const PoseMeasurement& m : batch) {
        m_lastUpdate = update(m);
    }
}
