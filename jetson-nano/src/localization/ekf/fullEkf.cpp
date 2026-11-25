#include "localization/ekf/fullEkf.hpp"

#include <algorithm>
#include <cmath>
#include <mutex>

#include "utils/mathHelpers.hpp"

FullEkf::FullEkf(const FullEkfNoise& noise) : m_noise(noise) {}


void FullEkf::setPose(const StampedPose& p) {
    // skip a pose or covariance with a NaN or infinity
    if (!(std::isfinite(p.pose.x) && std::isfinite(p.pose.y) && std::isfinite(p.pose.theta) && p.cov.allFinite())) {
        return;
    }
    
    // Skip invalid covariance matrices (not positive definite)
    if (p.cov.llt().info() != Eigen::Success) {
        return;
    }

    // drop measurements queued before the reset
    {
        std::lock_guard<std::mutex> lock(m_pendingMutex);
        m_pending.clear();
    }

    // the stored state, handed to subscribers after the state lock is released
    StampedPose reset;

    {
        std::lock_guard<std::mutex> lock(m_stateMutex);
        // new state, heading wrapped
        m_x = Pose{p.pose.x, p.pose.y, utils::wrapAngle(p.pose.theta)};
        // new covariance, rounding asymmetry removed
        m_P = ((p.cov + p.cov.transpose()) / 2.0).eval();
        // rejections from before the reset don't count
        m_rejectStreak = 0;

        // forget cycles from before the reset
        m_histHead = 0;
        m_histCount = 0;

        // measurements stamped before the reset now count as too old
        m_tUs = p.tUs;
        // start state as the first history entry, no motion led to it
        pushHistory(PoseIncrement{p.tUs, p.tUs, Pose{0.0, 0.0, 0.0}, Eigen::Matrix3d::Zero()});

         // copy what the filter stored, wrapped heading and symmetric P
        reset = StampedPose{m_tUs, m_x, m_P};
    }

    // onPoseReset() may add subscribers from another thread
    std::lock_guard<std::mutex> lock(m_callbackMutex);
    // tell subscribers after the state is set, outside m_stateMutex
    for (const PoseCallback& cb : m_resetCallbacks) {
        cb(reset);
    }
}

void FullEkf::onPoseReset(PoseCallback cb) {
    std::lock_guard<std::mutex> lock(m_callbackMutex);
    // keep every subscriber, each gets every reset
    m_resetCallbacks.push_back(std::move(cb));
}


void FullEkf::predict(const PoseIncrement& inc) {
    // apply queued measurements first
    drainQueue();

    // skip an increment with a NaN or infinity
    if (!(std::isfinite(inc.delta.x) && std::isfinite(inc.delta.y) && std::isfinite(inc.delta.theta) && inc.cov.allFinite())) {
        return;
    }

    std::lock_guard<std::mutex> lock(m_stateMutex);

    // move the state forward one cycle
    propagate(inc);
    // save this cycle for late measurements
    pushHistory(inc);
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
        // a measurement from the past is applied at its own time
        m_lastUpdate = (m.tUs <= m_tUs) ? applyLate(m) : update(m);

    }
}

std::optional<StampedPose> FullEkf::poseAt(int64_t tUs) const {
    std::lock_guard<std::mutex> lock(m_stateMutex);
    // first entry at or after tUs, nothing if empty or too old
    const std::optional<int> found = findEntry(tUs);
    if (found == std::nullopt) {
        return std::nullopt;
    }

    // oldest slot in the ring, then entry *found
    const int oldest = (m_histHead - m_histCount + FULL_EKF_HISTORY_SIZE) % FULL_EKF_HISTORY_SIZE;
    const HistoryEntry& b = m_history[(oldest + *found) % FULL_EKF_HISTORY_SIZE];
    // newer than the newest entry
    if (b.tUs < tUs) {
        return std::nullopt;
    }

    // exact hit
    if (b.tUs == tUs) {
        return StampedPose{b.tUs, b.x, b.P};
    }

    // entry just before tUs
    const HistoryEntry& a = m_history[(oldest + *found - 1) % FULL_EKF_HISTORY_SIZE];
    // fraction of the way from a to b
    const double alpha = static_cast<double>(tUs - a.tUs) / static_cast<double>(b.tUs - a.tUs);
    // x and y linear, heading the short way around
    const Pose p {(a.x.x + alpha * (b.x.x - a.x.x)), (a.x.y + alpha * (b.x.y - a.x.y)), utils::wrapAngle(a.x.theta + alpha * utils::wrapAngle(b.x.theta - a.x.theta))};
    // covariance from the nearer entry
    return StampedPose{tUs, p, alpha < 0.5 ? a.P : b.P};
}

int FullEkf::tooOldCount() const {
    std::lock_guard<std::mutex> lock(m_stateMutex);
    return m_tooOld;
}

void FullEkf::propagate(const PoseIncrement& inc) {
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

void FullEkf::pushHistory(const PoseIncrement& inc) {
    // store the state after this cycle and the increment that led to it
    m_history[m_histHead] = HistoryEntry{m_tUs, m_x, m_P, inc};
    // move the head forward
    m_histHead = (m_histHead + 1) % FULL_EKF_HISTORY_SIZE;
    // count up until the buffer is full
    m_histCount = std::min(m_histCount + 1, FULL_EKF_HISTORY_SIZE);
}

std::optional<int> FullEkf::findEntry(int64_t tUs) const {
    // oldest slot in the ring
    const int oldest = (m_histHead - m_histCount + FULL_EKF_HISTORY_SIZE) % FULL_EKF_HISTORY_SIZE;
    // nothing stored, or older than the history
    if (m_histCount == 0 || tUs < m_history[oldest].tUs) {
        return std::nullopt;
    }

    // binary search
    int lo = 0;
    int hi = m_histCount - 1;
    while (lo < hi) {
        const int mid = (lo + hi) / 2;
        if (m_history[(oldest + mid) % FULL_EKF_HISTORY_SIZE].tUs < tUs) {
            lo = mid + 1;
        } 
        else {
            hi = mid;
        }
    }
    // first entry at or after tUs
    return lo;
}

UpdateResult FullEkf::applyLate(const PoseMeasurement& m) {
    // first entry at or after m.tUs
    std::optional<int> found = findEntry(m.tUs);
    if (found == std::nullopt) {
        // older than the history, drop and count it
        ++m_tooOld;
        return UpdateResult{m.tUs, m.source};
    }

    // oldest slot in the ring
    const int oldest = (m_histHead - m_histCount + FULL_EKF_HISTORY_SIZE) % FULL_EKF_HISTORY_SIZE;
    // apply at the nearer cycle boundary
    if (*found > 0) {
        const HistoryEntry& prev = m_history[(oldest + *found - 1) % FULL_EKF_HISTORY_SIZE];
        const HistoryEntry& curr = m_history[(oldest + *found) % FULL_EKF_HISTORY_SIZE];
        if (m.tUs - prev.tUs < curr.tUs - m.tUs) {
            --*found;
        }
    }

    // restore the state at that cycle
    HistoryEntry& e = m_history[(oldest + *found) % FULL_EKF_HISTORY_SIZE];
    m_x = e.x;
    m_P = e.P;
    m_tUs = e.tUs;
    // correct it and save it back
    const UpdateResult result = update(m);
    e.x = m_x;
    e.P = m_P;

    // replay the later cycles up to now
    for (int i = *found + 1; i < m_histCount; ++i) {
        HistoryEntry& f = m_history[(oldest + i) % FULL_EKF_HISTORY_SIZE];
        propagate(f.inc);
        f.x = m_x;
        f.P = m_P;
    }
    return result;
}
