#include "localization/ekf/mockEkf.hpp"

#include <cmath>
#include <utility>

#include "utils/mathHelpers.hpp"

MockEkf::MockEkf(const DriftConfig& drift, double odomRateHz, double fusionGain, uint32_t seed)
    : m_drift(drift), m_odomPeriodUs(1e6 / odomRateHz), m_fusionGain(fusionGain), m_rng(seed) {}

void MockEkf::step(const StampedPose& truth) {
    if (!m_started) {
        // first call, start both poses at truth
        m_odom = truth.pose;
        m_main = truth.pose;
        // publish the first sample
        m_nextPublishUs = truth.tUs;
        m_started = true;
    }
    else {
        // true motion since the last step, in the old pose's frame
        Pose delta = se2::between(m_lastTruth.pose, truth.pose);
        // time since the last step (s)
        const double dt = static_cast<double>(truth.tUs - m_lastTruth.tUs) / 1e6;

        // drift: scale error, random walk, heading bias
        delta.x *= (1.0 + m_drift.scaleError);
        delta.x += m_drift.randomWalkIn * m_noise(m_rng);
        delta.y *= (1.0 + m_drift.scaleError);
        delta.y += m_drift.randomWalkIn * m_noise(m_rng);
        delta.theta += m_drift.headingBiasRadPerS * dt;

        // move both poses by the drifted motion
        m_odom = se2::compose(m_odom, delta);
        m_main = se2::compose(m_main, delta);
    }

    // save main and drop entries older than the history window
    m_history.push_back(StampedPose{truth.tUs, m_main, poseCov()});
    while (m_history.back().tUs - m_history.front().tUs > MOCK_EKF_HISTORY_US) {
        m_history.pop_front();
    }

    // publish odom when it is due
    if (truth.tUs >= m_nextPublishUs) {
        const StampedPose odom{truth.tUs, m_odom, poseCov()};
        for (const PoseCallback& cb : m_odomCallbacks) {
            cb(odom);
        }
        // next publish time after now, skips any missed while steps were slow
        while (m_nextPublishUs <= truth.tUs) {
            m_nextPublishUs += static_cast<int64_t>(m_odomPeriodUs);
        }
    }

    // truth for the next step
    m_lastTruth = truth;
}

void MockEkf::onOdomPose(PoseCallback cb) {
    m_odomCallbacks.push_back(std::move(cb));
}

std::optional<StampedPose> MockEkf::mainPoseAt(int64_t tUs) const {
    // empty, or outside the history
    if (m_history.empty() || tUs < m_history.front().tUs || tUs > m_history.back().tUs) {
        return std::nullopt;
    }

    // binary search for the first entry at or after tUs
    int lo = 0;
    int hi = static_cast<int>(m_history.size()) - 1;
    while (lo < hi) {
        const int mid = (lo + hi) / 2;
        if (m_history[mid].tUs < tUs) {
            lo = mid + 1;
        } 
        else {
            hi = mid;
        }
    }

    // exact hit
    const StampedPose& b = m_history[lo];
    if (b.tUs == tUs) {
        return b;
    }

    // entry just before tUs
    const StampedPose& a = m_history[lo - 1];
    // fraction of the way from a to b
    const double alpha = static_cast<double>(tUs - a.tUs) / static_cast<double>(b.tUs - a.tUs);

    // x and y linear, heading the short way around
    const Pose p{a.pose.x + alpha * (b.pose.x - a.pose.x),
                 a.pose.y + alpha * (b.pose.y - a.pose.y),
                 utils::wrapAngle(a.pose.theta + alpha * utils::wrapAngle(b.pose.theta - a.pose.theta))};

    // covariance from the nearer entry
    return StampedPose{tUs, p, alpha < 0.5 ? a.cov : b.cov};
}

StampedPose MockEkf::mainPoseLatest() const {
    // nothing stepped yet, no real time
    if (m_history.empty()) {
        return StampedPose{0, m_main, poseCov()};
    }
    // newest main pose
    return m_history.back();
}

void MockEkf::submitLidarMeasurement(const LidarMeasurement& m) {
    // record it for test checks
    m_submitted.push_back(m);
    // move main part of the way toward the measurement, heading the short way around
    m_main.x += m_fusionGain * (m.pose.x - m_main.x);
    m_main.y += m_fusionGain * (m.pose.y - m_main.y);
    m_main.theta = utils::wrapAngle(m_main.theta + m_fusionGain * utils::wrapAngle(m.pose.theta - m_main.theta));
}

void MockEkf::onPoseReset(PoseCallback cb) {
    m_resetCallbacks.push_back(std::move(cb));
}

const std::vector<LidarMeasurement>& MockEkf::submitted() const {
    return m_submitted;
}

void MockEkf::firePoseReset(const StampedPose& p) {
    // reset before the first step(), start from p instead of the first truth
    if (!m_started) {
        m_lastTruth = p;
        m_nextPublishUs = p.tUs;
        m_started = true;
    }

    // both filters start over at the new pose
    m_odom = p.pose;
    m_main = p.pose;

    // forget poses from before the reset
    m_history.clear();
    m_history.push_back(p);

    // tell subscribers after the pose is set
    for (const PoseCallback& reset : m_resetCallbacks) {
        reset(p);
    }
}

Eigen::Matrix3d MockEkf::poseCov() {
    // variances on the diagonal, x, y, θ order
    return Eigen::Vector3d(MOCK_EKF_POSE_SIGMA_IN * MOCK_EKF_POSE_SIGMA_IN,
                           MOCK_EKF_POSE_SIGMA_IN * MOCK_EKF_POSE_SIGMA_IN,
                           MOCK_EKF_POSE_SIGMA_RAD * MOCK_EKF_POSE_SIGMA_RAD).asDiagonal();
}
