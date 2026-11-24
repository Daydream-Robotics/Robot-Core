#include "localization/ekf/mockEkf.hpp"

#include <cmath>

#include "utils/mathHelpers.hpp"

MockEkf::MockEkf(const DriftConfig& drift, double odomRateHz, double fusionGain, uint32_t seed)
    : m_drift(drift), m_odomPeriodUs(1e6 / odomRateHz), m_fusionGain(fusionGain), m_rng(seed) {}

void MockEkf::step(const StampedPose& truth) {
    // TODO: first call -> odom and main = truth.pose, store m_lastTruth, push history,
    //       m_nextPublishUs = truth.tUs, then fall through to the publish
    // TODO: delta = se2::between(m_lastTruth.pose, truth.pose), dt in seconds
    // TODO: drift -> x and y times (1 + scaleError), theta += headingBiasRadPerS * dt,
    //       x and y += randomWalkIn * m_noise(m_rng)
    // TODO: m_odom and m_main = se2::compose(pose, delta)
    // TODO: push {truth.tUs, m_main, poseCov()}, pop the front while older than MOCK_EKF_HISTORY_US
    // TODO: truth.tUs >= m_nextPublishUs -> call every odom callback, m_nextPublishUs += m_odomPeriodUs
    // TODO: m_lastTruth = truth
    (void)truth;
}

void MockEkf::onOdomPose(PoseCallback cb) {
    // TODO: add cb to m_odomCallbacks
    (void)cb;
}

std::optional<StampedPose> MockEkf::mainPoseAt(int64_t tUs) const {
    // TODO: empty or tUs outside [front, back] -> nullopt
    // TODO: linear scan for the first entry with time >= tUs, exact hit -> that entry
    // TODO: interpolate with the entry before, theta with the wrapped difference
    (void)tUs;
    return std::nullopt;
}

StampedPose MockEkf::mainPoseLatest() const {
    // TODO: back of m_history, or {0, m_main, poseCov()} if empty
    return StampedPose{0, m_main, poseCov()};
}

void MockEkf::submitLidarMeasurement(const LidarMeasurement& m) {
    // TODO: record m in m_submitted
    // TODO: pull m_main toward m.pose by m_fusionGain, theta with the wrapped difference
    (void)m;
}

void MockEkf::onPoseReset(PoseCallback cb) {
    // TODO: add cb to m_resetCallbacks
    (void)cb;
}

const std::vector<LidarMeasurement>& MockEkf::submitted() const {
    return m_submitted;
}

void MockEkf::firePoseReset(const StampedPose& p) {
    // TODO: m_odom and m_main = p.pose, clear the history and push p
    // TODO: call every reset callback with p
    (void)p;
}

Eigen::Matrix3d MockEkf::poseCov() {
    // TODO: diagonal of MOCK_EKF_POSE_SIGMA_IN², MOCK_EKF_POSE_SIGMA_IN², MOCK_EKF_POSE_SIGMA_RAD²
    return Eigen::Matrix3d::Identity();
}
