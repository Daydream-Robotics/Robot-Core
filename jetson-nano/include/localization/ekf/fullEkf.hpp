#pragma once

#include <array>
#include <cstdint>
#include <mutex>
#include <optional>
#include <vector>

#include <Eigen/Dense>

#include "config/constants.h"
#include "localization/ekf/se2.hpp"
#include "localization/odometry.hpp"

/**
 * @brief Where an absolute pose measurement came from. #188 adds per-source settings and counters.
 */
enum class MeasurementSource : uint8_t {
    ICP ///< KISS-ICP scan-to-map match
};

/**
 * @brief Absolute pose measurement for the full EKF.
 */
struct PoseMeasurement {
    int64_t tUs = 0; ///< time the measurement describes (NOT arrival time), Nano steady_clock (us)
    Pose pose{0.0, 0.0, 0.0}; ///< field frame
    Eigen::Matrix3d cov = Eigen::Matrix3d::Zero(); ///< covariance of pose (x, y, θ order, in² and rad²)
    MeasurementSource source{}; ///< source
};

/**
 * @brief Result of one measurement update, for logging and consistency checks.
 */
struct UpdateResult {
    int64_t tUs = 0; ///< time of the measurement, Nano steady_clock (us)
    MeasurementSource source{}; ///< which source it came from
    bool accepted = false; ///< passed the gate and was applied
    double nis = 0.0; ///< d² = νᵀ S⁻¹ ν
};

/**
 * @brief Noise settings for FullEkf. Starting values go in constants.h ([FULL EKF] section).
 */
struct FullEkfNoise {
    ///< extra inflation per predict for what odometry can't see, such as collisions
    Eigen::Matrix3d qExtra = Eigen::Vector3d(FULL_EKF_Q_EXTRA_XY_SIGMA * FULL_EKF_Q_EXTRA_XY_SIGMA,
                                             FULL_EKF_Q_EXTRA_XY_SIGMA * FULL_EKF_Q_EXTRA_XY_SIGMA,
                                             FULL_EKF_Q_EXTRA_THETA_SIGMA * FULL_EKF_Q_EXTRA_THETA_SIGMA).asDiagonal();
    double gate = FULL_EKF_GATE; ///< reject an update with d² above this, starts at χ² 3-DOF 99%
    int recoveryRejects = FULL_EKF_RECOVERY_REJECTS; ///< consecutive rejections before P is raised to recoveryP
    ///< P is raised to at least this after recoveryRejects consecutive rejections
    Eigen::Matrix3d recoveryP = Eigen::Vector3d(FULL_EKF_RECOVERY_XY_SIGMA * FULL_EKF_RECOVERY_XY_SIGMA,
                                                FULL_EKF_RECOVERY_XY_SIGMA * FULL_EKF_RECOVERY_XY_SIGMA,
                                                FULL_EKF_RECOVERY_THETA_SIGMA * FULL_EKF_RECOVERY_THETA_SIGMA).asDiagonal();
};


/**
 * @brief Full EKF: odometry prediction plus gated absolute pose updates, in the field frame.
 *
 * State is (x, y, theta) in the field frame. predict() runs in the EKF's thread, submit() and
 * latest() may be called from any thread.
 */
class FullEkf {
public:
    /**
     * @brief Constructs the filter at the field origin. Call reset() with the start pose before use.
     *
     * @param noise Process inflation, gate, and recovery settings.
     */
    explicit FullEkf(const FullEkfNoise& noise);

    /**
     * @brief Sets the field pose and its covariance, and drops queued measurements (thread-safe).
     *
     * Call with the known start position for the match or auton. Ignored if the pose or
     * covariance has a NaN or infinity.
     *
     * @param pose Field pose.
     * @param cov Covariance of pose (x, y, θ order, in² and rad²).
     */
    void reset(const Pose& pose, const Eigen::Matrix3d& cov);


    /**
     * @brief Applies queued measurements, then predicts forward by one odometry increment.
     *
     * @param inc One cycle's motion in the previous pose's frame, from OdomEkf::onIncrement().
     */
    void predict(const PoseIncrement& inc);

    /**
     * @brief Queues an absolute measurement (thread-safe). Applied on the next predict().
     *
     * @param m Field-frame pose measurement.
     */
    void submit(const PoseMeasurement& m);

    /**
     * @brief Latest field pose with covariance (thread-safe).
     *
     * @return StampedPose Pose, its time, and P.
     */
    StampedPose latest() const;

    /**
     * @brief Result of the most recent update, accepted or not (thread-safe).
     *
     * @return UpdateResult Default-constructed if no measurement has been applied yet.
     */
    UpdateResult lastUpdate() const;

    /**
     * @brief Pose at a past time, interpolated from history (thread-safe).
     *
     * @param tUs Time on the Nano's steady_clock (us).
     * @return std::optional<StampedPose> Nothing if tUs is outside the history.
     */
    std::optional<StampedPose> poseAt(int64_t tUs) const;

    /**
     * @brief Measurements dropped for being older than the history (thread-safe).
     *
     * @return int Count since construction.
     */
    int tooOldCount() const;

private:
    /**
     * @brief One cycle of filter history.
     */
    struct HistoryEntry {
        int64_t tUs = 0; ///< time of this state, Nano steady_clock (us)
        Pose x{0.0, 0.0, 0.0}; ///< field pose after this cycle
        Eigen::Matrix3d P = Eigen::Matrix3d::Identity(); ///< covariance of x
        PoseIncrement inc; ///< the increment that led to this entry
    };

    /**
     * @brief Moves m_x and m_P forward by one increment. Caller holds m_stateMutex.
     *
     * @param inc One cycle's motion in the previous pose's frame.
     */
    void propagate(const PoseIncrement& inc);

    /**
     * @brief Stores the current state and the increment that led to it. Caller holds m_stateMutex.
     *
     * @param inc The increment just applied.
     */
    void pushHistory(const PoseIncrement& inc);

    /**
     * @brief Finds the first history entry at or after a time. Caller holds m_stateMutex.
     *
     * @param tUs Time to look up.
     * @return std::optional<int> Logical index, 0 is the oldest. Nothing if the history is empty or tUs is older than it.
     */
    std::optional<int> findEntry(int64_t tUs) const;

    /**
     * @brief Applies a measurement from the past and replays the increments since. Caller holds m_stateMutex.
     *
     * @param m Measurement with tUs older than m_tUs.
     * @return UpdateResult Whether it was accepted, and its NIS.
     */
    UpdateResult applyLate(const PoseMeasurement& m);

    /**
     * @brief Corrects the state with one absolute measurement, if it passes the gate.
     *
     * @param m Measurement to apply.
     * @return UpdateResult Whether it was accepted, and its NIS.
     */
    UpdateResult update(const PoseMeasurement& m);

    /**
     * @brief Takes every queued measurement and applies it with update(), oldest first.
     */
    void drainQueue();


    Pose m_x{0.0, 0.0, 0.0}; ///< field pose
    Eigen::Matrix3d m_P = Eigen::Matrix3d::Identity(); ///< covariance of m_x (x, y, θ order, in² and rad²)
    int64_t m_tUs = 0; ///< time of the latest predict, Nano steady_clock (us)
    FullEkfNoise m_noise; ///< process inflation and gate
    UpdateResult m_lastUpdate; ///< most recent update, for lastUpdate()
    int m_rejectStreak = 0; ///< consecutive gate rejections, reset on an accepted update
    std::array<HistoryEntry, FULL_EKF_HISTORY_SIZE> m_history{}; ///< ring buffer of past cycles
    int m_histHead = 0; ///< slot the next entry is written to
    int m_histCount = 0; ///< number of stored entries, up to FULL_EKF_HISTORY_SIZE
    int m_tooOld = 0; ///< measurements dropped for being older than the history
    mutable std::mutex m_stateMutex; ///< guards m_x, m_P, m_tUs, m_lastUpdate, m_rejectStreak, history
    std::vector<PoseMeasurement> m_pending; ///< measurements waiting for the next predict()
    std::mutex m_pendingMutex; ///< guards m_pending, submit() runs on other threads
};
