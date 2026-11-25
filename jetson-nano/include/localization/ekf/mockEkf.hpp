#pragma once

#include <cstdint>
#include <deque>
#include <optional>
#include <random>
#include <vector>

#include "config/constants.h"
#include "localization/ekf/ekfInterface.hpp"
#include "localization/ekf/se2.hpp"
#include "localization/odometry.hpp"

/**
 * @brief Stand-in EKF driven by ground truth, so the LiDAR code can be tested before the real EKF is wired in.
 *
 * odom_ekf_ is truth plus drift. main_ekf_ follows odom's motion and is pulled toward each
 * submitted measurement by a fixed gain. Single-threaded, for tests and the simulator, so no mutex.
 */
class MockEkf : public EkfInterface {
public:
    /**
     * @brief How odom_ekf_ drifts from ground truth.
     */
    struct DriftConfig {
        double scaleError = 0.0; ///< fractional distance error, 0.02 is 2%
        double headingBiasRadPerS = 0.0; ///< constant heading drift (rad/s)
        double randomWalkIn = 0.0; ///< position noise std dev added per step (in)
    };

    /**
     * @brief Constructs the mock. Nothing is published until the first step().
     *
     * @param drift How odom_ekf_ drifts from truth.
     * @param odomRateHz odom_ekf_ publish rate (A2).
     * @param fusionGain Fraction of the way main_ekf_ moves toward each measurement, 0 to 1.
     * @param seed Random walk seed, fixed so tests repeat.
     */
    MockEkf(const DriftConfig& drift, double odomRateHz = MOCK_EKF_ODOM_RATE_HZ, double fusionGain = 0.5, uint32_t seed = 0);

    /**
     * @brief Feeds the next ground-truth pose, from the simulator (#152) or a log.
     *
     * @param truth Ground-truth field pose and its time. Times must increase.
     */
    void step(const StampedPose& truth);

    // EkfInterface
    void onOdomPose(PoseCallback cb) override;
    std::optional<StampedPose> mainPoseAt(int64_t tUs) const override;
    StampedPose mainPoseLatest() const override;
    void submitLidarMeasurement(const LidarMeasurement& m) override;
    void onPoseReset(PoseCallback cb) override;

    /**
     * @brief Every measurement received, oldest first.
     *
     * @return const std::vector<LidarMeasurement>& The submitted measurements.
     */
    const std::vector<LidarMeasurement>& submitted() const;

    /**
     * @brief Simulates auton start or setPose(): moves both poses to p and calls the reset callbacks.
     *
     * @param p New field pose and its time.
     */
    void firePoseReset(const StampedPose& p);

private:
    /**
     * @brief The fixed covariance put on every mock pose.
     *
     * @return Eigen::Matrix3d Diagonal from MOCK_EKF_POSE_SIGMA_IN and MOCK_EKF_POSE_SIGMA_RAD.
     */
    static Eigen::Matrix3d poseCov();

    DriftConfig m_drift; ///< odom drift settings
    double m_odomPeriodUs; ///< time between odom publishes (us)
    double m_fusionGain; ///< pull toward each measurement, 0 to 1

    bool m_started = false; ///< false until the first step()
    StampedPose m_lastTruth; ///< truth from the previous step()
    Pose m_odom{0.0, 0.0, 0.0}; ///< odom_ekf_ pose, truth plus drift
    Pose m_main{0.0, 0.0, 0.0}; ///< main_ekf_ pose
    int64_t m_nextPublishUs = 0; ///< time of the next odom publish (us)

    std::vector<PoseCallback> m_odomCallbacks; ///< onOdomPose() subscribers
    std::vector<PoseCallback> m_resetCallbacks; ///< onPoseReset() subscribers
    std::vector<LidarMeasurement> m_submitted; ///< every submitted measurement
    std::deque<StampedPose> m_history; ///< main_ekf_ poses from the last MOCK_EKF_HISTORY_US, oldest first

    std::mt19937 m_rng; ///< random walk generator
    std::normal_distribution<double> m_noise{0.0, 1.0}; ///< standard normal for the random walk
};
