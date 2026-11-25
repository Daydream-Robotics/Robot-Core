#pragma once

#include <cstdint>
#include <functional>
#include <optional>

#include <Eigen/Dense>

#include "localization/ekf/se2.hpp"
#include "localization/odometry.hpp"

/**
 * @brief Absolute pose from the LiDAR, sent to main_ekf_. Mirrors FullEkf's PoseMeasurement.
 */
struct LidarMeasurement {
    int64_t tUs = 0; ///< sweep-end time, NOT arrival time, Nano steady_clock (us) (A1, A9)
    Pose pose{0.0, 0.0, 0.0}; ///< field frame (A6)
    Eigen::Matrix3d cov = Eigen::Matrix3d::Zero(); ///< covariance of pose (x, y, θ order, in² and rad²) (A8)
    uint32_t healthFlags = 0; ///< LiDAR health bits for logging, not used by the EKF
};

/**
 * @brief Every EKF dependency of the LiDAR code goes through this.
 *
 * The real adapter wraps OdomEkf and FullEkf. MockEkf stands in for tests and the simulator.
 *
 * @see docs/LIDAR_EKF_ASSUMPTIONS.md for what A1 to A12 mean.
 */
class EkfInterface {
public:
    using PoseCallback = std::function<void(const StampedPose&)>;

    virtual ~EkfInterface() = default;

    /**
     * @brief Subscribes to odom_ekf_ poses. Every sample is delivered, not only the latest (A2, A3).
     *
     * odom_ekf_ never includes LiDAR measurements (A4).
     *
     * @param cb Called with each odom_ekf_ pose, in the odometry frame.
     */
    virtual void onOdomPose(PoseCallback cb) = 0;

    /**
     * @brief main_ekf_ pose at a past time (A1, A7).
     *
     * @param tUs Time on the Nano's steady_clock (us).
     * @return std::optional<StampedPose> Nothing if tUs is outside main_ekf_'s history.
     */
    virtual std::optional<StampedPose> mainPoseAt(int64_t tUs) const = 0;

    /**
     * @brief Latest main_ekf_ pose, the fallback when mainPoseAt() has nothing (A7).
     *
     * @return StampedPose Field-frame pose, its time, and covariance.
     */
    virtual StampedPose mainPoseLatest() const = 0;

    /**
     * @brief Sends a LiDAR pose to main_ekf_. It may arrive after tUs (A8, A9).
     *
     * @param m Field-frame pose stamped at sweep end.
     */
    virtual void submitLidarMeasurement(const LidarMeasurement& m) = 0;

    /**
     * @brief Subscribes to pose resets, such as auton start or setPose() (A10).
     *
     * @param cb Called with the new field pose.
     */
    virtual void onPoseReset(PoseCallback cb) = 0;
};
