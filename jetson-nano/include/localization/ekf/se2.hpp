#pragma once

#include <cstdint>

#include <Eigen/Dense>

#include "localization/odometry.hpp"

// SE(2) pose math shared by both EKFs and the LiDAR pipeline. Poses use the Pose frame from
// odometry.hpp (+X forward, +Y left, CCW positive), x/y in inches, theta in radians.
// Every returned theta is wrapped to (-pi, pi] with utils::wrapAngle.

namespace se2 {

/**
 * @brief a ∘ b: start at a, then move by b in a's frame.
 *
 * @param a Starting pose, in the world frame.
 * @param b Motion, in a's frame.
 * @return Pose The resulting pose, in the world frame.
 */
Pose compose(const Pose& a, const Pose& b);

/**
 * @brief The pose that undoes a, so compose(a, inverse(a)) is the identity.
 *
 * @param a Pose to invert.
 * @return Pose a⁻¹.
 */
Pose inverse(const Pose& a);

/**
 * @brief a⁻¹ ∘ b: motion from a to b, in a's frame, so compose(a, between(a, b)) == b.
 *
 * @param a Starting pose, in the world frame.
 * @param b Ending pose, in the world frame.
 * @return Pose Motion from a to b, in a's frame.
 */
Pose between(const Pose& a, const Pose& b);

/**
 * @brief Jacobian of compose(a, b) with respect to a.
 *
 * @param a Starting pose.
 * @param b Motion, in a's frame.
 * @return Eigen::Matrix3d d compose / d a, rows and columns in (x, y, theta) order.
 */
Eigen::Matrix3d composeJacA(const Pose& a, const Pose& b);

/**
 * @brief Jacobian of compose(a, b) with respect to b. Depends only on a's heading.
 *
 * @param a Starting pose.
 * @return Eigen::Matrix3d d compose / d b, rows and columns in (x, y, theta) order.
 */
Eigen::Matrix3d composeJacB(const Pose& a);

} // namespace se2

/**
 * @brief A pose with a timestamp and covariance (x, y, θ order, in² and rad²).
 */
struct StampedPose {
    int64_t tUs = 0; ///< Nano steady_clock time (us)
    Pose pose{0.0, 0.0, 0.0}; ///< pose at tUs
    Eigen::Matrix3d cov = Eigen::Matrix3d::Zero(); ///< covariance of pose (x, y, θ order, in² and rad²)
};

/**
 * @brief Relative motion between two times, in the frame of the earlier pose.
 */
struct PoseIncrement {
    int64_t t0Us = 0; ///< start time, Nano steady_clock (us)
    int64_t t1Us = 0; ///< end time, Nano steady_clock (us)
    Pose delta{0.0, 0.0, 0.0}; ///< motion from t0 to t1, in the t0 pose's frame
    Eigen::Matrix3d cov = Eigen::Matrix3d::Zero(); ///< covariance of delta (x, y, θ order, in² and rad²)
};
