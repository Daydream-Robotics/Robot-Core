/** @file odometry.hpp
 *  @brief Platform-neutral pose integration for differential tracking-wheel data.
 */

#pragma once

#include "robotConfig.hpp"

namespace shared {

/** @brief Robot position and heading in the configured odometry coordinate frame. */
struct Pose {
    /// Position along the frame's x-axis, in RobotConfig length units.
    double x;
    /// Position along the frame's y-axis, in RobotConfig length units.
    double y;
    /// Robot heading in radians, using the shared coordinate convention.
    double heading;
};

/** @brief Wheel and heading deltas consumed by one odometry update. */
struct OdometrySample {
    /// Parallel tracking-wheel change in RobotConfig length units.
    double parallelDelta;
    /// Perpendicular tracking-wheel change in RobotConfig length units.
    double perpendicularDelta;
    /// Heading change in radians, counter-clockwise positive.
    double headingDelta;
};

/** @brief Integrates sensor deltas into a platform-neutral robot pose. */
class Odometry {
public:
    /** @brief Construct an odometry integrator using robot geometry.
     *  @param config Robot dimensions used to interpret tracking-wheel deltas.
     */
    explicit Odometry(const RobotConfig& config);

    /** @brief Integrate a new sample into the current pose.
     *  @param sample Parallel, perpendicular, and heading changes to apply.
     *  @return Updated pose after applying @p sample.
     */
    Pose update(const OdometrySample& sample);
    /** @brief Replace the current pose with a supplied pose.
     *  @param pose New odometry state.
     */
    void reset(const Pose& pose);
    /** @brief Read the current integrated pose.
     *  @return Current pose.
     */
    Pose pose() const;

private:
    /// Robot geometry used to interpret wheel offsets and diameters.
    RobotConfig config_;
    /// Last integrated pose.
    Pose pose_{};
};

} // namespace shared
