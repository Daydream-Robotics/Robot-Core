/** @file robotConfig.hpp
 *  @brief Platform-neutral physical measurements shared by Brain and Nano.
 */

#pragma once

namespace shared {

/** @brief Physical dimensions and placement of one tracking wheel.
 *
 * Diameter and offset use the same length unit as the rest of RobotConfig.
 */
struct TrackingWheelGeometry {
    /// Wheel diameter used to convert encoder rotation into travel distance.
    double diameter;
    /// Signed perpendicular distance from the tracking center to the wheel.
    double offset;
};

/** @brief Robot dimensions required by shared Brain and Nano calculations. */
struct RobotConfig {
    /// Diameter of the drivetrain wheel.
    double driveWheelDiameter;
    /// Ratio between motor-shaft and output-wheel rotation.
    double driveGearRatio;
    /// Distance between the left and right drivetrain tracking paths.
    double trackWidth;
    /// Geometry of the wheel measuring travel parallel to the robot centerline.
    TrackingWheelGeometry parallelTrackingWheel;
    /// Geometry of the wheel measuring travel perpendicular to the centerline.
    TrackingWheelGeometry perpendicularTrackingWheel;
};

} // namespace shared
