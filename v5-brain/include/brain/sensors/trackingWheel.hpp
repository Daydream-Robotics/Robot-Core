/** @file trackingWheel.hpp
 *  @brief Converts a rotation encoder reading into tracking-wheel distance.
 */

#pragma once

#include "robotConfig.hpp"

namespace brain {

class Encoder;

/** @brief Associates an encoder with the geometry of a tracking wheel. */
class TrackingWheel {
public:
    /** @brief Bind an encoder to its wheel geometry.
     *  @param encoder Encoder used to measure wheel rotation.
     *  @param geometry Wheel diameter and offset from the robot's tracking center.
     *  @note The referenced encoder must outlive this tracking wheel.
     */
    TrackingWheel(Encoder& encoder, shared::TrackingWheelGeometry geometry);

    /** @brief Refresh the wheel's cached distance and distance delta. */
    void update();
    /** @brief Get accumulated travel distance from the wheel.
     *  @return Distance in the geometry's length units.
     */
    double distance() const;
    /** @brief Get travel distance since the previous update.
     *  @return Distance delta in the geometry's length units.
     */
    double deltaDistance() const;

private:
    /// Non-owning encoder whose rotation is converted to distance.
    Encoder& encoder_;
    /// Diameter and signed offset used for the distance conversion.
    shared::TrackingWheelGeometry geometry_;
};

} // namespace brain
