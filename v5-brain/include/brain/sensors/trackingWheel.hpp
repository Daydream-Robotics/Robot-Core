/**
 * @file trackingWheel.hpp
 * @brief Converts a rotation encoder reading into tracking-wheel linear distance.
 * @details Associates a non-owning @ref Encoder instance with @ref shared::TrackingWheelGeometry
 *          to calculate accumulated linear travel distance and per-update deltas.
 */

#pragma once

#include "robotConfig.hpp"

namespace brain {

class Encoder;

/**
 * @class TrackingWheel
 * @brief Associates an encoder with the geometry of a tracking wheel.
 * @todo Planned feature: Support external gear ratios between the tracking wheel and sensor
 *       once added to @ref shared::TrackingWheelGeometry.
 * @todo Planned feature: Expose sensor health/connection state and zero-distance reset helper.
 */
class TrackingWheel {
public:
    /**
     * @brief Bind an encoder to its wheel geometry.
     * @param encoder Encoder used to measure wheel rotation.
     * @param geometry Wheel diameter and offset from the robot's tracking center.
     * @note The referenced @p encoder must outlive this tracking wheel.
     */
    TrackingWheel(Encoder& encoder, shared::TrackingWheelGeometry geometry);

    /**
     * @brief Destroy the TrackingWheel instance.
     * @note Does not destroy or reset the underlying non-owning @ref encoder_.
     */
    ~TrackingWheel() = default;

    /**
     * @brief Refresh the wheel's cached distance and distance delta.
     * @details Reads the latest position and delta from @ref encoder_ and converts
     *          angular sensor ticks into linear distance using @ref geometry_.diameter.
     * @note Assumes @ref Encoder::update() has already been called for the current tick.
     */
    void update();

    /**
     * @brief Get accumulated travel distance from the wheel.
     * @return Total linear distance in the geometry's length units.
     */
    double distance() const;

    /**
     * @brief Get travel distance since the previous update.
     * @return Linear distance delta in the geometry's length units.
     */
    double deltaDistance() const;

    /**
     * @brief Get the wheel's signed perpendicular offset from the robot's tracking center.
     * @return Offset in the geometry's length units.
     */
    double offset() const;

private:
    /**
     * @brief Convert angular encoder ticks into linear travel distance.
     * @param ticks Angular displacement in PROS rotation ticks (centidegrees).
     * @param wheelDiameter Diameter of the tracking wheel in target length units.
     * @return Linear travel distance in the same units as @p wheelDiameter.
     */
    static double ticksToDistance(double ticks, double wheelDiameter);

    /// Number of PROS rotation sensor ticks (centidegrees) per full wheel revolution.
    static constexpr double TICKS_PER_REVOLUTION = 36000.0;

    /// Mathematical constant pi used for wheel circumference calculations.
    static constexpr double PI = 3.14159265358979323846;

    /// Non-owning encoder whose rotation is converted to distance.
    Encoder& encoder_;

    /// Diameter and signed offset used for the distance conversion.
    shared::TrackingWheelGeometry geometry_;

    /// Cached total linear travel distance updated on each @ref update() call.
    double distance_ = 0.0;

    /// Cached linear travel distance change since the previous @ref update() call.
    double deltaDistance_ = 0.0;
};

} // namespace brain


