/**
 * @file trackingWheel.cpp
 * @brief Implementation of encoder-to-distance conversion for V5 tracking wheels.
 * @details Converts angular readings from @ref brain::Encoder into linear travel
 *          distances using wheel circumference derived from @ref shared::TrackingWheelGeometry.
 */

#include "brain/sensors/trackingWheel.hpp"

#include "brain/sensors/encoder.hpp"

namespace brain {

namespace {

/**
 * @brief Convert angular encoder ticks into linear travel distance.
 * @param ticks Angular displacement in PROS rotation ticks (centidegrees).
 * @param wheelDiameter Diameter of the tracking wheel in target length units.
 * @param ticksPerRev Number of encoder ticks per full wheel revolution.
 * @return Linear travel distance in the same units as @p wheelDiameter.
 */
double ticksToDistance(double ticks, double wheelDiameter, double ticksPerRev) {
    // Guard against invalid configuration values to prevent division by zero.
    if (ticksPerRev <= 0.0 || wheelDiameter <= 0.0) {
        return 0.0;
    }

    // Convert raw ticks to full wheel revolutions, then multiply by circumference (pi * d).
    const double revolutions = ticks / ticksPerRev;
    const double circumference = 3.14159265358979323846 * wheelDiameter;
    return revolutions * circumference;
}

} // namespace

/**
 * @brief Construct a TrackingWheel bound to @p encoder and @p geometry.
 * @param encoder Non-owning reference to the rotation encoder.
 * @param geometry Physical wheel diameter and tracking-center offset.
 */
TrackingWheel::TrackingWheel(Encoder& encoder, shared::TrackingWheelGeometry geometry)
    : encoder_(encoder),
      geometry_(geometry) {
    // Synchronize cached distance values with the encoder's initial state on construction.
    update();
}

/**
 * @brief Recompute cached linear distance and delta from the underlying encoder.
 * @todo Planned feature: Handle disconnected sensors (@c !encoder_.connected()) by zeroing
 *       @ref deltaDistance_ and flagging a sensor fault.
 * @todo Planned feature: Apply external gear ratio scaling if non-1:1 gearing is added to
 *       @ref shared::TrackingWheelGeometry.
 */
void TrackingWheel::update() {
    // Convert total accumulated ticks and per-tick delta ticks into linear distance.
    distance_ = ticksToDistance(encoder_.getPosition(), geometry_.diameter, TICKS_PER_REVOLUTION);
    deltaDistance_ = ticksToDistance(encoder_.getDelta(), geometry_.diameter, TICKS_PER_REVOLUTION);
}

/**
 * @brief Return the cached accumulated linear distance.
 * @return Total distance traveled in the geometry's length units.
 */
double TrackingWheel::distance() const {
    return distance_;
}

/**
 * @brief Return the cached linear distance delta from the last update cycle.
 * @return Distance delta in the geometry's length units.
 */
double TrackingWheel::deltaDistance() const {
    return deltaDistance_;
}

/**
 * @brief Return the signed perpendicular offset of this tracking wheel.
 * @return Offset from the tracking center in the geometry's length units.
 */
double TrackingWheel::offset() const {
    return geometry_.offset;
}

} // namespace brain

