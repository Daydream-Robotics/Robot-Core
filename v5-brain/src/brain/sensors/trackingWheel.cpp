/**
 * @file trackingWheel.cpp
 * @brief Implementation of encoder-to-distance conversion for V5 tracking wheels.
 * @details Converts angular readings from @ref brain::Encoder into linear travel
 *          distances using wheel CIRCUMFERENCE derived from @ref shared::TrackingWheelGeometry.
 */

#include "brain/sensors/trackingWheel.hpp"

#include "brain/sensors/encoder.hpp"

namespace brain {

/**
 * @brief Construct a TrackingWheel bound to @p encoder and @p geometry.
 * @param encoder Non-owning reference to the rotation encoder.
 * @param geometry Physical wheel diameter and tracking-center offset.
 */
TrackingWheel::TrackingWheel(Encoder& encoder, shared::TrackingWheelGeometry geometry)
    : m_encoder(encoder),
      m_geometry(geometry) {
}

/**
 * @brief Recompute cached linear distance and delta from the underlying encoder.
 * @todo Planned feature: Apply external gear ratio scaling if non-1:1 gearing is added to
 *       @ref shared::TrackingWheelGeometry.
 */
void TrackingWheel::update() {
    m_encoder.update();
    // Convert total accumulated ticks and per-tick delta ticks into linear distance.
    m_distance = ticksToDistance(m_encoder.getPosition(), m_geometry.diameter);
    m_deltaDistance = ticksToDistance(m_encoder.getDelta(), m_geometry.diameter);
}

double TrackingWheel::deltaRotation() const { return m_encoder.getDelta(); }
bool TrackingWheel::deltaValid() const { return m_encoder.deltaValid(); }

/**
 * @brief Return the cached accumulated linear distance.
 * @return Total distance traveled in the geometry's length units.
 */
double TrackingWheel::distance() const {
    return m_distance;
}

/**
 * @brief Return the cached linear distance delta from the last update cycle.
 * @return Distance delta in the geometry's length units.
 */
double TrackingWheel::deltaDistance() const {
    return m_deltaDistance;
}

/**
 * @brief Return the signed perpendicular offset of this tracking wheel.
 * @return Offset from the tracking center in the geometry's length units.
 */
double TrackingWheel::offset() const {
    return m_geometry.offset;
}

/**
 * @brief Convert angular encoder ticks into linear travel distance using @ref PI.
 * @param ticks Angular displacement in PROS rotation ticks (centidegrees).
 * @param wheelDiameter Diameter of the tracking wheel in target length units.
 * @return Linear travel distance in the same units as @p wheelDiameter.
 */
double TrackingWheel::ticksToDistance(double ticks, double wheelDiameter) {
    // Guard against invalid configuration values to prevent division by zero.
    if (TICKS_PER_REVOLUTION <= 0.0 || wheelDiameter <= 0.0) {
        return 0.0;
    }

    // Convert raw ticks to full wheel REVOLUTIONS, then multiply by CIRCUMFERENCE (PI * d).
    const double REVOLUTIONS = ticks / TICKS_PER_REVOLUTION;
    const double CIRCUMFERENCE = PI * wheelDiameter;
    return REVOLUTIONS * CIRCUMFERENCE;
}

} // namespace brain

