/** @file encoder.hpp
 *  @brief Cached interface for a PROS rotation sensor used as an encoder.
 */

#pragma once

#include "pros/rotation.hpp"

#include <cstdint>

namespace brain {

/** @brief Reads and tracks position changes from one rotation sensor. */
class Encoder {
public:
    /** @brief Construct an encoder adapter on a V5 smart port.
     *  @param smartPort PROS smart-port number for the rotation sensor.
     *  @param reversed Whether to invert the sensor's reported direction.
     */
    explicit Encoder(std::int8_t smartPort, bool reversed = false);

    /** @brief Refresh the cached sensor reading and position delta. */
    void update();
    /** @brief Get the cached absolute sensor position in PROS rotation ticks.
     *  @return Position since the sensor was last tared or reset.
     */
    double position() const;
    /** @brief Get the cached position change since the previous update.
     *  @return Position delta in PROS rotation ticks.
     */
    double delta() const;
    /** @brief Check whether the sensor is connected and providing readings.
     *  @return True when the most recent sensor status indicates a connection.
     */
    bool connected() const;
    /** @brief Reset the encoder's position reference to zero. */
    void tare();

private:
    /// PROS rotation sensor used to acquire encoder readings.
    pros::Rotation device_;
};

} // namespace brain
