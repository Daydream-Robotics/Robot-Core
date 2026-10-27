/** @file driverInputReader.hpp
 *  @brief Adapter that converts a PROS controller reading into DriverInput.
 */

#pragma once

#include "brain/types.hpp"

#include "pros/misc.hpp"

namespace brain {

/** @brief Reads operator controls from the configured master controller. */
class DriverInputReader {
public:
    /** @brief Bind the reader to a PROS controller.
     *  @param controller Controller whose axes, buttons, and connection state are read.
     *  @note The referenced controller must outlive this reader.
     */
    explicit DriverInputReader(pros::Controller& controller);

    /** @brief Read the controller's current state.
     *  @return Axes, button mask, and connection status represented as DriverInput.
     */
    DriverInput read() const;

private:
    /// Non-owning reference to the controller queried by @ref read.
    pros::Controller& controller_;
};

} // namespace brain
