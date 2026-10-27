/** @file robot.hpp
 *  @brief Public lifecycle interface for the V5 Brain control system.
 */

#pragma once

#include "brain/types.hpp"

#include <memory>

namespace brain {

/** @brief Owns and coordinates the components that make up the Brain runtime. */
class Robot {
public:
    /** @brief Construct the runtime facade and its component storage. */
    Robot();
    /** @brief Stop runtime activity and release owned components. */
    ~Robot();

    /** @brief Copy construction is disabled because Robot owns runtime components. */
    Robot(const Robot&) = delete;
    /** @brief Copy assignment is disabled because Robot owns runtime components. */
    Robot& operator=(const Robot&) = delete;

    /** @brief Initialize hardware-facing components and start required services. */
    void initialize();
    /** @brief Set the current control mode.
     *  @param mode Mode to apply to the runtime.
     */
    void setMode(RobotMode mode);
    /** @brief Run the requested PROS competition mode.
     *  @param mode Competition mode selected by the PROS callback.
     */
    void runCompetitionMode(RobotMode mode);
    /** @brief Stop runtime tasks and place outputs in their safe state. */
    void shutdown();

private:
    /// Private implementation type that owns the runtime's concrete components.
    class Components;
    /// Owned implementation, hidden to keep the public interface independent of wiring.
    std::unique_ptr<Components> components_;
};

} // namespace brain
