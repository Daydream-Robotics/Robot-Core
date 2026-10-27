/** @file motorOutput.hpp
 *  @brief Interface for applying motor-group commands to PROS motor groups.
 */

#pragma once

#include "brain/types.hpp"

#include "pros/motor_group.hpp"

#include <cstddef>

namespace brain {

/** @brief Applies abstract motor commands to the configured physical groups. */
class MotorOutput {
public:
    /** @brief Bind the output adapter to its motor groups.
     *  @param groups Non-owning array of pointers to motor groups.
     *  @param groupCount Number of entries in @p groups.
     *  @note The array and referenced motor groups must outlive this adapter.
     */
    MotorOutput(pros::MotorGroup* const* groups, std::size_t groupCount);

    /** @brief Apply each valid group command to its corresponding motor group.
     *  @param command Command to apply.
     */
    void apply(const MotorCommand& command);
    /** @brief Stop all configured motor groups. */
    void stop();

private:
    /// Non-owning array of motor groups controlled by this adapter.
    pros::MotorGroup* const* groups_;
    /// Number of valid pointers in @ref groups_.
    std::size_t groupCount_;
};

} // namespace brain
