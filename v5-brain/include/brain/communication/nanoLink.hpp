/** @file nanoLink.hpp
 *  @brief Brain hardware and link boundary for the shared serial protocol.
 */
#pragma once

#include "brain/configuration.hpp"
#include "brain/types.hpp"
#include "pros/adi.hpp"
#include "pros/misc.hpp"
#include "pros/motor_group.hpp"
#include "pros/rotation.hpp"
#include "pros/rtos.hpp"
#include <array>
#include <vector>

namespace brain {

/** @brief Provides serial access to existing Brain devices without owning them.
 *
 * All supplied devices must outlive this object.
 * The owner must coordinate access from other tasks with serial activity.
 */
class NanoLink {
public:
    /**
     * @brief Binds the devices used by the serial link.
     * @param controller Master controller owned by the caller.
     * @param groups Motor groups in protocol slot order; entries must not be null.
     * @param wheels Rotation sensors in protocol slot order; entries must not be null.
     * @param outputs ADI outputs by protocol slot; unused entries may be null.
     */
    NanoLink(pros::Controller& controller,
             const std::vector<pros::MotorGroup*>& groups,
             const std::vector<pros::Rotation*>& wheels,
             const std::array<pros::adi::DigitalOut*, packets::MAX_ADI>& outputs);
    /** @brief Releases serial bookkeeping without destroying the supplied devices. */
    ~NanoLink();
    /** @brief Copy construction is disabled for the device binding. */
    NanoLink(const NanoLink&) = delete;
    /** @brief Copy assignment is disabled for the device binding. */
    NanoLink& operator=(const NanoLink&) = delete;
    /**
     * @brief Checks the supplied motor and rotation bindings before use.
     * @return bool True when counts are supported and no group or wheel is null.
     */
    bool bindingsValid() const;
    /**
     * @brief Applies settings to the supplied devices after validating their ports.
     * @param config Configuration decoded from the shared protocol.
     * @return packets::Error NONE on success, otherwise the reason for rejection.
     * @note Device replacement and coordination with other users belong to the owner.
     */
    packets::Error applyConfiguration(const BrainConfiguration& config);
    /**
     * @brief Samples devices and accumulates positions in radians.
     * @return BrainState The sampled state, or a default state for invalid bindings.
     */
    BrainState sample();
    /**
     * @brief Applies motor and ADI outputs from a validated Nano command.
     * @param command Command carrying one group entry per configured group.
     * @return packets::Error NONE on success, otherwise the device or count error.
     */
    packets::Error applyOutputs(const NanoCommand& command);
    /**
     * @brief Executes one controller or motor-group request.
     * @param command Shared command fields containing the request.
     * @return packets::Error NONE on success, otherwise the request error.
     */
    packets::Error executeRequest(const packets::CommandFixed& command);
    /** @brief Brakes the bound motor groups while holding the device mutex. */
    void brakeGroups();
private:
    /** @brief Brakes bound groups; the caller must already hold the device mutex. */
    void brakeGroupsUnlocked();
    /** @brief Non-owning motor binding and its accumulated encoder state. */
    struct Group {
        pros::MotorGroup* motor = nullptr; ///< Motor group supplied by the owner.
        std::vector<double> last; ///< Last valid motor positions in degrees.
        std::vector<bool> valid; ///< Whether each previous position read succeeded.
        double position = 0; ///< Accumulated group position in radians.
        pros::MotorGears gear = pros::MotorGears::green; ///< Gear setting restored on reconnection.
        pros::MotorBrake brakeMode = pros::MotorBrake::coast; ///< Brake setting restored on reconnection.
        bool missing = false; ///< Whether a configured motor was missing at the last check.
    };
    /** @brief Non-owning rotation binding and its accumulated position. */
    struct Wheel {
        pros::Rotation* sensor = nullptr; ///< Rotation sensor supplied by the owner.
        double last = 0; ///< Last valid sensor reading in radians.
        double position = 0; ///< Accumulated position in radians.
        bool valid = false; ///< Whether the previous reading succeeded.
        bool reversed = false; ///< Direction restored on reconnection.
        bool missing = false; ///< Whether the sensor was missing at the last check.
    };
    pros::Controller& controller_; ///< Non-owning master controller reference.
    std::vector<Group> groups_; ///< Non-owning motor groups by protocol slot.
    std::vector<Wheel> wheels_; ///< Non-owning rotation sensors by protocol slot.
    std::array<pros::adi::DigitalOut*, packets::MAX_ADI> outputs_{}; ///< Non-owning ADI outputs by slot.
    uint16_t enabled_ = 0; ///< Enabled output slots.
    uint16_t activeLow_ = 0; ///< Slots whose logical state is inverted.
    uint16_t level_ = 0; ///< Last physical output level by slot.
    uint8_t expander_ = 0; ///< Configured ADI expander port, or zero.
    bool expanderMissing_ = false; ///< Whether the expander was missing at the last check.
    pros::Mutex mutex_; ///< Serializes access through this object only.
};

} // namespace brain
