/** @file nanoLink.hpp
 *  @brief Brain hardware and link boundary for the shared serial protocol.
 */
#pragma once

#include "brain/configuration.hpp"
#include "brain/types.hpp"

namespace brain {

/** @brief Hardware operations used by the protocol link.
 *
 * The runtime owns the devices. Calls are serialized by the link; implementations
 * must synchronize with any other task that accesses those devices. A failed
 * configuration must leave the previous configuration active. The robot geometry
 * in BrainConfiguration is local data and is not carried by CONFIG packets.
 */
class LinkHardware {
public:
    virtual ~LinkHardware() = default;

    /// Validate and apply the boot or received configuration, resetting sensor origins on success.
    virtual packets::Error applyConfiguration(const BrainConfiguration& config) = 0;
    /// Sample continuous sensor positions, controller input, battery and hardware status.
    virtual BrainState sample() = 0;
    /// Apply motor and ADI outputs from a validated command; do not execute its one-shot request.
    virtual packets::Error applyOutputs(const NanoCommand& command) = 0;
    /// Execute a validated one-shot request and report its result to the link.
    virtual packets::Error executeRequest(const packets::CommandFixed& command) = 0;
    /// Brake configured motor groups when the autonomous watchdog expires.
    virtual void brakeGroups() = 0;
};

} // namespace brain
