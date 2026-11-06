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

/** @brief Protocol link service implemented using the shared SerialLink.
 *
 * The service applies CONFIG through LinkHardware, receives COMMAND, and sends
 * STATE and LOG. It owns protocol bookkeeping, including command validation,
 * request deduplication, watchdog timing, sequence numbers and acknowledged
 * sensor-position history. It converts sampled cumulative radians to wire deltas
 * from the acknowledged baseline. An implementation requires the existing
 * SerialLink to be refactored to use LinkHardware instead of owning devices.
 */
class NanoLink {
public:
    virtual ~NanoLink() = default;
    /// Open the transport, apply the boot configuration, and start the link.
    virtual bool start() = 0;
    /// Stop link activity before destroying its hardware owner.
    virtual void stop() = 0;
    /// Enable Nano outputs in autonomous; disable them for local driver control.
    virtual void setNanoControl(bool enabled) = 0;
};

} // namespace brain
