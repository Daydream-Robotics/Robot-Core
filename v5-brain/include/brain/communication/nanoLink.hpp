/** @file nanoLink.hpp
 *  @brief Brain-side adapter for exchanging state and commands with the Nano.
 */

#pragma once

#include "brain/configuration.hpp"
#include "brain/types.hpp"
#include "serial/serialProtocol.hpp"

namespace brain {

/** @brief Result of one serial state/command exchange. */
struct LinkExchange {
    /// True when a valid Nano command was received for this exchange.
    bool commandReceived = false;
    /// Received command; inspect @ref commandReceived and its @c valid field first.
    NanoCommand command{};
    /// Protocol error reported by the serial operation, if any.
    packets::Error error = packets::Error::NONE;
};

/** @brief Encodes Brain state and decodes Nano responses over SerialProtocol. */
class NanoLink {
public:
    /** @brief Bind the link adapter to a serial protocol instance.
     *  @param serial Serial transport used for all link operations.
     *  @note The referenced protocol instance must outlive this adapter.
     */
    explicit NanoLink(SerialProtocol& serial);

    /** @brief Send robot configuration to the Nano.
     *  @param config Robot and protocol configuration to transmit.
     *  @return True when the configuration exchange succeeds.
     */
    bool configure(const BrainConfiguration& config);
    /** @brief Exchange one Brain state update for a Nano motor command.
     *  @param state Latest state snapshot to send.
     *  @return Result containing command-reception and protocol-error information.
     */
    LinkExchange exchange(const BrainState& state);

private:
    /// Non-owning serial protocol object used to perform transport I/O.
    SerialProtocol& serial_;
};

} // namespace brain
