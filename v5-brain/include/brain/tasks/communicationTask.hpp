/** @file communicationTask.hpp
 *  @brief Periodic task that exchanges Brain state and Nano commands.
 */

#pragma once

#include "brain/communication/nanoLink.hpp"
#include "protocol/serial.hpp"

namespace brain {

/** @brief Controls the lifecycle of the serial link service. */
class CommunicationTask {
public:
    /** @brief Bind the link service.
     *  @param link Protocol service; it must outlive this object.
     */
    explicit CommunicationTask(SerialLink& link);

    /**
     * @brief Starts the service.
     * @return bool True when startup succeeds.
     */
    bool start();
    /** @brief Stop the service. */
    void stop();

private:
    /// Non-owning protocol service, responsible for its own communication tasks.
    SerialLink& m_link;
};

} // namespace brain
