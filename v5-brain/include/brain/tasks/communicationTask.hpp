/** @file communicationTask.hpp
 *  @brief Periodic task that exchanges Brain state and Nano commands.
 */

#pragma once

#include "brain/communication/nanoLink.hpp"

namespace brain {

/** @brief Controls the lifecycle of a NanoLink service. */
class CommunicationTask {
public:
    /** @brief Bind the link service.
     *  @param link Protocol service; it must outlive this object.
     */
    explicit CommunicationTask(NanoLink& link);

    /** @brief Start the service; false if startup failed. */
    bool start();
    /** @brief Stop the service. */
    void stop();

private:
    /// Non-owning protocol service, responsible for its own communication tasks.
    NanoLink& link_;
};

} // namespace brain
