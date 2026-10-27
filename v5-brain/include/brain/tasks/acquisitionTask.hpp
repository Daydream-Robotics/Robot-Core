/** @file acquisitionTask.hpp
 *  @brief Periodic task that gathers Brain input and publishes BrainState.
 */

#pragma once

#include "brain/mailboxes.hpp"
#include "brain/control/driverInputReader.hpp"
#include "brain/sensors/sensorService.hpp"

#include "pros/rtos.hpp"

#include <memory>

namespace brain {

/** @brief Periodically combines physical sensor and controller input.
 *
 * The task is the aggregation boundary: it obtains a SensorSnapshot from
 * SensorService, reads DriverInput separately, samples the existing battery
 * telemetry field in millivolts, and publishes the resulting BrainState.
 * It does not define a new statusFlags policy.
 */
class AcquisitionTask {
public:
    /** @brief Bind the input sources and destination mailbox.
     *  @param sensors Sensor sampler used for physical device readings.
     *  @param driverInput Reader for the master controller.
     *  @param states Mailbox receiving each assembled BrainState.
     *  @note Referenced objects must outlive this task.
     */
    AcquisitionTask(SensorService& sensors,
                    DriverInputReader& driverInput,
                    BrainStateMailbox& states);

    /** @brief Start periodic state acquisition. */
    void start();
    /** @brief Stop acquisition and wait for its task to exit. */
    void stop();

private:
    /** @brief Perform the acquisition loop and publish assembled state. */
    void run();

    /// Sensor source used to produce the physical sensor portion of BrainState.
    SensorService& sensors_;
    /// Controller source used to produce the operator-input portion of BrainState.
    DriverInputReader& driverInput_;
    /// Destination used to share each completed BrainState with other tasks.
    BrainStateMailbox& states_;
    /// PROS task handle; empty when the acquisition loop is stopped.
    std::unique_ptr<pros::Task> task_;
};

} // namespace brain
