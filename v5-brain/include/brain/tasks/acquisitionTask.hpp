/** @file acquisitionTask.hpp
 *  @brief Periodic task that gathers Brain input and publishes BrainState.
 */

#pragma once

#include <atomic>
#include <memory>

#include "pros/rtos.hpp"

#include "brain/control/driverInputReader.hpp"
#include "brain/localization/backupOdometry.hpp"
#include "brain/mailboxes.hpp"
#include "brain/sensors/sensorService.hpp"

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
     *  @param odometry Brain-local tracker updated once per acquisition.
     *  @note Referenced objects must outlive this task.
     */
    AcquisitionTask(SensorService& sensors,
                    DriverInputReader& driverInput,
                    BrainStateMailbox& states,
                    BackupOdometry& odometry);

    /** @brief Stop the acquisition task before releasing its dependencies. */
    ~AcquisitionTask();

    /** @brief Start periodic state acquisition. */
    void start();
    /** @brief Stop acquisition and wait for its task to exit. */
    void stop();

private:
    /** @brief Perform the acquisition loop and publish assembled state. */
    void run();

    /// Sensor source used to produce the physical sensor portion of BrainState.
    SensorService& m_sensors;
    /// Controller source used to produce the operator-input portion of BrainState.
    DriverInputReader& m_driverInput;
    /// Destination used to share each completed BrainState with other tasks.
    BrainStateMailbox& m_states;
    /// Local tracker updated with each sensor sample.
    BackupOdometry& m_odometry;
    /// Whether acquisition should continue running.
    std::atomic<bool> m_running{false};
    /// Whether the acquisition loop has exited.
    std::atomic<bool> m_exited{true};
    /// PROS task handle; empty when the acquisition loop is stopped.
    std::unique_ptr<pros::Task> m_task;
};

} // namespace brain
