/** @file acquisitionTask.cpp
 *  @brief Periodic Brain sensor acquisition and local pose publication.
 */

#include "brain/tasks/acquisitionTask.hpp"

#include "pros/error.h"
#include "pros/misc.hpp"

namespace brain {

AcquisitionTask::AcquisitionTask(SensorService& sensors, DriverInputReader& driverInput,
                                 BrainStateMailbox& states, BackupOdometry& odometry)
    : m_sensors(sensors), m_driverInput(driverInput), m_states(states), m_odometry(odometry) {}

AcquisitionTask::~AcquisitionTask() { stop(); }

void AcquisitionTask::start() {
    if (m_running.exchange(true)) return;
    m_exited = false;
    m_task = std::make_unique<pros::Task>([this] { run(); }, TASK_PRIORITY_DEFAULT,
                                        TASK_STACK_DEPTH_DEFAULT, "brain acquisition");
}

void AcquisitionTask::stop() {
    if (!m_task) return;
    m_running = false;
    while (!m_exited) pros::delay(1);
    m_task.reset();
}

void AcquisitionTask::run() {
    constexpr std::uint32_t PERIOD_MS = 10;
    std::uint32_t wake = pros::millis();
    while (m_running) {
        BrainState state{};
        state.sensors = m_sensors.sample();
        state.driver = m_driverInput.read();
        const auto VOLTAGE = pros::battery::get_voltage();
        if (VOLTAGE != PROS_ERR) state.batteryVoltage = static_cast<std::uint16_t>(VOLTAGE);
        m_odometry.update(state.sensors);
        state.backupPose = m_odometry.pose();
        state.backupPoseValid = m_odometry.enabled() && m_odometry.valid();
        m_states.publish(state);
        pros::Task::delay_until(&wake, PERIOD_MS);
    }
    m_exited = true;
}

} // namespace brain
