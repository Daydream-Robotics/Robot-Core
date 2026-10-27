#pragma once

#include "brain/mailboxes.hpp"
#include "brain/control/driverInputReader.hpp"
#include "brain/sensors/sensorService.hpp"

#include "pros/rtos.hpp"

#include <memory>

namespace brain {

class AcquisitionTask {
public:
    AcquisitionTask(SensorService& sensors,
                    DriverInputReader& driverInput,
                    BrainStateMailbox& states);

    void start();
    void stop();

private:
    void run();

    SensorService& sensors_;
    DriverInputReader& driverInput_;
    BrainStateMailbox& states_;
    std::unique_ptr<pros::Task> task_;
};

} // namespace brain
