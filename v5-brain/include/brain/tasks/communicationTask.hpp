#pragma once

#include "brain/communication/nanoLink.hpp"
#include "brain/mailboxes.hpp"

#include "pros/rtos.hpp"

#include <memory>

namespace brain {

class CommunicationTask {
public:
    CommunicationTask(NanoLink& link,
                      BrainStateMailbox& states,
                      NanoCommandMailbox& commands);

    void start();
    void stop();

private:
    void run();

    NanoLink& link_;
    BrainStateMailbox& states_;
    NanoCommandMailbox& commands_;
    std::unique_ptr<pros::Task> task_;
};

} // namespace brain
