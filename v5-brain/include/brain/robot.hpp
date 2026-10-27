#pragma once

#include "brain/communication.hpp"
#include "brain/control.hpp"
#include "brain/diagnostics.hpp"
#include "brain/localization.hpp"
#include "brain/tasks.hpp"
#include "brain/types.hpp"

#include <memory>

namespace brain {

class Robot {
public:
    Robot();
    ~Robot();

    Robot(const Robot&) = delete;
    Robot& operator=(const Robot&) = delete;

    void initialize();
    void setMode(RobotMode mode);
    void runCompetitionMode(RobotMode mode);
    void shutdown();

private:
    class Components;
    std::unique_ptr<Components> components_;
};

} // namespace brain
