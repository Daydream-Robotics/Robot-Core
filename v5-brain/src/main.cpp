#include "main.h"
#include "brain/robot.hpp"

static brain::Robot robot;

void initialize() {
    robot.initialize();
}

void disabled() {
    robot.setMode(brain::RobotMode::Disabled);
}

void competition_initialize() {}

void autonomous() {
    robot.runCompetitionMode(brain::RobotMode::Autonomous);
}

void opcontrol() {
    robot.runCompetitionMode(brain::RobotMode::Manual);
}
