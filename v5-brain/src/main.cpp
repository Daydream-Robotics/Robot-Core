/** @file main.cpp
 *  @brief Connects PROS competition callbacks to the Brain runtime facade.
 */

#include "main.h"
#include "brain/robot.hpp"

/// Process-lifetime Brain runtime used by all PROS competition callbacks.
static brain::Robot robot;

/** @brief Initialize the Brain runtime when the V5 program starts. */
void initialize() {
    robot.initialize();
}

/** @brief Notify the Brain runtime that competition control is disabled. */
void disabled() {
    robot.setMode(brain::RobotMode::Disabled);
}

/** @brief PROS hook for work before an autonomous period begins. */
void competition_initialize() {}

/** @brief Run the Brain runtime in autonomous mode. */
void autonomous() {
    robot.runCompetitionMode(brain::RobotMode::Autonomous);
}

/** @brief Run the Brain runtime in manual operator-control mode. */
void opcontrol() {
    robot.runCompetitionMode(brain::RobotMode::Manual);
}
