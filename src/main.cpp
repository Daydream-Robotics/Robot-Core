#include "main.h"

#include "daydream/autonomous/autonomous.hpp"
#include "daydream/config/subsystems.hpp"
#include "daydream/subsystems/drivetrain.hpp"
#include "daydream/subsystems/pneumatics.hpp"

namespace {

daydream::Drivetrain drivetrain(controller, leftMotors, rightMotors);
daydream::Pneumatics pneumatics(
    controller,
    ballBlocker,
    scoringLifter,
    matchloader,
    descorer);

} // namespace

void initialize() {
    pros::lcd::initialize();

    drivetrain.initialize();
    pneumatics.initialize();

    imu.reset();
    while (imu.is_calibrating()) {
        pros::delay(20);
    }
}

void disabled() {}

void competition_initialize() {}

void autonomous() {
    Autonomous auton;
    pneumatics.setMatchloader(true);
    auton.travel(24.0, 60.0, 0.0, 3.0);
    pneumatics.setMatchloader(false);
}

void opcontrol() {
    pros::Task drivetrainTask(
        [] { drivetrain.control(); },
        TASK_PRIORITY_DEFAULT + 4,
        TASK_STACK_DEPTH_DEFAULT,
        "Drivetrain");
    pros::Task pneumaticsTask(
        [] { pneumatics.control(); },
        TASK_PRIORITY_DEFAULT + 2,
        TASK_STACK_DEPTH_DEFAULT,
        "Pneumatics");

    while (true) {
        pros::delay(300);
    }
}
