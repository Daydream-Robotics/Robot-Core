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

    odom.initialize();
    pros::Task::create(
        [] { odom.control(); },
        TASK_PRIORITY_DEFAULT + 1,
        TASK_STACK_DEPTH_DEFAULT,
        "Odometry");
}

void disabled() {
    drivetrain.stop();
}

void competition_initialize() {}

void autonomous() {
    drivetrain.stop();
    odom.setPose({0.0, 0.0, 0.0});

    Autonomous auton(drivetrain, odom, pneumatics);
    auton.runExample();
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

    while (!pros::competition::is_disabled() && !pros::competition::is_autonomous()) {
        pros::delay(300);
    }
}
