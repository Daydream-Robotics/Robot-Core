#include "main.h"

#include "autonomous/autonomous.hpp"
#include "config/subsystems.hpp"
#include "subsystems/drivetrain.hpp"
#include "subsystems/pneumatics.hpp"

void initialize() {
    pros::lcd::initialize();

    drivetrain.initialize();
    pneumatics.initialize();

    imu.reset();
    while (imu.is_calibrating()) {
        pros::delay(20);
    }

    


    // object handler setup
	// pros::Task frame_task(UpdateFrame_task_fn, (void*)"PROS_Task_Param", TASK_PRIORITY_DEFAULT, TASK_STACK_DEPTH_DEFAULT, "Vision Frame Update");
	// odom in background 
	pros::Task odom_task(Odometry::odomTask, TASK_PRIORITY_DEFAULT + 1, TASK_STACK_DEPTH_DEFAULT, "Odom Task");
	printf("[MAIN] Initialize complete\n");
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
