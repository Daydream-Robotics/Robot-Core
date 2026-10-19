#include "main.h"

#include "daydream/autonomous/autonomous.hpp"
#include "daydream/config/subsystems.hpp"
#include "daydream/motion/control/mpcSerial.hpp"
#include "daydream/motion/pathFollower.hpp"
#include "daydream/motion/pathing/paths.hpp"
#include "daydream/subsystems/drivetrain.hpp"
#include "daydream/subsystems/pneumatics.hpp"

namespace {

MPCSerial* mpc_serial_controller = nullptr;
PathFollower* pathFollower = nullptr;

} // namespace

void initialize() {
    pros::lcd::initialize();

    drivetrain.initialize();
    pneumatics.initialize();

    imu.reset();
    while (imu.is_calibrating()) {
        pros::delay(20);
    }

    printf("[MAIN] Loading paths...\n");
	paths = Path::buildAllPathsFromJerryIO("/usd/path.jerryio.txt");
	printf("[MAIN] Paths loaded: %zu\n", paths.size());

	if (paths.size() != PathName::COUNT) {
		printf("[MAIN] ERROR: Path count mismatch\n");
		pros::lcd::print(1, "EsRROR: Path count mismatch");
		pros::lcd::print(2, "Expected: %d, Actual: %zu", PathName::COUNT, paths.size());
	} else {
		printf("[MAIN] Trajectories Loaded successfully\n");
		pros::lcd::print(1, "Trajectories Loaded");
	}

    


    // period, track width, motor a/b, a_max come from nano/include/mpcConfig.hpp (shared with the Nano)
    MPCSerial::Params mpc_serial_params(DRIVE_GEAR_RATIO);
    mpc_serial_controller = new MPCSerial(mpc_serial_params);
    pathFollower = new PathFollower(*mpc_serial_controller);

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
