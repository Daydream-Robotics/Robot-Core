#include "main.h"

#include "daydream/autonomous/runner.hpp"
#include "daydream/config/subsystems.hpp"
#include "daydream/subsystems/drivetrain.hpp"
#include "daydream/subsystems/intake.hpp"
#include "daydream/subsystems/logging.hpp"
#include "daydream/subsystems/pneumatics.hpp"

/**
 * @file main.cpp
 * @brief PROS competition callbacks and application-owned subsystem instances.
 * @details The callbacks share function-local static subsystem objects so
 * autonomous and operator control act on the same hardware and services.
 */
namespace {

daydream::Drivetrain& drivetrain() {
	static daydream::Drivetrain instance(leftMotors, rightMotors);
	return instance;
}

daydream::Intake& intake() {
	static daydream::Intake instance;
	return instance;
}

daydream::Pneumatics& pneumatics() {
	static daydream::Pneumatics instance(ballBlocker, scoringLifter, matchloader, descorer);
	return instance;
}

daydream::Logging& logging() {
	static daydream::Logging instance;
	return instance;
}

void startOdometryTask() {
	// The odometry service is shared by autonomous and operator control.
	static pros::Task task(
		Odometry::odomTask,
		TASK_PRIORITY_DEFAULT + 1,
		TASK_STACK_DEPTH_DEFAULT,
		"Odometry"
	);
	static_cast<void>(task);
}

}  // namespace

/** @brief Initializes shared subsystems and starts background services. */
void initialize() {
	drivetrain().initialize();
	intake().initialize();
	pneumatics().initialize();
	logging().initialize();
	startOdometryTask();
}

/** @brief PROS disabled-mode callback. */
void disabled() {}

/** @brief PROS pre-autonomous initialization callback. */
void competition_initialize() {}

/** @brief Runs autonomous with the same subsystem instances used by opcontrol. */
void autonomous() {
	daydream::autonomous::run(drivetrain(), intake(), pneumatics(), odom);
}

/** @brief Runs the operator-control loop until the competition mode changes. */
void opcontrol() {
	while (true) {
		drivetrain().driveFromController(controller);
		intake().updateFromController(controller);
		pneumatics().updateFromController(controller);
		pros::delay(20);
	}
}
